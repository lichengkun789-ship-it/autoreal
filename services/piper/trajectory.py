"""Reusable joint-trajectory execution for Piper task and backend tools."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import time

import numpy as np

from .hold import PiperJointCommandAdapter
from .minimum_motion import _enabled
from .safety import DEFAULT_MAX_JOINT_STEP_RAD, validate_joint_target


PIPER_ZERO_FEEDBACK_DEADBAND_RAD = 0.012


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class TrajectorySettings:
    speed_percent: int = 2
    feedback_max_age_s: float = 0.25
    poll_interval_s: float = 0.05
    waypoint_timeout_s: float = 5.0
    settle_tolerance_rad: float = 0.002
    max_start_error_rad: float = 0.02
    joint_margin_rad: float = 0.02
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD

    def __post_init__(self):
        if type(self.speed_percent) is not int or not 1 <= self.speed_percent <= 5:
            raise ValueError("trajectory speed_percent must be in 1..5")
        for name in (
            "feedback_max_age_s",
            "poll_interval_s",
            "waypoint_timeout_s",
            "settle_tolerance_rad",
            "max_start_error_rad",
            "max_joint_step_rad",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(self.joint_margin_rad) or self.joint_margin_rad < 0:
            raise ValueError("joint_margin_rad must be non-negative and finite")


class PiperJointTrajectoryExecutor:
    """Command admitted joint waypoints with measured hold and reverse recovery."""

    def __init__(self, reader, settings: TrajectorySettings, *, sleeper=time.sleep):
        self.reader = reader
        self.settings = settings
        self.sleeper = sleeper
        self.commander = PiperJointCommandAdapter(
            reader.interfaces, speed_percent=settings.speed_percent
        )

    def _arm_state(self, snapshot: dict, side: str) -> dict:
        arm = snapshot.get("arms", {}).get(side)
        if not isinstance(arm, dict):
            raise ValueError(f"snapshot has no {side} arm")
        feedback = arm.get("feedback", {})
        for source in ("joint", "status"):
            age = feedback.get(f"{source}_age_s")
            if (
                not isinstance(age, (int, float))
                or not 0 <= age <= self.settings.feedback_max_age_s
            ):
                raise ValueError(f"{side} {source} feedback is stale or missing")
        controller = arm.get("controller", {})
        if controller.get("arm_status") != 0 or any(
            controller.get("errors", {}).values()
        ):
            raise ValueError(f"{side} controller is not healthy")
        joints = np.asarray(arm.get("joints_rad"), dtype=np.float64)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError(f"{side} requires six finite joint values")
        return {"joints": joints}

    def _wait_for_arm_state(self, side: str) -> dict:
        deadline = time.monotonic() + 3.0
        last_error = None
        while True:
            try:
                return self._arm_state(self.reader.snapshot(), side)
            except ValueError as exc:
                if "stale or missing" not in str(exc) and "snapshot has no" not in str(exc):
                    raise
                last_error = exc
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Piper feedback did not become ready: {last_error}"
                ) from last_error
            self.sleeper(self.settings.poll_interval_s)

    def _wait_for_enabled(self, side: str) -> bool:
        deadline = time.monotonic() + 3.0
        while True:
            enabled = _enabled(self.reader.interfaces[side])
            if enabled is not None:
                return enabled
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{side} motor enable feedback did not become complete")
            self.sleeper(self.settings.poll_interval_s)

    def _command(self, side: str, current: np.ndarray, target) -> tuple[int, ...]:
        target = [float(value) for value in target]
        report = validate_joint_target(
            current.tolist(),
            target,
            margin_rad=self.settings.joint_margin_rad,
            max_step_rad=self.settings.max_joint_step_rad,
        )
        if not report["accepted"]:
            reasons = [
                f"J{row['joint']}:{'/'.join(row['reasons'])}"
                for row in report["violations"]
            ]
            raise ValueError("runtime waypoint rejected: " + ", ".join(reasons))
        return self.commander.command_position(side, target)

    def _wait_target(self, side: str, target, authorization_guard) -> dict:
        target = np.asarray(target, dtype=np.float64)
        polls = max(
            1,
            math.ceil(
                self.settings.waypoint_timeout_s / self.settings.poll_interval_s
            ),
        )
        samples = []
        for _ in range(polls):
            authorization_guard()
            state = self._arm_state(self.reader.snapshot(), side)
            errors = np.abs(state["joints"] - target)
            zero_deadband = np.logical_and(
                np.abs(state["joints"]) <= self.settings.settle_tolerance_rad,
                np.abs(target) <= PIPER_ZERO_FEEDBACK_DEADBAND_RAD,
            )
            settle_error = float(np.max(np.where(zero_deadband, 0.0, errors)))
            tracking_error = float(np.max(errors))
            samples.append(
                {
                    "joints_rad": state["joints"].tolist(),
                    "tracking_error_rad": tracking_error,
                }
            )
            if settle_error <= self.settings.settle_tolerance_rad:
                return {
                    "state": state,
                    "tracking_error_rad": tracking_error,
                    "settle_error_rad": settle_error,
                    "samples": samples,
                }
            self.sleeper(self.settings.poll_interval_s)
        raise TimeoutError("joint waypoint did not settle before the trajectory timeout")

    def _hold_measured(self, side: str, state: dict) -> dict:
        joints = state["joints"].tolist()
        native = self.commander.command_position(side, joints)
        return {"target_joints_rad": joints, "native_command": list(native)}

    def hold(self, *, arm: str) -> dict:
        """Latch the fresh measured joint pose without changing the motion mode."""
        if arm not in {"left", "right"}:
            raise ValueError("arm must be left or right")
        state = self._wait_for_arm_state(arm)
        return {
            "schema_version": 1,
            "kind": "piper_measured_joint_hold",
            "status": "passed",
            "held_at": _utc_now(),
            "arm": arm,
            **self._hold_measured(arm, state),
        }

    @staticmethod
    def _path(start, waypoints) -> np.ndarray:
        points = np.asarray([start, *waypoints], dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (6,) or len(points) < 2:
            raise ValueError("trajectory requires a start and at least one six-joint waypoint")
        if not np.all(np.isfinite(points)):
            raise ValueError("trajectory joints must be finite")
        return points

    def _run_targets(
        self,
        arm,
        targets,
        authorization_guard,
        *,
        reached=None,
        rows=None,
    ) -> tuple[list, list]:
        reached = [] if reached is None else reached
        rows = [] if rows is None else rows
        for target in targets:
            authorization_guard()
            current = self._arm_state(self.reader.snapshot(), arm)
            native = self._command(arm, current["joints"], target)
            result = self._wait_target(arm, target, authorization_guard)
            measured = result["state"]["joints"].tolist()
            reached.append(measured)
            rows.append(
                {
                    "target_joints_rad": np.asarray(target).tolist(),
                    "measured_joints_rad": measured,
                    "native_command": list(native),
                    "tracking_error_rad": result["tracking_error_rad"],
                }
            )
        return reached, rows

    def execute(self, *, arm: str, start, waypoints, authorization_guard) -> dict:
        receipt = {
            "schema_version": 1,
            "kind": "piper_cartesian_trajectory",
            "status": "failed",
            "started_at": _utc_now(),
            "arm": arm,
            "motion_command_attempted": False,
            "retreat_attempted": False,
            "control_certain": True,
        }
        reached = []
        points = None
        try:
            if arm not in {"left", "right"}:
                raise ValueError("arm must be left or right")
            if not callable(authorization_guard):
                raise PermissionError("an operator authorization guard is required")
            points = self._path(start, waypoints)
            authorization_guard()
            if self._wait_for_enabled(arm) is not True:
                raise PermissionError(f"{arm} arm does not report all motors enabled")
            initial = self._wait_for_arm_state(arm)
            start_error = float(np.max(np.abs(initial["joints"] - points[0])))
            receipt["start_state_error_rad"] = start_error
            if start_error > self.settings.max_start_error_rad:
                raise ValueError(
                    "live joints do not match the Cartesian plan start "
                    f"({start_error:.6f} > {self.settings.max_start_error_rad:.6f} rad)"
                )
            reached = [initial["joints"].tolist()]
            receipt["motion_command_attempted"] = True
            rows = []
            self._run_targets(
                arm,
                points[1:].tolist(),
                authorization_guard,
                reached=reached,
                rows=rows,
            )
            final = self._arm_state(self.reader.snapshot(), arm)
            receipt.update(
                {
                    "status": "passed",
                    "waypoints": rows,
                    "reached_path": reached,
                    "final_hold": self._hold_measured(arm, final),
                    "final_joints_rad": final["joints"].tolist(),
                }
            )
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(
                f"{type(exc).__name__}: {exc}"
            )
            if receipt["motion_command_attempted"]:
                receipt["control_certain"] = False
                try:
                    current = self._arm_state(self.reader.snapshot(), arm)
                    receipt["failure_hold"] = self._hold_measured(arm, current)
                    receipt["control_certain"] = True
                    receipt["retreat_attempted"] = True
                    retreat_targets = list(reversed(reached))
                    _, rows = self._run_targets(
                        arm, retreat_targets, authorization_guard
                    )
                    final = self._arm_state(self.reader.snapshot(), arm)
                    receipt["retreat"] = {"status": "passed", "waypoints": rows}
                    receipt["final_hold"] = self._hold_measured(arm, final)
                    receipt["final_joints_rad"] = final["joints"].tolist()
                    receipt["control_certain"] = True
                except BaseException as retreat_exc:
                    receipt["retreat"] = {
                        "status": "failed",
                        "reason": f"{type(retreat_exc).__name__}: {retreat_exc}",
                    }
                    receipt.setdefault("failure_reasons", []).append(
                        "recovery failed: "
                        f"{type(retreat_exc).__name__}: {retreat_exc}"
                    )
                    try:
                        state = self._arm_state(self.reader.snapshot(), arm)
                        receipt["recovery_hold"] = self._hold_measured(arm, state)
                        receipt["final_joints_rad"] = state["joints"].tolist()
                        receipt["control_certain"] = True
                    except BaseException as hold_exc:
                        receipt["control_certain"] = False
                        receipt.setdefault("failure_reasons", []).append(
                            f"recovery hold failed: {type(hold_exc).__name__}: {hold_exc}"
                        )
        finally:
            receipt["finished_at"] = _utc_now()
        return receipt

    def retreat(self, *, arm: str, path, authorization_guard) -> dict:
        receipt = {
            "schema_version": 1,
            "kind": "piper_cartesian_trajectory_retreat",
            "status": "failed",
            "started_at": _utc_now(),
            "arm": arm,
            "motion_command_attempted": False,
            "control_certain": True,
        }
        try:
            points = np.asarray(path, dtype=np.float64)
            if points.ndim != 2 or points.shape[1:] != (6,) or len(points) < 2:
                raise ValueError("retreat requires a completed six-joint path")
            authorization_guard()
            if self._wait_for_enabled(arm) is not True:
                raise PermissionError(f"{arm} arm does not report all motors enabled")
            initial = self._wait_for_arm_state(arm)
            endpoint_error = float(np.max(np.abs(initial["joints"] - points[-1])))
            receipt["endpoint_error_rad"] = endpoint_error
            if endpoint_error > self.settings.max_start_error_rad:
                raise ValueError(
                    "live joints do not match the completed Cartesian path endpoint "
                    f"({endpoint_error:.6f} > {self.settings.max_start_error_rad:.6f} rad)"
                )
            receipt["motion_command_attempted"] = True
            _, rows = self._run_targets(
                arm, list(reversed(points[:-1].tolist())), authorization_guard
            )
            final = self._arm_state(self.reader.snapshot(), arm)
            final_error = float(np.max(np.abs(final["joints"] - points[0])))
            receipt.update(
                {
                    "waypoints": rows,
                    "final_start_error_rad": final_error,
                    "final_hold": self._hold_measured(arm, final),
                    "final_joints_rad": final["joints"].tolist(),
                }
            )
            if final_error > self.settings.max_start_error_rad:
                raise RuntimeError("Cartesian retreat did not return near its plan start")
            receipt["status"] = "passed"
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(
                f"{type(exc).__name__}: {exc}"
            )
            if receipt["motion_command_attempted"]:
                receipt["control_certain"] = False
                try:
                    state = self._arm_state(self.reader.snapshot(), arm)
                    receipt["failure_hold"] = self._hold_measured(arm, state)
                    receipt["control_certain"] = True
                except BaseException as hold_exc:
                    receipt.setdefault("failure_reasons", []).append(
                        f"hold failed: {type(hold_exc).__name__}: {hold_exc}"
                    )
        finally:
            receipt["finished_at"] = _utc_now()
        return receipt
