"""Operator-only guarded Piper press and reverse-path retreat.

This controller consumes a geometrically admitted read-only press plan. It commands
small joint waypoints, detects contact from a characterized motor-effort increase or
tracking stall, holds the measured pose, and retreats only through waypoints that were
actually reached. It is not safety-rated and is not exposed to the Agent.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import time

import numpy as np

from .hold import PiperJointCommandAdapter
from .effort_characterization import effort_statistics
from .minimum_motion import _enabled
from .safety import validate_joint_target


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class ContactStopSettings:
    effort_delta_nm: float
    speed_percent: int = 3
    feedback_max_age_s: float = 0.25
    poll_interval_s: float = 0.05
    waypoint_timeout_s: float = 3.0
    settle_tolerance_rad: float = 0.0008
    stall_motion_rad: float = 0.0002
    stall_speed_rad_s: float = 0.01
    stall_tracking_error_rad: float = 0.004
    consecutive_samples: int = 3
    max_contact_travel_m: float = 0.01
    max_start_error_rad: float = 0.01
    max_contact_gripper_opening_m: float = 0.005
    joint_margin_rad: float = 0.02
    max_joint_step_rad: float = 0.05

    def __post_init__(self):
        positive = (
            "effort_delta_nm",
            "feedback_max_age_s",
            "poll_interval_s",
            "waypoint_timeout_s",
            "settle_tolerance_rad",
            "stall_motion_rad",
            "stall_speed_rad_s",
            "stall_tracking_error_rad",
            "max_contact_travel_m",
            "max_start_error_rad",
            "max_contact_gripper_opening_m",
            "max_joint_step_rad",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not 1 <= self.speed_percent <= 5:
            raise ValueError("press speed_percent must be in 1..5")
        if type(self.consecutive_samples) is not int or not 2 <= self.consecutive_samples <= 10:
            raise ValueError("consecutive_samples must be in 2..10")
        if self.joint_margin_rad < 0:
            raise ValueError("joint_margin_rad must be non-negative")


class PiperGuardedPressExecutor:
    """Execute one admitted press plan and always attempt a measured hold."""

    def __init__(
        self,
        reader,
        settings: ContactStopSettings,
        *,
        sleeper=time.sleep,
        sample_observer=None,
    ):
        self.reader = reader
        self.settings = settings
        self.sleeper = sleeper
        self.sample_observer = sample_observer
        self.commander = PiperJointCommandAdapter(
            reader.interfaces, speed_percent=settings.speed_percent
        )

    def _record_sample(self, side: str, state: dict) -> None:
        if self.sample_observer is not None:
            self.sample_observer(side, state)

    def _arm_state(self, snapshot: dict, side: str) -> dict:
        arm = snapshot.get("arms", {}).get(side)
        if not isinstance(arm, dict):
            raise ValueError(f"snapshot has no {side} arm")
        feedback = arm.get("feedback", {})
        for source in ("joint", "status", "gripper", "motor"):
            age = feedback.get(f"{source}_age_s")
            if not isinstance(age, (int, float)) or not 0 <= age <= self.settings.feedback_max_age_s:
                raise ValueError(f"{side} {source} feedback is stale or missing")
        controller = arm.get("controller", {})
        if controller.get("arm_status") != 0 or any(controller.get("errors", {}).values()):
            raise ValueError(f"{side} controller is not healthy")
        joints = np.asarray(arm.get("joints_rad"), dtype=np.float64)
        effort = np.asarray(arm.get("motor", {}).get("effort_nm"), dtype=np.float64)
        speed = np.asarray(arm.get("motor", {}).get("speed_rad_s"), dtype=np.float64)
        gripper = float(arm.get("gripper_width_m", math.nan))
        if joints.shape != (6,) or effort.shape != (6,) or speed.shape != (6,):
            raise ValueError(f"{side} requires six joint, effort, and speed values")
        if not np.all(np.isfinite(np.concatenate((joints, effort, speed)))) or not math.isfinite(gripper):
            raise ValueError(f"{side} motor feedback must be finite")
        return {
            "joints": joints,
            "effort": effort,
            "speed": speed,
            "gripper_width_m": gripper,
        }

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

    def _validated_plan(
        self, plan: dict, side: str
    ) -> tuple[list[float], list[list[float]], list[list[float]]]:
        if plan.get("kind") != "click_bell_readonly_press_path" or plan.get("status") != "passed":
            raise ValueError("a passed ClickBell press-path report is required")
        if plan.get("arm") not in (None, side):
            raise ValueError("press plan arm does not match the requested arm")
        if plan.get("geometry_admissible") is not True:
            raise ValueError("press plan geometry is not admitted")
        start = [float(value) for value in plan.get("start_joints_rad", ())]
        if len(start) != 6 or not all(math.isfinite(value) for value in start):
            raise ValueError("press plan has no valid starting joint state")
        stages = plan.get("stages")
        if not isinstance(stages, list) or len(stages) < 2:
            raise ValueError("press plan requires approach and contact stages")
        distances = [float(stage.get("approach_distance_m")) for stage in stages]
        if distances[-1] != 0.0 or not 0 < distances[-2] <= self.settings.max_contact_travel_m:
            raise ValueError("final contact travel exceeds the configured cap")

        groups = []
        for stage in stages:
            stage_plan = stage.get("plan", {})
            if stage_plan.get("status") != "passed":
                raise ValueError("every press stage must have a passed IK plan")
            rows = stage_plan.get("waypoints")
            if not isinstance(rows, list) or not rows:
                raise ValueError("every press stage must contain joint waypoints")
            group = []
            for row in rows:
                joints = [float(value) for value in row.get("joints_rad", ())]
                if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
                    raise ValueError("press plan contains an invalid joint waypoint")
                group.append(joints)
            groups.append(group)
        approach = [row for group in groups[:-1] for row in group]
        return start, approach, groups[-1]

    def _validate_contact_stage_evidence(
        self,
        plan: dict,
        side: str,
        prior_stage_receipt: dict | None,
        approach_waypoints: int,
    ) -> None:
        previous = prior_stage_receipt or {}
        summary = previous.get("plan_summary", {})
        if (
            previous.get("kind") != "piper_operator_no_contact_approach"
            or previous.get("status") != "passed"
            or previous.get("arm") != side
            or previous.get("contact_intended") is not False
            or previous.get("motion_command_attempted") is not True
            or previous.get("retreat_attempted") is not True
            or previous.get("retreat", {}).get("status") != "passed"
            or summary.get("frame_id") != plan.get("frame_id")
            or not math.isclose(
                float(summary.get("approach_distance_m", math.nan)),
                float(plan["stages"][-2]["approach_distance_m"]),
                abs_tol=1e-6,
            )
            or summary.get("waypoints") != approach_waypoints
            or float(previous.get("final_start_error_rad", math.inf))
            > self.settings.max_start_error_rad
        ):
            raise ValueError(
                "guarded contact requires the immediately previous passed no-contact stage receipt"
            )

    def _command(self, side: str, current: np.ndarray, target: list[float]) -> tuple[int, ...]:
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

    def _wait_target(
        self,
        side: str,
        target: list[float],
        *,
        contact_baseline: np.ndarray | None,
        authorization_guard,
    ) -> dict:
        previous = None
        effort_hits = stall_hits = 0
        samples = []
        polls = max(1, math.ceil(self.settings.waypoint_timeout_s / self.settings.poll_interval_s))
        for _ in range(polls):
            authorization_guard()
            state = self._arm_state(self.reader.snapshot(), side)
            self._record_sample(side, state)
            error = float(np.max(np.abs(state["joints"] - target)))
            samples.append(
                {
                    "joints_rad": state["joints"].tolist(),
                    "effort_nm": state["effort"].tolist(),
                    "speed_rad_s": state["speed"].tolist(),
                    "tracking_error_rad": error,
                }
            )
            if error <= self.settings.settle_tolerance_rad:
                return {
                    "outcome": "reached",
                    "state": state,
                    "tracking_error_rad": error,
                    "samples": samples,
                }

            effort_delta = 0.0
            if contact_baseline is not None:
                effort_delta = float(np.max(np.abs(state["effort"] - contact_baseline)))
                effort_hits = effort_hits + 1 if effort_delta >= self.settings.effort_delta_nm else 0
                movement = (
                    math.inf
                    if previous is None
                    else float(np.max(np.abs(state["joints"] - previous)))
                )
                stalled = (
                    error >= self.settings.stall_tracking_error_rad
                    and movement <= self.settings.stall_motion_rad
                    and float(np.max(np.abs(state["speed"]))) <= self.settings.stall_speed_rad_s
                )
                stall_hits = stall_hits + 1 if stalled else 0
                if effort_hits >= self.settings.consecutive_samples:
                    return {
                        "outcome": "contact_effort",
                        "state": state,
                        "tracking_error_rad": error,
                        "max_effort_delta_nm": effort_delta,
                        "samples": samples,
                    }
                if stall_hits >= self.settings.consecutive_samples:
                    return {
                        "outcome": "contact_stall",
                        "state": state,
                        "tracking_error_rad": error,
                        "max_effort_delta_nm": effort_delta,
                        "samples": samples,
                    }
                previous = state["joints"].copy()
            self.sleeper(self.settings.poll_interval_s)
        raise TimeoutError("joint waypoint did not settle before the contact watchdog timeout")

    def _validated_no_contact_plan(
        self,
        plan: dict,
        side: str,
        *,
        approach_distance_m: float,
        max_waypoints: int,
        prior_stage_receipt: dict | None = None,
    ) -> tuple[list[float], list[list[float]]]:
        if plan.get("kind") != "click_bell_readonly_press_path" or plan.get("status") != "passed":
            raise ValueError("a passed ClickBell press-path report is required")
        if plan.get("arm") not in (None, side):
            raise ValueError("press plan arm does not match the requested arm")
        if plan.get("geometry_admissible") is not True:
            raise ValueError("press plan geometry is not admitted")
        if not math.isfinite(approach_distance_m) or approach_distance_m < 0.005:
            raise ValueError("no-contact tests must remain at least 0.005 m from the surface")
        if type(max_waypoints) is not int or not 1 <= max_waypoints <= 128:
            raise ValueError("max_waypoints must be in 1..128")
        start = [float(value) for value in plan.get("start_joints_rad", ())]
        if len(start) != 6 or not all(math.isfinite(value) for value in start):
            raise ValueError("press plan has no valid starting joint state")
        stages = list(plan.get("stages", ()))
        selected_index = None
        for index, stage in enumerate(stages):
            distance = float(stage.get("approach_distance_m"))
            if math.isclose(distance, approach_distance_m, abs_tol=1e-6):
                selected_index = index
                break
        if selected_index is None:
            raise ValueError("requested no-contact stage is absent from the press plan")
        if float(stages[selected_index].get("approach_distance_m")) <= 0:
            raise ValueError("the contact stage is not a no-contact target")

        rows = []
        for stage in stages[: selected_index + 1]:
            stage_plan = stage.get("plan", {})
            stage_rows = stage_plan.get("waypoints")
            if stage_plan.get("status") != "passed" or not isinstance(stage_rows, list):
                raise ValueError("requested no-contact stage did not pass IK planning")
            rows.extend(stage_rows)
        if len(rows) < max_waypoints:
            raise ValueError("requested no-contact waypoint count is unavailable")
        if selected_index > 0:
            expected_previous_rows = sum(
                len(stage.get("plan", {}).get("waypoints", ()))
                for stage in stages[:selected_index]
            )
            expected_previous_distance = float(
                stages[selected_index - 1].get("approach_distance_m")
            )
            previous = prior_stage_receipt or {}
            summary = previous.get("plan_summary", {})
            retreat = previous.get("retreat", {})
            if (
                previous.get("kind") != "piper_operator_no_contact_approach"
                or previous.get("status") != "passed"
                or previous.get("arm") != side
                or previous.get("contact_intended") is not False
                or previous.get("motion_command_attempted") is not True
                or previous.get("retreat_attempted") is not True
                or retreat.get("status") != "passed"
                or summary.get("frame_id") != plan.get("frame_id")
                or not math.isclose(
                    float(summary.get("approach_distance_m", math.nan)),
                    expected_previous_distance,
                    abs_tol=1e-6,
                )
                or summary.get("waypoints") != expected_previous_rows
                or float(previous.get("final_start_error_rad", math.inf))
                > self.settings.max_start_error_rad
            ):
                raise ValueError("closer no-contact stage requires the immediately previous passed stage receipt")
            if max_waypoints != len(rows):
                raise ValueError("closer no-contact stages must execute their complete cumulative path")
        targets = []
        for row in rows[:max_waypoints]:
            joints = [float(value) for value in row.get("joints_rad", ())]
            if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
                raise ValueError("press plan contains an invalid joint waypoint")
            targets.append(joints)
        return start, targets

    def run_no_contact(
        self,
        plan: dict,
        *,
        arm: str,
        approach_distance_m: float,
        max_waypoints: int,
        operator_lease_id: str,
        execute: bool,
        authorization_guard,
        prior_stage_receipt: dict | None = None,
    ) -> dict:
        """Execute a short admitted approach prefix and reverse only reached poses."""
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_no_contact_approach",
            "status": "failed",
            "started_at": _utc_now(),
            "arm": arm,
            "agent_tool_exposed": False,
            "contact_intended": False,
            "motion_command_attempted": False,
            "retreat_attempted": False,
        }
        try:
            if arm not in ("left", "right"):
                raise ValueError("arm must be left or right")
            if not isinstance(operator_lease_id, str) or not operator_lease_id.strip():
                raise PermissionError("a fresh operator lease id is required")
            if execute and not callable(authorization_guard):
                raise PermissionError("an operator authorization guard is required")
            expected_start, targets = self._validated_no_contact_plan(
                plan,
                arm,
                approach_distance_m=approach_distance_m,
                max_waypoints=max_waypoints,
                prior_stage_receipt=prior_stage_receipt,
            )
            receipt["plan_summary"] = {
                "frame_id": plan.get("frame_id"),
                "approach_distance_m": approach_distance_m,
                "waypoints": len(targets),
            }
            if not execute:
                receipt["status"] = "planned"
                return receipt
            authorization_guard()
            if self._wait_for_enabled(arm) is not True:
                raise PermissionError(f"{arm} arm does not report all motors enabled")
            initial = self._wait_for_arm_state(arm)
            start_error = float(np.max(np.abs(initial["joints"] - expected_start)))
            receipt["start_state_error_rad"] = start_error
            if start_error > self.settings.max_start_error_rad:
                raise ValueError(
                    "live joints do not match the audited path start "
                    f"({start_error:.6f} > {self.settings.max_start_error_rad:.6f} rad)"
                )
            reached = [initial["joints"].tolist()]
            samples = []
            approach_rows = []
            for target in targets:
                authorization_guard()
                current = self._arm_state(self.reader.snapshot(), arm)
                receipt["motion_command_attempted"] = True
                native = self._command(arm, current["joints"], target)
                result = self._wait_target(
                    arm, target, contact_baseline=None, authorization_guard=authorization_guard
                )
                reached.append(result["state"]["joints"].tolist())
                samples.extend(result["samples"])
                approach_rows.append(
                    {
                        "target_joints_rad": target,
                        "native_command": list(native),
                        "tracking_error_rad": result["tracking_error_rad"],
                    }
                )

            receipt["retreat_attempted"] = True
            retreat_rows = []
            for target in reversed(reached[:-1]):
                authorization_guard()
                current = self._arm_state(self.reader.snapshot(), arm)
                native = self._command(arm, current["joints"], target)
                result = self._wait_target(
                    arm, target, contact_baseline=None, authorization_guard=authorization_guard
                )
                samples.extend(result["samples"])
                retreat_rows.append(
                    {
                        "target_joints_rad": target,
                        "native_command": list(native),
                        "tracking_error_rad": result["tracking_error_rad"],
                    }
                )
            final_state = self._arm_state(self.reader.snapshot(), arm)
            receipt["final_hold"] = self._hold_measured(arm, final_state)
            final_error = float(np.max(np.abs(final_state["joints"] - expected_start)))
            receipt.update(
                {
                    "approach": {"status": "passed", "waypoints": approach_rows},
                    "retreat": {"status": "passed", "waypoints": retreat_rows},
                    "final_start_error_rad": final_error,
                    "dynamic_effort": {
                        "samples": samples,
                        "statistics": effort_statistics(
                            [sample["effort_nm"] for sample in samples]
                        ),
                    },
                }
            )
            if final_error > self.settings.max_start_error_rad:
                raise RuntimeError("reverse retreat did not return near the audited start")
            receipt["status"] = "passed"
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(f"{type(exc).__name__}: {exc}")
            if execute and receipt["motion_command_attempted"]:
                try:
                    state = self._arm_state(self.reader.snapshot(), arm)
                    receipt["failure_hold"] = self._hold_measured(arm, state)
                except BaseException as hold_exc:
                    receipt.setdefault("failure_reasons", []).append(
                        f"hold failed: {type(hold_exc).__name__}: {hold_exc}"
                    )
        finally:
            receipt["finished_at"] = _utc_now()
        return receipt

    def return_to_plan_start(
        self,
        plan: dict,
        *,
        arm: str,
        operator_lease_id: str,
        execute: bool,
        authorization_guard,
    ) -> dict:
        """Return directly to an admitted plan start when one bounded step is valid."""
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_plan_start_recovery",
            "status": "failed",
            "started_at": _utc_now(),
            "arm": arm,
            "agent_tool_exposed": False,
            "motion_command_attempted": False,
        }
        try:
            expected_start, _ = self._validated_no_contact_plan(
                plan,
                arm,
                approach_distance_m=0.06,
                max_waypoints=1,
            )
            if not isinstance(operator_lease_id, str) or not operator_lease_id.strip():
                raise PermissionError("a fresh operator lease id is required")
            if execute and not callable(authorization_guard):
                raise PermissionError("an operator authorization guard is required")
            if not execute:
                receipt["target_joints_rad"] = expected_start
                receipt["status"] = "planned"
                return receipt
            authorization_guard()
            if self._wait_for_enabled(arm) is not True:
                raise PermissionError(f"{arm} arm does not report all motors enabled")
            current = self._wait_for_arm_state(arm)
            receipt["initial_joints_rad"] = current["joints"].tolist()
            receipt["initial_start_error_rad"] = float(
                np.max(np.abs(current["joints"] - expected_start))
            )
            receipt["motion_command_attempted"] = True
            native = self._command(arm, current["joints"], expected_start)
            result = self._wait_target(
                arm,
                expected_start,
                contact_baseline=None,
                authorization_guard=authorization_guard,
            )
            final = result["state"]
            receipt.update(
                {
                    "native_command": list(native),
                    "tracking_error_rad": result["tracking_error_rad"],
                    "final_hold": self._hold_measured(arm, final),
                    "final_joints_rad": final["joints"].tolist(),
                    "status": "passed",
                }
            )
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(f"{type(exc).__name__}: {exc}")
            if execute and receipt["motion_command_attempted"]:
                try:
                    state = self._arm_state(self.reader.snapshot(), arm)
                    receipt["failure_hold"] = self._hold_measured(arm, state)
                except BaseException as hold_exc:
                    receipt.setdefault("failure_reasons", []).append(
                        f"hold failed: {type(hold_exc).__name__}: {hold_exc}"
                    )
        finally:
            receipt["finished_at"] = _utc_now()
        return receipt

    def _hold_measured(self, side: str, state: dict) -> dict:
        native = self.commander.command_position(side, state["joints"].tolist())
        return {"target_joints_rad": state["joints"].tolist(), "native_command": list(native)}

    def run(
        self,
        plan: dict,
        *,
        arm: str,
        operator_lease_id: str,
        execute: bool,
        authorization_guard,
        prior_stage_receipt: dict | None = None,
    ) -> dict:
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_guarded_press",
            "status": "failed",
            "started_at": _utc_now(),
            "arm": arm,
            "agent_tool_exposed": False,
            "safety_rated_stop_available": False,
            "motion_command_attempted": False,
            "retreat_attempted": False,
        }
        try:
            if arm not in ("left", "right"):
                raise ValueError("arm must be left or right")
            if not isinstance(operator_lease_id, str) or not operator_lease_id.strip():
                raise PermissionError("a fresh operator lease id is required")
            if execute and not callable(authorization_guard):
                raise PermissionError("an operator authorization guard is required")
            expected_start, approach, contact = self._validated_plan(plan, arm)
            receipt["plan_summary"] = {
                "frame_id": plan.get("frame_id"),
                "approach_waypoints": len(approach),
                "contact_waypoints": len(contact),
                "contact_travel_m": float(plan["stages"][-2]["approach_distance_m"]),
            }
            if not execute:
                receipt["status"] = "planned"
                return receipt
            authorization_guard()
            if self._wait_for_enabled(arm) is not True:
                raise PermissionError(f"{arm} arm does not report all motors enabled")
            initial = self._wait_for_arm_state(arm)
            receipt["initial_gripper_width_m"] = initial["gripper_width_m"]
            if initial["gripper_width_m"] > self.settings.max_contact_gripper_opening_m:
                raise ValueError(
                    "closed-gripper contact geometry requires gripper width at most "
                    f"{self.settings.max_contact_gripper_opening_m:.4f} m; measured "
                    f"{initial['gripper_width_m']:.4f} m"
                )
            self._validate_contact_stage_evidence(
                plan, arm, prior_stage_receipt, len(approach)
            )
            start_error = float(np.max(np.abs(initial["joints"] - expected_start)))
            receipt["start_state_error_rad"] = start_error
            if start_error > self.settings.max_start_error_rad:
                raise ValueError(
                    "live joints do not match the audited press-plan start "
                    f"({start_error:.6f} > {self.settings.max_start_error_rad:.6f} rad)"
                )
            reached = [initial["joints"].tolist()]
            receipt["initial_joints_rad"] = reached[0]

            for target in approach:
                authorization_guard()
                current = self._arm_state(self.reader.snapshot(), arm)
                receipt["motion_command_attempted"] = True
                self._command(arm, current["joints"], target)
                result = self._wait_target(
                    arm, target, contact_baseline=None, authorization_guard=authorization_guard
                )
                reached.append(result["state"]["joints"].tolist())

            baseline = self._arm_state(self.reader.snapshot(), arm)["effort"].copy()
            contact_event = None
            for target in contact:
                authorization_guard()
                current = self._arm_state(self.reader.snapshot(), arm)
                receipt["motion_command_attempted"] = True
                self._command(arm, current["joints"], target)
                result = self._wait_target(
                    arm,
                    target,
                    contact_baseline=baseline,
                    authorization_guard=authorization_guard,
                )
                if result["outcome"].startswith("contact_"):
                    contact_event = result
                    receipt["contact_hold"] = self._hold_measured(arm, result["state"])
                    break
                reached.append(result["state"]["joints"].tolist())

            receipt["contact_event"] = None if contact_event is None else {
                key: value for key, value in contact_event.items() if key != "state"
            }
            receipt["retreat_attempted"] = True
            retreat_rows = []
            retreat_targets = reached if contact_event is not None else reached[:-1]
            for target in reversed(retreat_targets):
                authorization_guard()
                current = self._arm_state(self.reader.snapshot(), arm)
                native = self._command(arm, current["joints"], target)
                result = self._wait_target(
                    arm, target, contact_baseline=None, authorization_guard=authorization_guard
                )
                retreat_rows.append({
                    "target_joints_rad": target,
                    "native_command": list(native),
                    "tracking_error_rad": result["tracking_error_rad"],
                })
            final_state = self._arm_state(self.reader.snapshot(), arm)
            receipt["final_hold"] = self._hold_measured(arm, final_state)
            receipt["retreat"] = {"status": "passed", "waypoints": retreat_rows}
            receipt["status"] = "passed" if contact_event is not None else "contact_unverified"
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(f"{type(exc).__name__}: {exc}")
            if execute and receipt["motion_command_attempted"]:
                try:
                    state = self._arm_state(self.reader.snapshot(), arm)
                    receipt["failure_hold"] = self._hold_measured(arm, state)
                except BaseException as hold_exc:
                    receipt.setdefault("failure_reasons", []).append(
                        f"hold failed: {type(hold_exc).__name__}: {hold_exc}"
                    )
        finally:
            receipt["finished_at"] = _utc_now()
        return receipt
