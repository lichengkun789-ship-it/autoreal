"""Operator-only Piper gripper close-and-restore characterization."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import time

import numpy as np

from .gripper import PiperGripperCommandAdapter


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class GripperCycleSettings:
    step_m: float = 0.01
    effort: int = 500
    target_tolerance_m: float = 0.003
    feedback_max_age_s: float = 0.25
    poll_interval_s: float = 0.05
    target_timeout_s: float = 3.0
    startup_timeout_s: float = 3.0
    max_arm_joint_drift_rad: float = 0.002

    def __post_init__(self):
        for name in (
            "step_m",
            "target_tolerance_m",
            "feedback_max_age_s",
            "poll_interval_s",
            "target_timeout_s",
            "startup_timeout_s",
            "max_arm_joint_drift_rad",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.step_m > 0.02:
            raise ValueError("gripper step_m cannot exceed 0.02 m")
        if type(self.effort) is not int or not 1 <= self.effort <= 1000:
            raise ValueError("characterization effort must be an integer in 1..1000")


def incremental_targets(start_m: float, goal_m: float, step_m: float) -> list[float]:
    start, goal, step = float(start_m), float(goal_m), float(step_m)
    if not all(math.isfinite(value) for value in (start, goal, step)) or step <= 0:
        raise ValueError("gripper target inputs must be finite and step must be positive")
    if start == goal:
        return []
    direction = 1.0 if goal > start else -1.0
    values = []
    current = start
    while abs(goal - current) > step:
        current += direction * step
        values.append(round(current, 6))
    values.append(goal)
    return values


class PiperGripperCycleRunner:
    """Close one empty gripper in small steps, then restore its initial opening."""

    def __init__(
        self,
        reader,
        settings=None,
        *,
        sleeper=time.sleep,
        sample_observer=None,
    ):
        self.reader = reader
        self.settings = settings or GripperCycleSettings()
        self.sleeper = sleeper
        self.sample_observer = sample_observer
        self.commander = PiperGripperCommandAdapter(reader.interfaces)

    def _state(self, snapshot: dict, arm: str, initial_joints: dict | None = None) -> dict:
        arms = snapshot.get("arms", {})
        if set(arms) != {"left", "right"}:
            raise ValueError("snapshot must contain both Piper arms")
        joints = {}
        for side in ("left", "right"):
            value = arms[side]
            feedback = value.get("feedback", {})
            for source in ("joint", "status", "gripper"):
                age = feedback.get(f"{source}_age_s")
                if not isinstance(age, (int, float)) or not 0 <= age <= self.settings.feedback_max_age_s:
                    raise ValueError(f"{side} {source} feedback is stale or missing")
            controller = value.get("controller", {})
            if controller.get("arm_status") != 0 or any(controller.get("errors", {}).values()):
                raise ValueError(f"{side} arm controller is not healthy")
            joints[side] = np.asarray(value.get("joints_rad"), dtype=np.float64)
            if joints[side].shape != (6,) or not np.all(np.isfinite(joints[side])):
                raise ValueError(f"{side} joint feedback is invalid")
            if initial_joints is not None:
                drift = float(np.max(np.abs(joints[side] - initial_joints[side])))
                if drift > self.settings.max_arm_joint_drift_rad:
                    raise RuntimeError(
                        f"{side} arm moved during gripper test ({drift:.6f} rad)"
                    )
        selected = arms[arm]
        gripper = selected.get("gripper", {})
        width = float(gripper.get("width_m", selected.get("gripper_width_m", math.nan)))
        effort = float(gripper.get("effort_nm", 0.0))
        if not math.isfinite(width) or not 0 <= width <= 0.11 or not math.isfinite(effort):
            raise ValueError(f"{arm} gripper feedback is invalid")
        errors = gripper.get("errors", {})
        if any(errors.values()):
            raise RuntimeError(f"{arm} gripper reports an error: {errors}")
        state = {"joints": joints, "width_m": width, "effort_nm": effort}
        if self.sample_observer is not None:
            motor = selected.get("motor", {})
            self.sample_observer(
                arm,
                {
                    "joints": joints[arm],
                    "effort": np.asarray(motor.get("effort_nm", [0.0] * 6)),
                    "speed": np.asarray(motor.get("speed_rad_s", [0.0] * 6)),
                    "gripper_width_m": width,
                },
            )
        return state

    def _wait_for_target(
        self,
        arm: str,
        target_m: float,
        initial_joints: dict,
        authorization_guard=None,
    ) -> dict:
        polls = max(1, math.ceil(self.settings.target_timeout_s / self.settings.poll_interval_s))
        samples = []
        for _ in range(polls):
            if authorization_guard is not None:
                authorization_guard()
            state = self._state(self.reader.snapshot(), arm, initial_joints)
            sample = {"width_m": state["width_m"], "effort_nm": state["effort_nm"]}
            samples.append(sample)
            if abs(state["width_m"] - target_m) <= self.settings.target_tolerance_m:
                return {"target_m": target_m, "reached": True, "samples": samples}
            self.sleeper(self.settings.poll_interval_s)
        raise TimeoutError(
            f"{arm} gripper did not reach {target_m:.4f} m; last width "
            f"{samples[-1]['width_m']:.4f} m"
        )

    def _wait_for_initial_state(self, arm: str) -> dict:
        deadline = time.monotonic() + self.settings.startup_timeout_s
        last_error = None
        while True:
            try:
                return self._state(self.reader.snapshot(), arm)
            except ValueError as exc:
                if "stale or missing" not in str(exc) and "both Piper arms" not in str(exc):
                    raise
                last_error = exc
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Piper feedback did not become ready: {last_error}"
                ) from last_error
            self.sleeper(self.settings.poll_interval_s)

    def current_state(self, arm: str) -> dict:
        if arm not in ("left", "right"):
            raise ValueError("arm must be left or right")
        return self._wait_for_initial_state(arm)

    def _move_sequence(
        self,
        arm: str,
        targets: list[float],
        initial_joints: dict,
        receipt: dict,
        authorization_guard=None,
    ) -> list[dict]:
        rows = []
        for target in targets:
            if authorization_guard is not None:
                authorization_guard()
            receipt["gripper_command_attempted"] = True
            command = self.commander.command_opening(
                arm, target, effort=self.settings.effort
            )
            result = self._wait_for_target(
                arm, target, initial_joints, authorization_guard
            )
            rows.append({"command": command, "feedback": result})
        return rows

    def run(self, *, arm: str, execute: bool, authorization_guard=None) -> dict:
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_gripper_cycle",
            "status": "failed",
            "started_at": _utc_now(),
            "arm": arm,
            "agent_tool_exposed": False,
            "arm_motion_commanded": False,
            "gripper_command_attempted": False,
        }
        initial_joints = None
        restore_target = None
        restored = False
        try:
            if arm not in ("left", "right"):
                raise ValueError("arm must be left or right")
            initial = self._wait_for_initial_state(arm)
            initial_joints = initial["joints"]
            restore_target = initial["width_m"]
            close_targets = incremental_targets(
                restore_target, 0.0, self.settings.step_m
            )
            open_targets = incremental_targets(
                0.0, restore_target, self.settings.step_m
            )
            receipt.update(
                {
                    "initial_width_m": restore_target,
                    "close_targets_m": close_targets,
                    "restore_targets_m": open_targets,
                    "settings": self.settings.__dict__,
                }
            )
            if not execute:
                receipt["status"] = "planned"
                return receipt
            receipt["close"] = self._move_sequence(
                arm, close_targets, initial_joints, receipt, authorization_guard
            )
            receipt["restore"] = self._move_sequence(
                arm, open_targets, initial_joints, receipt, authorization_guard
            )
            final = self._state(self.reader.snapshot(), arm, initial_joints)
            restored = abs(final["width_m"] - restore_target) <= self.settings.target_tolerance_m
            receipt["final_width_m"] = final["width_m"]
            receipt["restored"] = restored
            if not restored:
                raise RuntimeError("gripper did not return to its initial opening")
            receipt["status"] = "passed"
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(f"{type(exc).__name__}: {exc}")
        finally:
            if execute and receipt["gripper_command_attempted"] and not restored and restore_target is not None:
                try:
                    command = self.commander.command_opening(
                        arm, restore_target, effort=self.settings.effort
                    )
                    state = self._wait_for_target(
                        arm, restore_target, initial_joints, authorization_guard
                    )
                    receipt["emergency_restore"] = {"command": command, "feedback": state}
                    receipt["restored"] = True
                except BaseException as restore_exc:
                    receipt.setdefault("failure_reasons", []).append(
                        f"restore failed: {type(restore_exc).__name__}: {restore_exc}"
                    )
                    receipt["restored"] = False
            receipt["finished_at"] = _utc_now()
        return receipt


class PiperGripperSetRunner(PiperGripperCycleRunner):
    """Move one empty gripper to an operator-selected opening and keep it there."""

    def run(
        self,
        *,
        arm: str,
        target_m: float,
        execute: bool,
        authorization_guard=None,
    ) -> dict:
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_gripper_set",
            "status": "failed",
            "started_at": _utc_now(),
            "arm": arm,
            "agent_tool_exposed": False,
            "arm_motion_commanded": False,
            "gripper_command_attempted": False,
        }
        initial_joints = None
        initial_width = None
        reached = False
        try:
            if arm not in ("left", "right"):
                raise ValueError("arm must be left or right")
            target = float(target_m)
            if not math.isfinite(target) or not 0 <= target <= 0.10:
                raise ValueError("target_m must be in 0..0.10 m")
            initial = self._wait_for_initial_state(arm)
            initial_joints = initial["joints"]
            initial_width = initial["width_m"]
            targets = incremental_targets(initial_width, target, self.settings.step_m)
            receipt.update(
                {
                    "initial_width_m": initial_width,
                    "target_width_m": target,
                    "targets_m": targets,
                    "settings": self.settings.__dict__,
                }
            )
            if not execute:
                receipt["status"] = "planned"
                return receipt
            receipt["commands"] = self._move_sequence(
                arm, targets, initial_joints, receipt, authorization_guard
            )
            final = self._state(self.reader.snapshot(), arm, initial_joints)
            receipt["final_width_m"] = final["width_m"]
            reached = abs(final["width_m"] - target) <= self.settings.target_tolerance_m
            if not reached:
                raise RuntimeError("gripper did not reach the requested opening")
            receipt["status"] = "passed"
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(f"{type(exc).__name__}: {exc}")
        finally:
            if (
                execute
                and receipt["gripper_command_attempted"]
                and not reached
                and initial_width is not None
                and initial_joints is not None
            ):
                try:
                    command = self.commander.command_opening(
                        arm, initial_width, effort=self.settings.effort
                    )
                    state = self._wait_for_target(
                        arm, initial_width, initial_joints, authorization_guard
                    )
                    receipt["failure_restore"] = {"command": command, "feedback": state}
                except BaseException as restore_exc:
                    receipt.setdefault("failure_reasons", []).append(
                        f"restore failed: {type(restore_exc).__name__}: {restore_exc}"
                    )
            receipt["finished_at"] = _utc_now()
        return receipt
