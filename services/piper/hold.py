"""Hardware-independent Piper position-hold latch.

The controller accepts SDK-shaped command interfaces by dependency injection. Tests
use in-memory fakes; this module never creates a CAN connection, changes controller
mode, enables motors, resets faults, or exposes an Agent tool.
"""

from __future__ import annotations

import math

from .safety import JOINT_LIMITS_RAD


SIDES = ("left", "right")
READ_ONLY = "read_only"
MOTION_ARMED = "motion_armed"
HOLD_LATCHED = "hold_latched"
RECOVERY_REQUIRED = "recovery_required"
RAD_TO_MILLIDEGREES = 180_000.0 / math.pi


class HoldDispatchError(RuntimeError):
    """A hold target was not confirmed for both arms."""

    control_certain = False

    def __init__(self, message: str, *, robot_state: dict | None = None):
        super().__init__(message)
        self.robot_state = robot_state


class MotionBlocked(RuntimeError):
    """Motion is blocked by the administrative interlock."""

    control_certain = True


def _native_joint_target(joints_rad) -> tuple[int, ...]:
    if not isinstance(joints_rad, (list, tuple)) or len(joints_rad) != 6:
        raise ValueError("Piper joint targets must contain exactly 6 values")
    values = tuple(float(value) for value in joints_rad)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Piper joint targets must be finite")
    return tuple(round(value * RAD_TO_MILLIDEGREES) for value in values)


class PiperJointCommandAdapter:
    """Narrow adapter over already-connected SDK interfaces.

    The owning motion worker is responsible for placing the controller in joint
    position mode before arming. This adapter deliberately exposes only ``JointCtrl``.
    """

    def __init__(self, interfaces: dict):
        if set(interfaces) != set(SIDES):
            raise ValueError("command adapter requires exactly left and right interfaces")
        self.interfaces = dict(interfaces)

    def command_position(self, side: str, joints_rad) -> tuple[int, ...]:
        if side not in SIDES:
            raise ValueError("side must be left or right")
        native = _native_joint_target(joints_rad)
        self.interfaces[side].JointCtrl(*native)
        return native


class PiperHoldController:
    """Fail-closed state machine for a future single-owner motion worker."""

    def __init__(self, reader, commander, *, feedback_max_age_s: float = 0.25):
        if feedback_max_age_s <= 0:
            raise ValueError("feedback_max_age_s must be positive")
        self.reader = reader
        self.commander = commander
        self.feedback_max_age_s = float(feedback_max_age_s)
        self.state = READ_ONLY
        self.reason = None
        self.hold_targets_rad = None
        self.hold_native_arguments = None
        self.last_operator_lease_id = None

    def status(self) -> dict:
        return {
            "state": self.state,
            "motion_allowed": self.state == MOTION_ARMED,
            "hold_latched": self.state in {HOLD_LATCHED, RECOVERY_REQUIRED},
            "hold_sdk_calls_returned": self.hold_native_arguments is not None,
            "reason": self.reason,
            "hold_targets_rad": self.hold_targets_rad,
        }

    def operator_arm_for_characterization(
        self,
        *,
        operator_lease_id: str,
        stop_plan_passed: bool,
        passive_probe_passed: bool,
    ) -> dict:
        """Enter a software motion window; this sends no hardware command."""
        if self.state != READ_ONLY:
            raise MotionBlocked(f"cannot arm from {self.state}; recover to read_only first")
        if not isinstance(operator_lease_id, str) or not operator_lease_id.strip():
            raise PermissionError("a new operator lease id is required")
        if operator_lease_id == self.last_operator_lease_id:
            raise PermissionError("the previous operator lease cannot be reused")
        if stop_plan_passed is not True:
            raise PermissionError("the operator stop plan has not passed")
        if passive_probe_passed is not True:
            raise PermissionError("the passive Piper probe has not passed")
        self.last_operator_lease_id = operator_lease_id
        self.state = MOTION_ARMED
        self.reason = None
        self.hold_targets_rad = None
        self.hold_native_arguments = None
        return self.status()

    def require_motion_allowed(self) -> None:
        if self.state != MOTION_ARMED:
            raise MotionBlocked(f"motion blocked while Piper state is {self.state}")

    def request_position_hold(self, reason: str) -> dict:
        """Latch first, then dispatch fresh measured positions to both arms once.

        A successful return means only that both SDK calls returned. Physical hold and
        refresh-rate behavior remain hardware-characterization questions.
        """
        if self.state == HOLD_LATCHED:
            return self.status()
        if self.state != MOTION_ARMED:
            raise MotionBlocked(f"position hold cannot start from {self.state}")
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("hold reason must be a non-empty string")

        self.state = HOLD_LATCHED
        self.reason = reason.strip()
        snapshot = None
        try:
            snapshot = self.reader.snapshot()
            targets = self._fresh_targets(snapshot)
            native = {}
            for side in SIDES:
                native[side] = self.commander.command_position(side, targets[side])
            self.hold_targets_rad = targets
            self.hold_native_arguments = native
            return self.status()
        except Exception as exc:
            self.state = RECOVERY_REQUIRED
            raise HoldDispatchError(
                f"position hold was not confirmed for both arms: {exc}",
                robot_state=snapshot,
            ) from exc

    def _fresh_targets(self, snapshot: dict) -> dict[str, tuple[float, ...]]:
        arms = snapshot.get("arms") if isinstance(snapshot, dict) else None
        if not isinstance(arms, dict) or set(arms) != set(SIDES):
            raise ValueError("snapshot must contain exactly left and right arms")
        targets = {}
        for side in SIDES:
            arm = arms[side]
            feedback = arm.get("feedback", {})
            for source in ("joint", "status"):
                age = feedback.get(f"{source}_age_s")
                if not isinstance(age, (int, float)) or age < 0 or age > self.feedback_max_age_s:
                    raise ValueError(f"{side} {source} feedback is stale or missing")
            controller = arm.get("controller", {})
            if controller.get("arm_status") != 0:
                raise ValueError(f"{side} controller reports arm_status={controller.get('arm_status')}")
            if any(controller.get("errors", {}).values()):
                raise ValueError(f"{side} controller reports joint errors")
            joints = arm.get("joints_rad")
            if not isinstance(joints, (list, tuple)) or len(joints) != 6:
                raise ValueError(f"{side} must report exactly 6 joints")
            values = tuple(float(value) for value in joints)
            for index, (value, limits) in enumerate(zip(values, JOINT_LIMITS_RAD), start=1):
                if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
                    raise ValueError(f"{side} joint {index} feedback is outside known limits")
            targets[side] = values
        return targets

    def operator_recover_to_read_only(
        self,
        *,
        arm_mechanically_supported: bool,
        workspace_inspected: bool,
        operator_recovery_completed: bool,
        faults_clear: bool,
        passive_probe_passed: bool,
    ) -> dict:
        """Record completed manual recovery; never reset or enable hardware."""
        if self.state not in {HOLD_LATCHED, RECOVERY_REQUIRED}:
            raise MotionBlocked(f"recovery cannot start from {self.state}")
        checks = {
            "arm_mechanically_supported": arm_mechanically_supported,
            "workspace_inspected": workspace_inspected,
            "operator_recovery_completed": operator_recovery_completed,
            "faults_clear": faults_clear,
            "passive_probe_passed": passive_probe_passed,
        }
        missing = [name for name, passed in checks.items() if passed is not True]
        if missing:
            raise PermissionError("manual recovery checks not satisfied: " + ", ".join(missing))
        self.state = READ_ONLY
        self.reason = "manual recovery completed; new operator lease required"
        return self.status()
