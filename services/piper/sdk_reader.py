"""Read-only adapter around the public ``piper_sdk`` interface.

This module intentionally exposes no command method. Motion is added only after the
operator bring-up and characterization protocol establishes a safe hold operation,
control rate, stop latency, and per-call limits.
"""

from __future__ import annotations

import math
import time


MILLI_DEGREES_TO_RADIANS = math.pi / 180_000.0
MICROMETRES_TO_METRES = 1e-6


def _integer(value) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _status_flags(status) -> dict[str, bool]:
    errors = getattr(status, "err_status", None)
    result = {}
    for index in range(1, 7):
        result[f"joint_{index}_angle_limit"] = bool(
            getattr(errors, f"joint_{index}_angle_limit", False)
        )
        result[f"joint_{index}_communication_error"] = bool(
            getattr(errors, f"communication_status_joint_{index}", False)
        )
    return result


class PiperSdkReader:
    """Own two Piper SDK connections and return JSON-safe measured state."""

    def __init__(self, arms: dict[str, dict], *, interface_factory=None, clock=time.time):
        if set(arms) != {"left", "right"}:
            raise ValueError("piper backend requires exactly left and right arm configurations")
        if interface_factory is None:
            try:
                from piper_sdk import C_PiperInterface_V2
            except ImportError as exc:
                raise RuntimeError(
                    "piper_sdk is not installed in the robot-service environment"
                ) from exc
            interface_factory = C_PiperInterface_V2

        self.clock = clock
        self.interfaces = {}
        for side, spec in arms.items():
            unknown = set(spec) - {"can_name"}
            if unknown:
                raise ValueError(f"unknown {side} arm options: {sorted(unknown)}")
            can_name = spec.get("can_name")
            if not isinstance(can_name, str) or not can_name:
                raise ValueError(f"{side}.can_name must be a non-empty string")
            interface = interface_factory(can_name)
            interface.ConnectPort()
            self.interfaces[side] = interface

    def snapshot(self) -> dict:
        measured_at = self.clock()
        return {
            "measured_at": measured_at,
            "arms": {
                side: self._arm_snapshot(interface, measured_at)
                for side, interface in self.interfaces.items()
            },
        }

    @staticmethod
    def _arm_snapshot(interface, measured_at: float) -> dict:
        joints_message = interface.GetArmJointMsgs()
        status_message = interface.GetArmStatus()
        gripper_message = interface.GetArmGripperMsgs()
        joint_state = joints_message.joint_state
        status = status_message.arm_status
        gripper = gripper_message.gripper_state
        joint_time = float(getattr(joints_message, "time_stamp", 0.0) or 0.0)
        status_time = float(getattr(status_message, "time_stamp", 0.0) or 0.0)
        gripper_time = float(getattr(gripper_message, "time_stamp", 0.0) or 0.0)
        return {
            "joints_rad": [
                float(getattr(joint_state, f"joint_{index}")) * MILLI_DEGREES_TO_RADIANS
                for index in range(1, 7)
            ],
            "gripper_width_m": float(getattr(gripper, "grippers_angle", 0))
            * MICROMETRES_TO_METRES,
            "feedback": {
                "joint_time": joint_time,
                "joint_age_s": None if joint_time <= 0 else max(0.0, measured_at - joint_time),
                "joint_hz": float(getattr(joints_message, "Hz", 0.0) or 0.0),
                "status_time": status_time,
                "status_age_s": None if status_time <= 0 else max(0.0, measured_at - status_time),
                "gripper_time": gripper_time,
                "gripper_age_s": None if gripper_time <= 0 else max(0.0, measured_at - gripper_time),
            },
            "controller": {
                "control_mode": _integer(getattr(status, "ctrl_mode", 0)),
                "arm_status": _integer(getattr(status, "arm_status", 0)),
                "motion_status": _integer(getattr(status, "motion_status", 0)),
                "errors": _status_flags(status),
            },
        }

    def close(self) -> None:
        for interface in self.interfaces.values():
            interface.DisconnectPort()
