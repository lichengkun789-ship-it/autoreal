"""Narrow Piper gripper command adapter.

The adapter owns unit conversion and hard command bounds. It is intentionally not an
Agent tool; operator characterization and admission remain separate work.
"""

from __future__ import annotations

import math


SIDES = ("left", "right")
GRIPPER_MIN_OPENING_M = 0.0
GRIPPER_MAX_OPENING_M = 0.10
GRIPPER_MAX_EFFORT = 5000
METRES_TO_MICROMETRES = 1_000_000.0


def validate_gripper_command(opening_m: float, effort: int) -> dict:
    opening = float(opening_m)
    if not math.isfinite(opening):
        raise ValueError("gripper opening must be finite")
    if not GRIPPER_MIN_OPENING_M <= opening <= GRIPPER_MAX_OPENING_M:
        raise ValueError("gripper opening must be in 0..0.10 m")
    if type(effort) is not int or not 0 <= effort <= GRIPPER_MAX_EFFORT:
        raise ValueError("gripper effort must be an integer in 0..5000")
    return {
        "opening_m": opening,
        "effort": effort,
        "native_opening": round(opening * METRES_TO_MICROMETRES),
    }


class PiperGripperCommandAdapter:
    """Command an already-connected SDK gripper without enabling or resetting it."""

    def __init__(self, interfaces: dict):
        if set(interfaces) != set(SIDES):
            raise ValueError("gripper adapter requires exactly left and right interfaces")
        self.interfaces = dict(interfaces)

    def command_opening(self, side: str, opening_m: float, *, effort: int = 1000) -> dict:
        if side not in SIDES:
            raise ValueError("side must be left or right")
        command = validate_gripper_command(opening_m, effort)
        self.interfaces[side].GripperCtrl(
            command["native_opening"], command["effort"], 0x01, 0x00
        )
        return command
