"""Robot-independent validation for Piper joint targets."""

from __future__ import annotations

import math


JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = tuple(
    (math.radians(low), math.radians(high))
    for low, high in (
        (-150.0, 150.0),
        (0.0, 180.0),
        (-170.0, 0.0),
        (-100.0, 100.0),
        (-70.0, 70.0),
        (-120.0, 120.0),
    )
)


def validate_joint_target(
    current: list[float] | tuple[float, ...],
    target: list[float] | tuple[float, ...],
    *,
    margin_rad: float,
    max_step_rad: float,
) -> dict:
    if len(current) != 6 or len(target) != 6:
        raise ValueError("Piper joint targets must contain exactly 6 values")
    if margin_rad < 0 or max_step_rad <= 0:
        raise ValueError("joint margin must be non-negative and max step must be positive")

    violations = []
    joints = []
    for index, (measured, requested, limits) in enumerate(zip(current, target, JOINT_LIMITS_RAD), start=1):
        measured = float(measured)
        requested = float(requested)
        if not math.isfinite(measured) or not math.isfinite(requested):
            raise ValueError(f"joint {index} values must be finite")
        lower, upper = limits
        safe_lower, safe_upper = lower + margin_rad, upper - margin_rad
        delta = abs(requested - measured)
        reasons = []
        if not safe_lower <= requested <= safe_upper:
            reasons.append("outside_joint_limit_margin")
        if delta > max_step_rad:
            reasons.append("per_call_step_exceeded")
        row = {
            "joint": index,
            "current_rad": measured,
            "target_rad": requested,
            "delta_rad": delta,
            "safe_min_rad": safe_lower,
            "safe_max_rad": safe_upper,
            "reasons": reasons,
        }
        joints.append(row)
        if reasons:
            violations.append(row)
    return {
        "accepted": not violations,
        "max_delta_rad": max(row["delta_rad"] for row in joints),
        "joints": joints,
        "violations": violations,
    }
