"""Build an operator-auditable cross-check of measured Piper joint poses."""

from __future__ import annotations

from datetime import datetime, timezone
import math


SIDES = ("left", "right")
JOINT_COUNT = 6


def _joint_degrees(sample: dict, side: str) -> list[float]:
    joints = sample["arms"][side]["joints_rad"]
    if len(joints) != JOINT_COUNT:
        raise ValueError(f"{side} arm must report exactly {JOINT_COUNT} joints")
    return [math.degrees(float(value)) for value in joints]


def _pose_stability(samples: list[dict]) -> tuple[dict, float]:
    by_arm = {}
    maximum = 0.0
    for side in SIDES:
        rows = [_joint_degrees(sample, side) for sample in samples]
        ranges = [max(values) - min(values) for values in zip(*rows)]
        maximum = max(maximum, *ranges)
        by_arm[side] = {"joint_range_deg": ranges}
    return by_arm, maximum


def _validate_confirmation(confirmation: dict) -> None:
    if confirmation.get("schema_version") != 1:
        raise ValueError("confirmation.schema_version must be 1")
    for field in ("operator", "reference_source"):
        if not isinstance(confirmation.get(field), str) or not confirmation[field].strip():
            raise ValueError(f"confirmation.{field} must be a non-empty string")
    for field in ("left_right_mapping_confirmed", "joint_order_and_sign_confirmed"):
        if confirmation.get(field) is not True:
            raise ValueError(f"confirmation.{field} must be true")
    arms = confirmation.get("arms")
    if not isinstance(arms, dict) or set(arms) != set(SIDES):
        raise ValueError("confirmation.arms must define exactly left and right")
    for side in SIDES:
        joints = arms[side].get("joints_deg") if isinstance(arms[side], dict) else None
        if (
            not isinstance(joints, list)
            or len(joints) != JOINT_COUNT
            or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in joints)
        ):
            raise ValueError(f"confirmation.arms.{side}.joints_deg must contain six numbers")


def build_pose_check_receipt(
    probe_receipt: dict,
    *,
    can_names: dict[str, str],
    confirmation: dict | None = None,
    skip_reason: str | None = None,
    tolerance_deg: float = 2.0,
    stability_tolerance_deg: float = 0.2,
) -> dict:
    """Compare a passive SDK capture with operator-supplied independent readings."""
    if tolerance_deg <= 0 or stability_tolerance_deg <= 0:
        raise ValueError("pose tolerances must be positive")
    if set(can_names) != set(SIDES):
        raise ValueError("can_names must define exactly left and right")
    if confirmation is not None and skip_reason is not None:
        raise ValueError("confirmation and skip_reason are mutually exclusive")
    if skip_reason is not None and not skip_reason.strip():
        raise ValueError("skip_reason must be a non-empty string")

    samples = probe_receipt.get("samples", [])
    receipt = {
        "schema_version": 1,
        "kind": "piper_pose_cross_check",
        "status": "failed",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "can_queries_sent": False,
        "motion_commands_sent": False,
        "probe_status": probe_receipt.get("status"),
        "can_names": can_names,
        "tolerance_deg": tolerance_deg,
        "stability_tolerance_deg": stability_tolerance_deg,
    }
    if probe_receipt.get("status") != "passed" or not samples:
        receipt["failure_reasons"] = list(
            probe_receipt.get("failure_reasons", ["passive probe did not pass"])
        )
        return receipt

    stability, maximum_drift = _pose_stability(samples)
    latest = samples[-1]
    receipt["automatic_checks"] = {
        "feedback_ready": all(sample.get("ready") is True for sample in samples),
        "no_can_transmit_observed": all(
            interface.get("delta", {}).get("tx_packets") == 0
            for interface in probe_receipt.get("can_interfaces", {}).values()
        ),
        "pose_stable": maximum_drift <= stability_tolerance_deg,
        "maximum_joint_drift_deg": maximum_drift,
        "arms": stability,
    }
    receipt["measured_pose"] = {
        "measured_at": latest.get("measured_at"),
        "arms": {
            side: {
                "joints_rad": latest["arms"][side]["joints_rad"],
                "joints_deg": _joint_degrees(latest, side),
                "gripper_width_m": latest["arms"][side]["gripper_width_m"],
                "controller": latest["arms"][side]["controller"],
            }
            for side in SIDES
        },
    }
    if not receipt["automatic_checks"]["no_can_transmit_observed"]:
        receipt["failure_reasons"] = ["CAN transmission was observed during pose capture"]
        return receipt
    if not receipt["automatic_checks"]["pose_stable"]:
        receipt["failure_reasons"] = [
            f"pose changed by {maximum_drift:.6f} deg during capture"
        ]
        return receipt

    if skip_reason is not None:
        receipt["status"] = "skipped"
        receipt["independent_confirmation"] = {
            "status": "skipped",
            "reason": skip_reason.strip(),
            "blocking": False,
        }
        return receipt

    if confirmation is None:
        receipt["status"] = "awaiting_operator_confirmation"
        receipt["required_confirmation"] = {
            "independent_source": "Use the teach pendant or another independent joint display.",
            "left_right_mapping": "Confirm each CAN interface belongs to the named physical arm.",
            "joint_order_and_sign": "Confirm J1..J6 order and signed angles, not only rough shape.",
        }
        return receipt

    _validate_confirmation(confirmation)
    comparisons = {}
    maximum_error = 0.0
    for side in SIDES:
        measured = receipt["measured_pose"]["arms"][side]["joints_deg"]
        reference = [float(value) for value in confirmation["arms"][side]["joints_deg"]]
        errors = [abs(actual - expected) for actual, expected in zip(measured, reference)]
        maximum_error = max(maximum_error, *errors)
        comparisons[side] = {
            "reported_joints_deg": measured,
            "reference_joints_deg": reference,
            "absolute_error_deg": errors,
            "within_tolerance": all(error <= tolerance_deg for error in errors),
        }
    receipt["operator_confirmation"] = {
        "operator": confirmation["operator"],
        "reference_source": confirmation["reference_source"],
        "left_right_mapping_confirmed": True,
        "joint_order_and_sign_confirmed": True,
        "notes": confirmation.get("notes", ""),
    }
    receipt["comparisons"] = comparisons
    receipt["maximum_reference_error_deg"] = maximum_error
    if all(result["within_tolerance"] for result in comparisons.values()):
        receipt["status"] = "passed"
    else:
        receipt["failure_reasons"] = [
            f"independent joint-angle error {maximum_error:.6f} deg exceeds {tolerance_deg:.6f} deg"
        ]
    return receipt
