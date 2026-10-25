"""Build the operator receipt for Piper stop, hold, and recovery rules.

This module is administrative only. It does not import ``piper_sdk`` and cannot send
CAN frames. The receipt proves that the operator protocol was reviewed; it does not
claim that position hold or stop latency has been characterized.
"""

from __future__ import annotations

from datetime import datetime, timezone


ACKNOWLEDGEMENTS = (
    "software_estop_is_not_hold",
    "disable_is_not_hold",
    "no_automatic_reset_or_enable",
    "recovery_returns_to_read_only",
    "new_arming_lease_after_recovery",
)


def _policy() -> dict:
    return {
        "operator_emergency_stop": {
            "path": "physical_emergency_stop",
            "owner": "operator",
            "primary": True,
            "independent_of_agent_and_workstation": True,
        },
        "operational_interruption": {
            "path": "latched_position_hold",
            "owner": "future_motion_worker",
            "behavior": (
                "reject new motion and command a target derived from the last fresh "
                "measured pose; exact command rate and stability remain uncharacterized"
            ),
            "status": "implemented_with_fake_interfaces_hardware_uncharacterized",
            "safety_rated": False,
        },
        "firmware_emergency_stop": {
            "path": "EmergencyStop(0x01)",
            "owner": "operator_only",
            "behavior": "damped descent; not a static hold",
            "status": "not_exposed",
        },
        "disable": {
            "path": "DisableArm",
            "classification": "not_a_stop_or_hold_path",
            "risk": "the arm may lose support and fall",
        },
        "recovery": {
            "automatic": False,
            "agent_allowed": False,
            "destination": "read_only",
            "required_before_new_arming": [
                "mechanically support the arm and inspect the workspace",
                "perform any reset, fault clear, and enable manually",
                "run a fresh passive probe and require passed status",
                "inspect controller faults and feedback freshness",
                "issue a new time-bounded operator arming lease",
            ],
        },
    }


def _required_confirmation() -> dict:
    return {
        "operator": "non-empty operator name",
        "robot_asset_id": "non-empty identifier for the exact dual-arm system",
        "physical_estop": {
            "location": "where the operator reaches it",
            "reachable_during_test": True,
            "robot_mechanically_supported": True,
            "descent_envelope_clear": True,
            "activation_tested_on_this_robot": True,
            "reset_requires_deliberate_operator_action": True,
            "observed_behavior": "what happened when activated while stationary",
        },
        "acknowledgements": {key: True for key in ACKNOWLEDGEMENTS},
    }


def _non_empty(value, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{path} must be a non-empty string")
    return value.strip()


def _validate_confirmation(confirmation: dict) -> dict:
    if not isinstance(confirmation, dict):
        raise ValueError("confirmation must be an object")
    if confirmation.get("schema_version") != 1:
        raise ValueError("confirmation.schema_version must be 1")
    operator = _non_empty(confirmation.get("operator"), "confirmation.operator")
    asset_id = _non_empty(
        confirmation.get("robot_asset_id"), "confirmation.robot_asset_id"
    )

    physical = confirmation.get("physical_estop")
    if not isinstance(physical, dict):
        raise ValueError("confirmation.physical_estop must be an object")
    checked_physical = {
        "location": _non_empty(
            physical.get("location"), "confirmation.physical_estop.location"
        ),
        "observed_behavior": _non_empty(
            physical.get("observed_behavior"),
            "confirmation.physical_estop.observed_behavior",
        ),
    }
    for key in (
        "reachable_during_test",
        "robot_mechanically_supported",
        "descent_envelope_clear",
        "activation_tested_on_this_robot",
        "reset_requires_deliberate_operator_action",
    ):
        if physical.get(key) is not True:
            raise ValueError(f"confirmation.physical_estop.{key} must be true")
        checked_physical[key] = True

    acknowledgements = confirmation.get("acknowledgements")
    if not isinstance(acknowledgements, dict):
        raise ValueError("confirmation.acknowledgements must be an object")
    for key in ACKNOWLEDGEMENTS:
        if acknowledgements.get(key) is not True:
            raise ValueError(f"confirmation.acknowledgements.{key} must be true")

    return {
        "operator": operator,
        "robot_asset_id": asset_id,
        "physical_estop": checked_physical,
        "acknowledgements": {key: True for key in ACKNOWLEDGEMENTS},
        "notes": str(confirmation.get("notes", "")).strip(),
    }


def build_stop_plan_receipt(confirmation: dict | None = None) -> dict:
    """Return an auditable static-gate receipt without touching robot hardware."""
    receipt = {
        "schema_version": 1,
        "kind": "piper_operator_stop_plan",
        "status": "awaiting_operator_confirmation",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "can_frames_sent": False,
        "motion_commands_sent": False,
        "general_motion_authorized": False,
        "policy": _policy(),
        "state_rules": {
            "initial": "read_only",
            "motion_requires": "operator_arming_lease",
            "hold_or_stop_is_latched": True,
            "recovery_target": "read_only",
            "direct_stop_to_motion_transition": False,
        },
        "remaining_before_minimum_motion": [
            "complete the operator stop-plan confirmation for the exact robot",
            "implement an operator-only runner around the tested hold controller",
            "measure hold latency and post-request travel on the exact robot",
        ],
    }
    if confirmation is None:
        receipt["required_confirmation"] = _required_confirmation()
        return receipt

    receipt["operator_confirmation"] = _validate_confirmation(confirmation)
    receipt["status"] = "passed"
    receipt["operator_protocol_ready"] = True
    receipt["minimum_motion_characterization_ready"] = False
    return receipt
