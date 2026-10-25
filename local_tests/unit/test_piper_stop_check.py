import pytest

from services.piper.stop_check import build_stop_plan_receipt


def confirmation():
    return {
        "schema_version": 1,
        "operator": "operator",
        "robot_asset_id": "dual-piper-01",
        "physical_estop": {
            "location": "front-left of the workbench",
            "reachable_during_test": True,
            "robot_mechanically_supported": True,
            "descent_envelope_clear": True,
            "activation_tested_on_this_robot": True,
            "reset_requires_deliberate_operator_action": True,
            "observed_behavior": "controller entered stop state while both arms were supported",
        },
        "acknowledgements": {
            "software_estop_is_not_hold": True,
            "disable_is_not_hold": True,
            "no_automatic_reset_or_enable": True,
            "recovery_returns_to_read_only": True,
            "new_arming_lease_after_recovery": True,
        },
    }


def test_stop_plan_waits_for_operator_and_sends_nothing():
    receipt = build_stop_plan_receipt()
    assert receipt["status"] == "awaiting_operator_confirmation"
    assert receipt["can_frames_sent"] is False
    assert receipt["motion_commands_sent"] is False
    assert receipt["general_motion_authorized"] is False
    assert receipt["policy"]["firmware_emergency_stop"]["behavior"].startswith(
        "damped descent"
    )


def test_confirmed_static_gate_still_does_not_authorize_motion():
    receipt = build_stop_plan_receipt(confirmation())
    assert receipt["status"] == "passed"
    assert receipt["operator_protocol_ready"] is True
    assert receipt["minimum_motion_characterization_ready"] is False
    assert receipt["general_motion_authorized"] is False
    assert receipt["state_rules"]["recovery_target"] == "read_only"
    assert receipt["state_rules"]["direct_stop_to_motion_transition"] is False


@pytest.mark.parametrize(
    "path",
    [
        ("physical_estop", "reachable_during_test"),
        ("physical_estop", "robot_mechanically_supported"),
        ("acknowledgements", "no_automatic_reset_or_enable"),
        ("acknowledgements", "recovery_returns_to_read_only"),
    ],
)
def test_stop_plan_rejects_an_unconfirmed_safety_rule(path):
    value = confirmation()
    value[path[0]][path[1]] = False
    with pytest.raises(ValueError, match=path[1]):
        build_stop_plan_receipt(value)
