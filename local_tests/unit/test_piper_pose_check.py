import copy
import math

from services.piper.pose_check import build_pose_check_receipt


def sample(left=None, right=None):
    left = left or [0.0] * 6
    right = right or [0.1] * 6
    arm = lambda joints: {
        "joints_rad": joints,
        "gripper_width_m": 0.05,
        "controller": {"arm_status": 0, "errors": {}},
    }
    return {
        "ready": True,
        "measured_at": 100.0,
        "arms": {"left": arm(left), "right": arm(right)},
    }


def probe_receipt(samples=None):
    samples = samples or [sample(), sample()]
    return {
        "status": "passed",
        "samples": samples,
        "can_interfaces": {
            "left": {"delta": {"tx_packets": 0}},
            "right": {"delta": {"tx_packets": 0}},
        },
    }


def confirmation(left=None, right=None):
    return {
        "schema_version": 1,
        "operator": "operator",
        "reference_source": "independent display",
        "left_right_mapping_confirmed": True,
        "joint_order_and_sign_confirmed": True,
        "arms": {
            "left": {"joints_deg": left or [0.0] * 6},
            "right": {"joints_deg": right or [math.degrees(0.1)] * 6},
        },
    }


def build(probe=None, operator_confirmation=None):
    return build_pose_check_receipt(
        probe or probe_receipt(),
        can_names={"left": "can_l", "right": "can_r"},
        confirmation=operator_confirmation,
    )


def test_pose_check_waits_for_independent_operator_confirmation():
    receipt = build()
    assert receipt["status"] == "awaiting_operator_confirmation"
    assert receipt["motion_commands_sent"] is False
    assert receipt["measured_pose"]["arms"]["right"]["joints_deg"][0] == math.degrees(0.1)


def test_pose_check_passes_matching_independent_readings():
    receipt = build(operator_confirmation=confirmation())
    assert receipt["status"] == "passed"
    assert receipt["maximum_reference_error_deg"] == 0.0


def test_pose_check_rejects_large_reference_error():
    values = confirmation()
    values["arms"]["right"]["joints_deg"][2] += 3.0
    receipt = build(operator_confirmation=values)
    assert receipt["status"] == "failed"
    assert receipt["comparisons"]["right"]["within_tolerance"] is False


def test_pose_check_rejects_motion_during_capture():
    moving = sample()
    moved = copy.deepcopy(moving)
    moved["arms"]["left"]["joints_rad"][0] = math.radians(0.3)
    receipt = build(probe=probe_receipt([moving, moved]))
    assert receipt["status"] == "failed"
    assert receipt["automatic_checks"]["pose_stable"] is False


def test_pose_check_rejects_observed_can_transmission():
    probe = probe_receipt()
    probe["can_interfaces"]["right"]["delta"]["tx_packets"] = 1
    receipt = build(probe=probe)
    assert receipt["status"] == "failed"
    assert receipt["automatic_checks"]["no_can_transmit_observed"] is False
