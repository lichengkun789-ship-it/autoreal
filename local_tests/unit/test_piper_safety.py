"""Piper target validation distinguishes hard limits from soft-margin recovery."""

import pytest

from services.piper.safety import validate_joint_target


PARKED = [0.0, 0.007, -0.002, 0.0, 0.0, 0.0]


def check(target):
    return validate_joint_target(PARKED, target, margin_rad=0.02, max_step_rad=0.05)


def test_current_pose_can_be_held_inside_soft_margin():
    report = check(PARKED)
    assert report["accepted"] is True
    assert report["within_soft_limits"] is False
    assert report["joints"][1]["soft_limit_action"] == "hold"
    assert report["joints"][2]["soft_limit_action"] == "hold"


def test_motion_from_soft_margin_must_escape_not_move_deeper():
    escaping = check([0.0, 0.012, -0.01, 0.0, 0.0, 0.0])
    assert escaping["accepted"] is True
    assert escaping["joints"][1]["soft_limit_action"] == "escape"
    assert escaping["joints"][2]["soft_limit_action"] == "escape"

    deeper = check([0.0, 0.006, -0.001, 0.0, 0.0, 0.0])
    assert deeper["accepted"] is False
    assert "moves_deeper_into_joint_limit_margin" in deeper["joints"][1]["reasons"]
    assert "moves_deeper_into_joint_limit_margin" in deeper["joints"][2]["reasons"]


def test_motion_from_safe_region_may_not_enter_margin_or_cross_hard_limit():
    current = [0.0, 0.03, -0.03, 0.0, 0.0, 0.0]
    margin = validate_joint_target(
        current,
        [0.0, 0.019, -0.03, 0.0, 0.0, 0.0],
        margin_rad=0.02,
        max_step_rad=0.05,
    )
    assert margin["accepted"] is False
    assert "outside_joint_limit_margin" in margin["joints"][1]["reasons"]

    hard = check([0.0, -0.001, -0.002, 0.0, 0.0, 0.0])
    assert hard["accepted"] is False
    assert "outside_hard_joint_limit" in hard["joints"][1]["reasons"]


def test_step_cap_still_applies_to_escape_motion():
    target = PARKED.copy()
    target[1] = 0.2
    report = check(target)
    assert report["accepted"] is False
    assert "per_call_step_exceeded" in report["joints"][1]["reasons"]
