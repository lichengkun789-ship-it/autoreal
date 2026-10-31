import math

import numpy as np
import pytest

from services.piper.ik import numerical_jacobian, plan_pose, solve
from services.piper.kinematics import end_transform, rotation_to_rpy


START = np.asarray([0.1, 0.7, -1.2, 0.2, 0.25, -0.15])
GOAL = np.asarray([0.16, 0.78, -1.28, 0.24, 0.18, -0.08])


def test_numerical_jacobian_is_finite_and_full_rank():
    jacobian = numerical_jacobian(START)
    assert jacobian.shape == (6, 6)
    assert np.all(np.isfinite(jacobian))
    assert np.linalg.matrix_rank(jacobian) == 6


def test_solver_recovers_a_nearby_fk_target():
    target = end_transform(GOAL)
    joints, converged, report = solve(START, target)

    assert converged is True
    assert report["position_error_mm"] <= 0.5
    assert report["rotation_error_deg"] <= 0.5
    measured = end_transform(joints)
    assert measured[:3, 3] == pytest.approx(target[:3, 3], abs=5e-4)


def test_plan_pose_checks_a_straight_path_without_authorizing_motion():
    target = end_transform(GOAL)
    report = plan_pose(
        START,
        target[:3, 3],
        rpy_rad=rotation_to_rpy(target[:3, :3]),
        waypoints=8,
        workspace_min=[-1.0, -1.0, -1.0],
        workspace_max=[1.0, 1.0, 1.0],
    )

    assert report["status"] == "passed"
    assert report["dry_run"] is True
    assert report["motion_authorized"] is False
    assert report["checks"]["workspace_bounds"] is True
    assert report["waypoints_planned"] >= 1


def test_plan_pose_preserves_current_orientation_when_omitted():
    start = end_transform(START)
    report = plan_pose(
        START,
        [0.005, 0.0, 0.0],
        relative=True,
        waypoints=4,
        workspace_min=[-1.0, -1.0, -1.0],
        workspace_max=[1.0, 1.0, 1.0],
    )

    assert report["status"] == "passed"
    assert report["target"]["position_m"] == pytest.approx(
        start[:3, 3] + [0.005, 0.0, 0.0], abs=1e-6
    )
    assert report["target_joints_rad"] is not None
    assert report["waypoints"][-1]["jacobian_min_singular_value"] > 0
    assert report["waypoints"][-1]["jacobian_condition_number"] > 1


def test_plan_pose_targets_configured_tool_point_instead_of_sdk_end():
    start = end_transform(START)
    offset = np.asarray([0.0, 0.0, 0.1358])
    tool_start = start[:3, 3] + start[:3, :3] @ offset
    report = plan_pose(
        START,
        [0.004, 0.0, 0.0],
        relative=True,
        waypoints=4,
        workspace_min=[-1.0, -1.0, -1.0],
        workspace_max=[1.0, 1.0, 1.0],
        tool_offset_tcp_m=offset,
    )

    assert report["status"] == "passed"
    assert report["start"]["position_m"] == pytest.approx(tool_start, abs=1e-6)
    assert report["target"]["position_m"] == pytest.approx(
        tool_start + [0.004, 0.0, 0.0], abs=1e-6
    )
    assert report["tool"]["offset_tcp_m"] == pytest.approx(offset)


def test_plan_pose_rejects_a_joint_jump_over_the_configured_waypoint_cap():
    target = end_transform(GOAL)
    report = plan_pose(
        START,
        target[:3, 3],
        rpy_rad=rotation_to_rpy(target[:3, :3]),
        waypoints=2,
        max_joint_step_rad=0.001,
    )

    assert report["status"] == "failed"
    assert report["checks"]["joint_waypoint_step"] is True
    assert "joint waypoint step exceeds configured maximum" in " ".join(
        report["failures"][0]["reasons"]
    )


def test_plan_pose_rejects_an_unreachable_target():
    report = plan_pose(START, [2.0, 0.0, 2.0], waypoints=4)

    assert report["status"] == "failed"
    assert report["motion_authorized"] is False
    assert any(
        "IK did not converge" in reason
        for failure in report["failures"]
        for reason in failure["reasons"]
    )


def test_plan_pose_validates_input():
    with pytest.raises(ValueError, match="waypoints"):
        plan_pose(START, [0.3, 0.0, 0.2], waypoints=1)
    with pytest.raises(ValueError, match="configured together"):
        plan_pose(START, [0.3, 0.0, 0.2], workspace_min=[0.0, 0.0, 0.0])
    with pytest.raises(ValueError, match="finite"):
        plan_pose(START, [math.nan, 0.0, 0.2])
    with pytest.raises(ValueError, match="maximum joint step"):
        plan_pose(START, [0.3, 0.0, 0.2], max_joint_step_rad=0.0)
