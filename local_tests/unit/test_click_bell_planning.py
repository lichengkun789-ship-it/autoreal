from copy import deepcopy

import numpy as np
import pytest

from services.piper.ik import rotation_from_rpy
from services.piper.kinematics import end_transform, rotation_to_rpy
from services.piper.click_bell.dynamic import build_dynamic_press_plan, validate_localization
from services.piper.click_bell.planning import plan_press_path


START = np.asarray([0.1, 0.7, -1.2, 0.2, 0.25, -0.15])
GOAL = np.asarray([0.16, 0.78, -1.28, 0.24, 0.18, -0.08])


def localization_for_goal(offset):
    target = end_transform(GOAL)
    rpy = rotation_to_rpy(target[:3, :3])
    contact = target[:3, 3] + rotation_from_rpy(rpy) @ offset
    return {
        "status": "passed",
        "frame_id": "frame1",
        "camera": "cam_right_wrist",
        "frame": "piper_right_base",
        "surface": {
            "status": "passed",
            "position_m": contact.tolist(),
            "normal_base": [0.0, 0.0, 1.0],
        },
    }, rpy


def test_press_path_accounts_for_tool_contact_offset_and_stays_read_only():
    offset = np.asarray([0.0, 0.0, 0.1])
    localization, rpy = localization_for_goal(offset)
    report = plan_press_path(
        START,
        localization,
        tool_contact_offset_tcp_m=offset,
        rpy_rad=rpy,
        approach_distances_m=(0.0,),
        waypoint_counts=(8,),
        workspace_min=[-1.0, -1.0, -1.0],
        workspace_max=[1.0, 1.0, 1.0],
        link_min_z_m=-1.0,
        tool_geometry_verified=True,
    )

    assert report["status"] == "passed"
    assert report["motion_authorized"] is False
    assert report["geometry_admissible"] is True
    assert report["execution_admissible"] is False
    assert report["stages"][0]["tcp_target_m"] == pytest.approx(
        end_transform(GOAL)[:3, 3], abs=1e-6
    )


def test_press_path_refuses_execution_admission_without_physical_geometry_checks():
    offset = np.asarray([0.0, 0.0, 0.1])
    localization, rpy = localization_for_goal(offset)
    report = plan_press_path(
        START,
        localization,
        tool_contact_offset_tcp_m=offset,
        rpy_rad=rpy,
        approach_distances_m=(0.0,),
        waypoint_counts=(8,),
    )

    assert report["status"] == "passed"
    assert report["geometry_admissible"] is False
    assert report["execution_admissible"] is False
    assert len(report["warnings"]) == 4


def test_press_path_can_explicitly_skip_workspace_bounds_only():
    offset = np.asarray([0.0, 0.0, 0.1])
    localization, rpy = localization_for_goal(offset)
    report = plan_press_path(
        START,
        localization,
        tool_contact_offset_tcp_m=offset,
        rpy_rad=rpy,
        approach_distances_m=(0.0,),
        waypoint_counts=(8,),
        workspace_required=False,
        link_min_z_m=-1.0,
        tool_geometry_verified=True,
    )

    assert report["status"] == "passed"
    assert report["geometry_admissible"] is True
    assert report["workspace_check_required"] is False
    assert report["workspace_bounds_configured"] is False
    assert "disabled by operator" in " ".join(report["warnings"])


def test_press_path_validates_stage_order():
    localization, rpy = localization_for_goal(np.zeros(3))
    with pytest.raises(ValueError, match="strictly descending"):
        plan_press_path(
            START,
            localization,
            tool_contact_offset_tcp_m=[0.0, 0.0, 0.0],
            rpy_rad=rpy,
            approach_distances_m=(0.01, 0.02),
            waypoint_counts=(4, 4),
        )


def test_dynamic_press_plan_tracks_new_surface_position():
    offset = np.asarray([0.0, 0.0, 0.1])
    localization, rpy = localization_for_goal(offset)
    localization["surface"].update(
        {
            "valid_depth_fraction": 0.95,
            "plane_rmse_mm": 0.8,
            "depth_fit_spread_95_mm": 3.0,
        }
    )
    template = {
        "tool_contact_offset_tcp_m": offset.tolist(),
        "target_rpy_rad": rpy.tolist(),
        "tool_geometry_verified": True,
        "frame_id": "commissioned-frame",
        "stages": [
            {
                "approach_distance_m": 0.06,
                "plan": {"waypoints": [{} for _ in range(64)]},
            },
            {
                "approach_distance_m": 0.02,
                "plan": {"waypoints": [{} for _ in range(24)]},
            },
            {
                "approach_distance_m": 0.005,
                "plan": {"waypoints": [{} for _ in range(16)]},
            },
            {
                "approach_distance_m": 0.0,
                "plan": {"waypoints": [{} for _ in range(8)]},
            },
        ],
    }
    backend = {
        "workspace_bounds": {
            "right": {"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]}
        },
        "link_min_z_m": -1.0,
        "joint_limit_margin_rad": 0.02,
        "max_joint_step_rad": 0.05,
    }
    first = build_dynamic_press_plan(
        template, localization, START, arm="right", backend_options=backend
    )
    moved = deepcopy(localization)
    moved["surface"]["position_m"][0] += 0.01
    second = build_dynamic_press_plan(
        template, moved, START, arm="right", backend_options=backend
    )

    assert first["execution_admissible"] is True
    assert second["execution_admissible"] is True
    assert len(first["stages"][0]["plan"]["waypoints"]) == 40
    assert len(first["stages"][1]["plan"]["waypoints"]) == 4
    assert second["stages"][-1]["contact_target_m"][0] == pytest.approx(
        first["stages"][-1]["contact_target_m"][0] + 0.01,
        abs=1e-6,
    )


def test_dynamic_press_plan_rejects_uncertain_depth():
    localization, _ = localization_for_goal(np.zeros(3))
    localization["surface"].update(
        {
            "valid_depth_fraction": 0.95,
            "plane_rmse_mm": 4.0,
            "depth_fit_spread_95_mm": 3.0,
        }
    )

    with pytest.raises(ValueError, match="plane RMSE"):
        validate_localization(localization, arm="right")


def test_dynamic_press_plan_can_skip_old_workspace_bounds():
    offset = np.asarray([0.0, 0.0, 0.1])
    localization, rpy = localization_for_goal(offset)
    localization["surface"].update(
        {
            "valid_depth_fraction": 0.95,
            "plane_rmse_mm": 0.8,
            "depth_fit_spread_95_mm": 3.0,
        }
    )
    template = {
        "tool_contact_offset_tcp_m": offset.tolist(),
        "target_rpy_rad": rpy.tolist(),
        "tool_geometry_verified": True,
        "stages": [
            {
                "approach_distance_m": 0.005,
                "plan": {"waypoints": [{} for _ in range(64)]},
            },
            {
                "approach_distance_m": 0.0,
                "plan": {"waypoints": [{} for _ in range(8)]},
            },
        ],
    }
    backend = {
        "link_min_z_m": -1.0,
        "joint_limit_margin_rad": 0.02,
        "max_joint_step_rad": 0.05,
    }

    report = build_dynamic_press_plan(
        template,
        localization,
        START,
        arm="right",
        backend_options=backend,
        workspace_check_enabled=False,
    )

    assert report["execution_admissible"] is True
    assert report["workspace_policy"] == "operator_disabled_kinematic_only"
