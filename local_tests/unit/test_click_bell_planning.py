import numpy as np
import pytest

from services.piper.ik import rotation_from_rpy
from services.piper.kinematics import end_transform, rotation_to_rpy
from tasks.click_bell.planning import plan_press_path


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
