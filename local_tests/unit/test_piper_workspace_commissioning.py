import numpy as np
import pytest

from services.piper.kinematics import joint_frames
from services.piper.workspace_commissioning import derive_workspace_candidate


def plan():
    start = [0.0, 0.2, -0.2, 0.0, 0.0, 0.0]
    target = [0.01, 0.21, -0.2, 0.0, 0.0, 0.0]
    return {
        "kind": "click_bell_readonly_press_path",
        "status": "passed",
        "arm": "right",
        "frame_id": "frame1",
        "start_joints_rad": start,
        "stages": [{"plan": {"waypoints": [{"joints_rad": target}]}}],
    }


def test_candidate_wraps_fk_path_and_remains_unapproved():
    value = derive_workspace_candidate(
        plan(), arm="right", tcp_margin_m=(0.01, 0.02, 0.03), link_margin_m=0.01
    )
    points = np.asarray([joint_frames(row)[-1][:3, 3] for row in (
        plan()["start_joints_rad"],
        plan()["stages"][0]["plan"]["waypoints"][0]["joints_rad"],
    )])
    bounds = value["candidate"]["workspace_bounds"]["right"]

    assert value["status"] == "candidate"
    assert value["activation_allowed"] is False
    assert bounds["min"] == pytest.approx(
        np.min(points, axis=0) - [0.01, 0.02, 0.03], abs=1e-6
    )
    assert bounds["max"] == pytest.approx(
        np.max(points, axis=0) + [0.01, 0.02, 0.03], abs=1e-6
    )


def test_candidate_rejects_wrong_arm():
    with pytest.raises(ValueError, match="does not match"):
        derive_workspace_candidate(plan(), arm="left")
