"""Dynamic ClickBell plans are fresh, lease-bound, and single-use."""

import json
from types import SimpleNamespace

import pytest

from services.piper.click_bell import dynamic


JOINTS = [0.0, 0.2, -0.2, 0.0, 0.0, 0.0]


class Reader:
    def snapshot(self):
        return {"arms": {"right": {"joints_rad": JOINTS}}}


def localization():
    return {
        "status": "passed",
        "frame_id": "frame1",
        "camera": "cam_right_wrist",
        "frame": "piper_right_base",
        "surface": {
            "status": "passed",
            "position_m": [0.3, 0.1, 0.04],
            "normal_base": [0.0, 0.0, 1.0],
            "valid_depth_fraction": 0.95,
            "plane_rmse_mm": 0.5,
            "depth_fit_spread_95_mm": 2.0,
        },
    }


def test_dynamic_plan_is_bound_to_frame_lease_and_single_use(tmp_path, monkeypatch):
    root = tmp_path / "session"
    frames = root / "agent-workspace" / "runtime" / "frames"
    frame = frames / "frame1"
    frame.mkdir(parents=True)
    (frame / "meta.json").write_text(
        json.dumps(
            {
                "captured_at": 100.0,
                "max_joint_movement_during_capture_rad": 0.0,
                "state_after": {"arms": {"right": {"joints_rad": JOINTS}}},
            }
        )
    )
    monkeypatch.setattr(dynamic, "localize_saved_frame", lambda *_a, **_k: localization())
    monkeypatch.setattr(
        dynamic,
        "build_dynamic_press_plan",
        lambda *_a, **_k: {
            "status": "passed",
            "frame_id": "frame1",
            "arm": "right",
            "dynamic": True,
            "execution_admissible": True,
            "start_joints_rad": JOINTS,
        },
    )
    planner = dynamic.ClickBellDynamicPlanner(
        root,
        frames,
        Reader(),
        SimpleNamespace(arm="right", plan={}, dynamic_workspace_check=True),
        {},
        clock=lambda: 100.0,
        arming_reader=lambda _root: {
            "armed": True,
            "since": 90.0,
            "until": 200.0,
            "by": "operator",
        },
    )

    plan = planner.plan("frame1", "cam_right_wrist")
    planner.validate(plan["plan_id"])
    consumed = planner.consume(plan["plan_id"])

    assert consumed["used"] is True
    assert consumed["source_frame_directory"] == str(frame)
    assert json.loads((root / "click-bell-plans" / f"{plan['plan_id']}.json").read_text())[
        "used"
    ] is True
    with pytest.raises(PermissionError, match="already consumed"):
        planner.validate(plan["plan_id"])
    with pytest.raises(PermissionError, match="budget is exhausted"):
        planner.plan("frame1", "cam_right_wrist")


def test_dynamic_plan_rejects_stale_frame(tmp_path):
    frames = tmp_path / "frames"
    frame = frames / "frame1"
    frame.mkdir(parents=True)
    (frame / "meta.json").write_text(
        json.dumps(
            {
                "captured_at": 0.0,
                "max_joint_movement_during_capture_rad": 0.0,
            }
        )
    )
    planner = dynamic.ClickBellDynamicPlanner(
        tmp_path,
        frames,
        Reader(),
        SimpleNamespace(arm="right", plan={}, dynamic_workspace_check=True),
        {},
        clock=lambda: 500.0,
        arming_reader=lambda _root: {"armed": True, "since": 1.0},
    )

    with pytest.raises(ValueError, match="stale"):
        planner.plan("frame1", "cam_right_wrist")
