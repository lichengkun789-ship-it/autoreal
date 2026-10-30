"""ClickBell Agent motion tests use fakes and never open CAN."""

from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from services import arming
from services.piper.click_bell_motion import (
    ClickBellPreviewController,
    ClickBellPreviewSettings,
)


START = [0.0, 0.2, -0.2, 0.0, 0.0, 0.0]


def arm_state(joints):
    return {
        "joints_rad": list(joints),
        "gripper_width_m": 0.0,
        "motor": {"effort_nm": [0.0] * 6, "speed_rad_s": [0.0] * 6},
        "feedback": {
            "joint_age_s": 0.01,
            "status_age_s": 0.01,
            "gripper_age_s": 0.01,
            "motor_age_s": 0.01,
        },
        "controller": {"arm_status": 0, "errors": {}},
    }


class Interface:
    def __init__(self, reader, side):
        self.reader = reader
        self.side = side
        self.calls = []

    def MotionCtrl_2(self, *args):
        self.calls.append(("mode", args))

    def JointCtrl(self, *native):
        self.calls.append(("joints", native))
        self.reader.value["arms"][self.side]["joints_rad"] = [
            value * math.pi / 180_000.0 for value in native
        ]

    def GetArmLowSpdInfoMsgs(self):
        motors = {
            f"motor_{index}": SimpleNamespace(
                can_id=0x260 + index,
                foc_status=SimpleNamespace(driver_enable_status=True),
            )
            for index in range(1, 7)
        }
        return SimpleNamespace(**motors)


class Reader:
    def __init__(self):
        self.value = {"arms": {side: arm_state(START) for side in ("left", "right")}}
        self.interfaces = {side: Interface(self, side) for side in ("left", "right")}

    def snapshot(self):
        return deepcopy(self.value)


def waypoint(j1):
    return {"joints_rad": [j1, 0.2, -0.2, 0.0, 0.0, 0.0]}


def evidence(tmp_path: Path):
    backend = {
        "arms": {
            "left": {"can_name": "can_l"},
            "right": {"can_name": "can_r"},
        },
        "joint_limit_margin_rad": 0.02,
        "max_joint_step_rad": 0.05,
        "workspace_bounds": {
            "right": {"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]}
        },
        "link_min_z_m": -1.0,
    }
    backend_path = tmp_path / "backend.json"
    backend_path.write_text(json.dumps(backend))
    plan = {
        "kind": "click_bell_readonly_press_path",
        "status": "passed",
        "arm": "right",
        "frame_id": "frame1",
        "geometry_admissible": True,
        "start_joints_rad": START,
        "stages": [
            {
                "approach_distance_m": 0.06,
                "plan": {
                    "status": "passed",
                    "waypoints": [waypoint(0.002), waypoint(0.004)],
                },
            },
            {
                "approach_distance_m": 0.0,
                "plan": {"status": "passed", "waypoints": [waypoint(0.006)]},
            },
        ],
    }
    plan_path = tmp_path / "plan.json"
    plan_path.write_text(json.dumps(plan))
    receipt = {
        "kind": "piper_operator_no_contact_approach",
        "status": "passed",
        "arm": "right",
        "contact_intended": False,
        "motion_command_attempted": True,
        "retreat_attempted": True,
        "retreat": {"status": "passed"},
        "plan_summary": {
            "frame_id": "frame1",
            "approach_distance_m": 0.06,
            "waypoints": 2,
        },
        "approach": {
            "status": "passed",
            "waypoints": [
                {"target_joints_rad": waypoint(0.002)["joints_rad"]},
                {"target_joints_rad": waypoint(0.004)["joints_rad"]},
            ],
        },
        "final_start_error_rad": 0.001,
        "press_plan_path": str(plan_path),
        "backend_options_path": str(backend_path),
        "motion_settings": {
            "speed_percent": 2,
            "waypoint_timeout_s": 5.0,
            "settle_tolerance_rad": 0.002,
        },
    }
    receipt_path = tmp_path / "commissioning.json"
    receipt_path.write_text(json.dumps(receipt))
    options = {
        "enabled": True,
        "mode": "approach_preview",
        "arm": "right",
        "plan": str(plan_path),
        "commissioning_receipt": str(receipt_path),
        "approach_distance_m": 0.06,
        "waypoints": 2,
        "speed_percent": 2,
        "waypoint_timeout_s": 5.0,
        "settle_tolerance_rad": 0.002,
    }
    return backend, options, receipt_path


def settings(tmp_path):
    backend, options, _ = evidence(tmp_path)
    return ClickBellPreviewSettings.from_options(options, backend_options=backend)


def test_agent_executes_commissioned_preview_and_returns_to_start(tmp_path):
    reader = Reader()
    root = tmp_path / "session"
    arming.write(root, True, minutes=5, by="operator")
    controller = ClickBellPreviewController(
        root, reader, settings(tmp_path), sleeper=lambda _: None
    )

    result = controller.execute()

    assert result["executed"] is True
    assert result["contact_intended"] is False
    assert result["retreat_status"] == "passed"
    assert result["final_start_error_rad"] < 0.01
    assert Path(result["receipt"]).is_file()
    assert len([call for call in reader.interfaces["right"].calls if call[0] == "joints"]) == 5
    assert reader.interfaces["left"].calls == []
    with pytest.raises(PermissionError, match="budget is exhausted"):
        controller.validate_request()


def test_preview_requires_a_live_operator_arming_lease(tmp_path):
    controller = ClickBellPreviewController(
        tmp_path / "session", Reader(), settings(tmp_path), sleeper=lambda _: None
    )
    with pytest.raises(PermissionError, match="not armed"):
        controller.validate_request()


def test_new_arming_lease_resets_the_one_call_budget(tmp_path):
    root = tmp_path / "session"
    arming.write(root, True, minutes=5)
    controller = ClickBellPreviewController(
        root, Reader(), settings(tmp_path), sleeper=lambda _: None
    )
    controller.execute()
    arming.write(root, False)
    arming.write(root, True, minutes=5)
    controller.validate_request()
    assert controller.status()["calls_remaining"] == 1


def test_tampered_or_mismatched_evidence_is_rejected(tmp_path):
    backend, options, receipt_path = evidence(tmp_path)
    receipt = json.loads(receipt_path.read_text())
    receipt["approach"]["waypoints"][1]["target_joints_rad"][0] = 0.2
    receipt_path.write_text(json.dumps(receipt))
    with pytest.raises(ValueError, match="did not execute"):
        ClickBellPreviewSettings.from_options(options, backend_options=backend)

    backend, options, _ = evidence(tmp_path)
    backend["workspace_bounds"]["right"]["max"][0] = 2.0
    with pytest.raises(ValueError, match="workspace_bounds"):
        ClickBellPreviewSettings.from_options(options, backend_options=backend)
