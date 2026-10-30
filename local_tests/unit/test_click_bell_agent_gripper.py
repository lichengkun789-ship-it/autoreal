"""ClickBell Agent gripper tests use fakes and never open CAN or RealSense."""

from copy import deepcopy
import json
from pathlib import Path

import pytest

from services import arming
from services.piper.click_bell_gripper import (
    ClickBellGripperController,
    ClickBellGripperSettings,
)


def arm(width=0.0988):
    return {
        "joints_rad": [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
        "gripper_width_m": width,
        "gripper": {"width_m": width, "effort_nm": 0.1, "errors": {}},
        "motor": {"effort_nm": [0.0] * 6, "speed_rad_s": [0.0] * 6},
        "feedback": {
            "joint_age_s": 0.01,
            "status_age_s": 0.01,
            "gripper_age_s": 0.01,
        },
        "controller": {"arm_status": 0, "errors": {}},
    }


class Interface:
    def __init__(self, reader, side):
        self.reader = reader
        self.side = side
        self.calls = []

    def GripperCtrl(self, opening, effort, status, zero):
        self.calls.append((opening, effort, status, zero))
        width = opening * 1e-6
        value = self.reader.value["arms"][self.side]
        value["gripper_width_m"] = width
        value["gripper"]["width_m"] = width


class Reader:
    def __init__(self):
        self.value = {"arms": {"left": arm(), "right": arm()}}
        self.interfaces = {
            side: Interface(self, side) for side in ("left", "right")
        }

    def snapshot(self):
        return deepcopy(self.value)


class FakeRecorder:
    counter = 0

    def __init__(self, root, cameras, camera_names, settings, **kwargs):
        type(self).counter += 1
        self.root = Path(root)
        self.recording_id = f"fake-{self.counter}"
        self.samples = []
        self.started = False

    def start(self):
        self.started = True

    def record_robot_sample(self, arm_name, state):
        self.samples.append((arm_name, float(state["gripper_width_m"])))

    def stop(self):
        assert self.started
        return {
            "status": "passed",
            "manifest": str(
                self.root / "recordings" / self.recording_id / "manifest.json"
            ),
            "camera_samples": 2,
            "robot_samples": len(self.samples),
        }


class Press:
    last_motion = None


def evidence(tmp_path: Path):
    backend = {
        "arms": {
            "left": {"can_name": "can_l"},
            "right": {"can_name": "can_r"},
        }
    }
    backend_path = tmp_path / "backend.json"
    backend_path.write_text(json.dumps(backend))
    settings = {
        "step_m": 0.01,
        "effort": 500,
        "target_tolerance_m": 0.003,
        "feedback_max_age_s": 0.25,
        "poll_interval_s": 0.05,
        "target_timeout_s": 3.0,
        "startup_timeout_s": 3.0,
        "max_arm_joint_drift_rad": 0.002,
    }
    receipt = {
        "kind": "piper_operator_gripper_cycle",
        "status": "passed",
        "arm": "right",
        "agent_tool_exposed": False,
        "arm_motion_commanded": False,
        "gripper_command_attempted": True,
        "initial_width_m": 0.0988,
        "final_width_m": 0.0980,
        "restored": True,
        "settings": settings,
        "backend_options_path": str(backend_path),
        "close": [
            {
                "command": {"opening_m": 0.0},
                "feedback": {"reached": True},
            }
        ],
        "restore": [
            {
                "command": {"opening_m": 0.0988},
                "feedback": {"reached": True},
            }
        ],
    }
    receipt_path = tmp_path / "gripper.json"
    receipt_path.write_text(json.dumps(receipt))
    options = {
        "enabled": True,
        "arm": "right",
        "commissioning_receipt": str(receipt_path),
    }
    return backend, options, receipt_path


def settings(tmp_path):
    backend, options, _ = evidence(tmp_path)
    return ClickBellGripperSettings.from_options(options, backend_options=backend)


def controller(tmp_path):
    root = tmp_path / "session"
    arming.write(root, True, minutes=5, by="operator")
    press = Press()
    value = ClickBellGripperController(
        root,
        Reader(),
        object(),
        ("cam_high", "cam_right_wrist", "cam_left_wrist"),
        settings(tmp_path),
        object(),
        press,
        sleeper=lambda _: None,
        recorder_factory=FakeRecorder,
    )
    return root, press, value


def test_agent_closes_then_opens_only_after_matching_press_retreat(tmp_path):
    root, press, gripper = controller(tmp_path)

    closed = gripper.execute("close")
    assert closed["phase"] == "closed"
    assert closed["recording_detail"]["robot_samples"] > 0
    with pytest.raises(PermissionError, match="only after"):
        gripper.execute("open")

    lease = arming.read(root)["since"]
    press.last_motion = {
        "press_id": "a" * 12,
        "status": "contact_unverified",
        "retreat_status": "passed",
        "authorization": {"lease_since": lease},
    }
    opened = gripper.execute("open")

    assert opened["phase"] == "opened"
    assert opened["press_id"] == "a" * 12
    evidence_value = gripper.completed_task_evidence("a" * 12)
    assert [item["action"] for item in evidence_value["actions"]] == ["close", "open"]
    assert len(evidence_value["recordings"]) == 2
    assert len(list((root / "piper-motion").glob("*.json"))) == 2


def test_agent_gripper_refuses_open_before_close_and_second_close(tmp_path):
    _, _, gripper = controller(tmp_path)
    with pytest.raises(PermissionError, match="must close"):
        gripper.validate_request("open")
    gripper.execute("close")
    with pytest.raises(PermissionError, match="budget is exhausted"):
        gripper.validate_request("close")


def test_agent_close_refuses_to_count_an_already_closed_gripper(tmp_path):
    _, _, gripper = controller(tmp_path)
    right = gripper.reader.value["arms"]["right"]
    right["gripper_width_m"] = 0.0
    right["gripper"]["width_m"] = 0.0
    with pytest.raises(PermissionError, match="requires start width"):
        gripper.validate_request("close")


def test_tampered_gripper_commissioning_is_rejected(tmp_path):
    backend, options, receipt_path = evidence(tmp_path)
    receipt = json.loads(receipt_path.read_text())
    receipt["settings"]["step_m"] = 0.02
    receipt_path.write_text(json.dumps(receipt))
    with pytest.raises(ValueError, match="hard limits"):
        ClickBellGripperSettings.from_options(options, backend_options=backend)
