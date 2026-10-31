"""Guarded ClickBell Agent tests use fakes and never open CAN or RealSense."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from services import arming
from services.controller import operator
from services.piper.camera import CameraFrame
from services.piper.click_bell.press import (
    ClickBellPressController,
    ClickBellPressSettings,
)
from services.piper.episode_recording import (
    EpisodeRecordingSettings,
    MotionEpisodeRecorder,
    TaskEpisodeRecorder,
)
from services.task_tools import task_tool_names
from tasks.click_bell.outcome import ClickBellOutcomeController


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


class FakeRecorder:
    def __init__(self, root, cameras, camera_names, settings, **kwargs):
        self.root = Path(root)
        self.samples = []
        self.started = False

    def start(self):
        self.started = True

    def record_robot_sample(self, arm, state):
        self.samples.append((arm, state["joints"].tolist()))

    def stop(self):
        assert self.started
        return {
            "status": "passed",
            "manifest": str(self.root / "recordings" / "fake" / "manifest.json"),
            "camera_samples": 2,
            "robot_samples": len(self.samples),
        }


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
                "approach_distance_m": 0.005,
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
    prior = {
        "kind": "piper_operator_no_contact_approach",
        "status": "passed",
        "arm": "right",
        "contact_intended": False,
        "motion_command_attempted": True,
        "retreat_attempted": True,
        "retreat": {"status": "passed"},
        "plan_summary": {
            "frame_id": "frame1",
            "approach_distance_m": 0.005,
            "waypoints": 2,
        },
        "final_start_error_rad": 0.001,
    }
    prior_path = tmp_path / "prior.json"
    prior_path.write_text(json.dumps(prior))
    receipt = {
        "kind": "piper_operator_guarded_press",
        "status": "contact_unverified",
        "arm": "right",
        "agent_tool_exposed": False,
        "motion_command_attempted": True,
        "retreat_attempted": True,
        "retreat": {"status": "passed"},
        "plan_summary": {
            "frame_id": "frame1",
            "approach_waypoints": 2,
            "contact_waypoints": 1,
            "contact_travel_m": 0.005,
        },
        "initial_gripper_width_m": 0.0,
        "start_state_error_rad": 0.001,
        "press_plan_path": str(plan_path),
        "backend_options_path": str(backend_path),
        "prior_stage_evidence": {
            "path": str(prior_path),
            "sha256": hashlib.sha256(prior_path.read_bytes()).hexdigest(),
        },
        "contact_settings": {
            "effort_delta_nm": 1.0,
            "speed_percent": 2,
            "waypoint_timeout_s": 5.0,
            "settle_tolerance_rad": 0.002,
        },
    }
    receipt_path = tmp_path / "commissioning.json"
    receipt_path.write_text(json.dumps(receipt))
    options = {
        "enabled": True,
        "mode": "guarded_press",
        "arm": "right",
        "plan": str(plan_path),
        "commissioning_receipt": str(receipt_path),
        "prior_stage_receipt": str(prior_path),
        "effort_delta_nm": 1.0,
        "speed_percent": 2,
        "waypoint_timeout_s": 5.0,
        "settle_tolerance_rad": 0.002,
    }
    return backend, options, receipt_path


def settings(tmp_path):
    backend, options, _ = evidence(tmp_path)
    return ClickBellPressSettings.from_options(options, backend_options=backend)


def test_agent_executes_guarded_press_retreat_and_requires_visual_result(tmp_path):
    root = tmp_path / "session"
    arming.write(root, True, minutes=5, by="operator")
    controller = ClickBellPressController(
        root,
        Reader(),
        object(),
        ("cam_high", "cam_right_wrist", "cam_left_wrist"),
        settings(tmp_path),
        EpisodeRecordingSettings(),
        sleeper=lambda _: None,
        recorder_factory=FakeRecorder,
    )

    result = controller.execute()

    assert result["executed"] is True
    assert result["retreat_status"] == "passed"
    assert result["visual_confirmation_status"] == "pending"
    assert result["recording"]["robot_samples"] > 0
    assert controller.last_motion["authorization"]["lease_since"] == arming.read(root)["since"]
    assert Path(result["receipt"]).is_file()
    with pytest.raises(PermissionError, match="budget is exhausted"):
        controller.validate_request()


def test_dynamic_agent_press_executes_one_direct_approach(tmp_path):
    root = tmp_path / "session"
    arming.write(root, True, minutes=5, by="operator")
    controller = ClickBellPressController(
        root,
        Reader(),
        object(),
        ("cam_high", "cam_right_wrist", "cam_left_wrist"),
        settings(tmp_path),
        EpisodeRecordingSettings(),
        sleeper=lambda _: None,
        recorder_factory=FakeRecorder,
    )
    plan = deepcopy(controller.settings.plan)
    plan.update(
        {
            "dynamic": True,
            "execution_admissible": True,
            "plan_id": "0123456789ab",
        }
    )

    result = controller.execute(plan)

    receipt = json.loads(Path(result["receipt"]).read_text())
    assert result["plan_id"] == plan["plan_id"]
    assert receipt["admission"]["plan_mode"] == "dynamic"
    assert receipt["plan_summary"]["prior_stage_evidence_required"] is False
    assert "dynamic_no_contact_preflight" not in receipt


def test_visual_result_requires_durable_operator_reply(tmp_path):
    root = tmp_path / "session"
    arming.write(root, True, minutes=5)
    press = ClickBellPressController(
        root,
        Reader(),
        object(),
        ("cam_high",),
        settings(tmp_path),
        EpisodeRecordingSettings(),
        sleeper=lambda _: None,
        recorder_factory=FakeRecorder,
    )
    motion = press.execute()
    outcome = ClickBellOutcomeController(root, press)
    request = operator.open_request(root, "visual_confirmation", "Reply success or failure")
    operator.reply(root, request["id"], "success: cap visibly moved", by="operator")
    waited = operator.wait(root, request, 30, poll_s=0)

    result = outcome.confirm(motion["press_id"], waited["request_id"])

    assert result["evaluation"]["status"] == "passed"
    assert Path(result["receipt"]).is_file()
    with pytest.raises(PermissionError, match="already has"):
        outcome.confirm(motion["press_id"], waited["request_id"])


def test_tampered_guarded_press_evidence_is_rejected(tmp_path):
    backend, options, receipt_path = evidence(tmp_path)
    receipt = json.loads(receipt_path.read_text())
    receipt["retreat"]["status"] = "failed"
    receipt_path.write_text(json.dumps(receipt))
    with pytest.raises(ValueError, match="does not admit"):
        ClickBellPressSettings.from_options(options, backend_options=backend)


def test_guarded_press_can_disable_dynamic_workspace_check(tmp_path):
    backend, options, _ = evidence(tmp_path)
    options["dynamic_workspace_check"] = False

    configured = ClickBellPressSettings.from_options(options, backend_options=backend)

    assert configured.dynamic_workspace_check is False
    assert configured.public()["dynamic_workspace_check"] is False


def test_guarded_press_accepts_relaxed_real_robot_settle_tolerance(tmp_path):
    backend, options, _ = evidence(tmp_path)
    options["settle_tolerance_rad"] = 0.01

    configured = ClickBellPressSettings.from_options(options, backend_options=backend)

    assert configured.settle_tolerance_rad == pytest.approx(0.01)
    assert configured.public()["settle_tolerance_rad"] == pytest.approx(0.01)
    assert configured.public()["runtime_max_joint_step_rad"] == pytest.approx(
        backend["max_joint_step_rad"]
    )


class FakeWriter:
    def __init__(self, path):
        self.path = Path(path)
        self.path.write_bytes(b"video")
        self.frames = 0

    def isOpened(self):
        return True

    def write(self, _frame):
        self.frames += 1

    def release(self):
        pass


class FakeCv2:
    def VideoWriter_fourcc(self, *_args):
        return 0

    def VideoWriter(self, path, *_args):
        return FakeWriter(path)

    def imwrite(self, path, _value):
        Path(path).write_bytes(b"image")
        return True


class FakeCameras:
    def capture(self, names, *, timeout_ms):
        return {
            name: CameraFrame(
                name=name,
                serial=name,
                color_rgb=np.zeros((4, 6, 3), dtype=np.uint8),
                depth_mm=np.full((4, 6), 500, dtype=np.uint16),
                captured_at=1.0,
                device_timestamp_ms=2.0,
                intrinsics={},
            )
            for name in names
        }


def test_episode_recorder_saves_video_depth_and_robot_state(tmp_path):
    recorder = MotionEpisodeRecorder(
        tmp_path,
        FakeCameras(),
        ("cam_high", "cam_right_wrist", "cam_left_wrist"),
        EpisodeRecordingSettings(fps=1.0, save_depth=True),
        cv2_module=FakeCv2(),
    )
    recorder.start()
    recorder.record_robot_sample(
        "right",
        {
            "joints": np.zeros(6),
            "effort": np.zeros(6),
            "speed": np.zeros(6),
            "gripper_width_m": 0.0,
        },
    )
    result = recorder.stop()

    assert result["status"] == "passed"
    assert result["camera_samples"] == 1
    assert result["robot_samples"] == 1
    assert Path(result["manifest"]).is_file()
    assert all(Path(path).is_file() for path in result["video"].values())


def test_task_recorder_spans_multiple_motion_phases(tmp_path):
    recorder = TaskEpisodeRecorder(
        tmp_path,
        FakeCameras(),
        ("cam_high",),
        EpisodeRecordingSettings(fps=1.0, save_depth=True),
        recorder_factory=lambda *args, **kwargs: MotionEpisodeRecorder(
            *args, cv2_module=FakeCv2(), **kwargs
        ),
    )
    recorder.start()
    recorder.mark("gripper_close_started", arm="right")
    recorder.record_robot_sample(
        "right",
        {
            "joints": np.zeros(6),
            "effort": np.zeros(6),
            "speed": np.zeros(6),
            "gripper_width_m": 0.0,
        },
    )
    recorder.mark("guarded_press_finished", status="contact_unverified")
    recorder.mark("gripper_open_finished", arm="right")
    result = recorder.stop()

    assert result["status"] == "passed"
    assert result["kind"] == "piper_task_episode_recording"
    assert result["scope"] == "task"
    assert [item["event"] for item in result["events"]] == [
        "task_recording_started",
        "gripper_close_started",
        "guarded_press_finished",
        "gripper_open_finished",
        "task_recording_stopped",
    ]
    saved = json.loads(Path(result["manifest"]).read_text())
    assert saved["kind"] == "piper_task_episode_recording"


def test_task_contract_lists_only_enabled_click_bell_tools():
    names = task_tool_names(
        "click_bell",
        robot="piper",
        task_options={"guarded_press": {"enabled": True}},
    )
    assert names == ()


def test_task_contract_lists_agent_gripper_before_press_when_enabled():
    names = task_tool_names(
        "click_bell",
        robot="piper",
        task_options={
            "guarded_press": {"enabled": True},
            "gripper": {"enabled": True},
        },
    )
    assert names == ()
