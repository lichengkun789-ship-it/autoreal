"""Piper backend contract tests use fakes and never open CAN or command hardware."""

import base64
from copy import deepcopy
import json
import math
from types import SimpleNamespace

from PIL import Image
import pytest

from services.piper.backend import PiperBackend
from services.piper.camera import CameraFrame, camera_specs
from services.piper.probe import inspect_can_interface, run_readonly_probe
from services.piper.sdk_reader import PiperSdkReader


def arm_state(*, age=0.01, arm_status=0, errors=False):
    return {
        "joints_rad": [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
        "gripper_width_m": 0.04,
        "feedback": {
            "joint_time": 99.99,
            "joint_age_s": age,
            "joint_hz": 200.0,
            "status_time": 99.99,
            "status_age_s": age,
            "gripper_time": 99.99,
            "gripper_age_s": age,
        },
        "controller": {
            "control_mode": 1,
            "arm_status": arm_status,
            "motion_status": 0,
            "errors": {"joint_1_communication_error": errors},
        },
    }


class FakeReader:
    def __init__(self, left=None, right=None):
        self.value = {"measured_at": 100.0, "arms": {
            "left": left or arm_state(),
            "right": right or arm_state(),
        }}
        self.closed = False

    def snapshot(self):
        return deepcopy(self.value)

    def close(self):
        self.closed = True


def config(*, root=None, observation_profile="state-only", **options):
    return SimpleNamespace(
        backend_options=options,
        root=root,
        observation_profile=observation_profile,
    )


class FakeCameras:
    def __init__(self, *, depth=False):
        self.depth = depth
        self.calls = []
        self.closed = False

    def capture(self, names, *, timeout_ms):
        self.calls.append((tuple(names), timeout_ms))
        return {
            name: CameraFrame(
                name=name,
                serial=f"serial-{name}",
                color_rgb=Image.new("RGB", (8, 6), (40, 80, 120)),
                depth_mm=Image.new("I;16", (8, 6), 500) if self.depth else None,
                captured_at=100.0,
                device_timestamp_ms=1234.5,
                intrinsics={"width": 8, "height": 6, "fx": 4.0, "fy": 4.0},
            )
            for name in names
        }

    def close(self):
        self.closed = True


def test_tools_are_read_only_and_status_requires_fresh_healthy_feedback():
    backend = PiperBackend(config(), reader=FakeReader())
    assert backend.mutating == frozenset()
    assert [tool["name"] for tool in backend.tools()] == [
        "piper_status",
        "piper_observe",
        "piper_check_joints",
    ]
    status = backend.status()
    assert status["ready"] is True
    assert status["motion_tools_available"] is False

    backend.reader.value["arms"]["left"]["feedback"]["status_age_s"] = 1.0
    status = backend.status()
    assert status["ready"] is False
    assert "left status feedback is stale or missing" in status["readiness_reasons"]

    backend.reader.value["arms"]["right"]["controller"]["arm_status"] = 4
    backend.reader.value["arms"]["right"]["controller"]["errors"] = {"joint_2_angle_limit": True}
    status = backend.status()
    assert "right controller reports arm_status=4" in status["readiness_reasons"]
    assert "right controller reports joint errors" in status["readiness_reasons"]


def test_joint_check_accepts_small_valid_target_and_rejects_limits_and_steps():
    backend = PiperBackend(config(), reader=FakeReader())
    valid = [0.01, 0.2, -0.2, 0.0, 0.0, 0.0]
    value, images = backend.call("piper_check_joints", {"arm": "left", "positions": valid})
    assert images == []
    assert value["dry_run"] and value["state_ready"] and value["accepted"]

    outside_j2 = [0.0, -0.01, -0.2, 0.0, 0.0, 0.0]
    value, _ = backend.call("piper_check_joints", {"arm": "left", "positions": outside_j2})
    assert not value["accepted"]
    assert "outside_joint_limit_margin" in value["joints"][1]["reasons"]

    large_step = [0.2, 0.2, -0.2, 0.0, 0.0, 0.0]
    value, _ = backend.call("piper_check_joints", {"arm": "right", "positions": large_step})
    assert not value["accepted"]
    assert "per_call_step_exceeded" in value["joints"][0]["reasons"]


def test_stale_state_never_produces_an_accepted_target():
    backend = PiperBackend(config(), reader=FakeReader(left=arm_state(age=1.0)))
    target = [0.01, 0.2, -0.2, 0.0, 0.0, 0.0]
    value, _ = backend.call("piper_check_joints", {"arm": "left", "positions": target})
    assert value["target_within_limits"] is True
    assert value["state_ready"] is False
    assert value["accepted"] is False


def test_configuration_validation_and_close():
    reader = FakeReader()
    with pytest.raises(ValueError, match="unknown piper backend options"):
        PiperBackend(config(unknown=True), reader=reader)
    with pytest.raises(ValueError, match="feedback_max_age_s must be positive"):
        PiperBackend(config(feedback_max_age_s=0), reader=reader)
    backend = PiperBackend(config(), reader=reader)
    with pytest.raises(ValueError, match="requires arm and positions"):
        backend.validate("piper_check_joints", {"arm": "left"})
    backend.close()
    assert reader.closed


def test_rgb_observation_returns_images_and_writes_auditable_frame(tmp_path):
    reader = FakeReader()
    cameras = FakeCameras()
    backend = PiperBackend(
        config(
            root=tmp_path,
            observation_profile="rgb-only",
            cameras=[
                {"name": "cam_high", "serial": "serial-cam_high"},
                {"name": "cam_right_wrist", "serial": "serial-cam_right_wrist"},
            ],
            agent_image_width=4,
        ),
        reader=reader,
        cameras=cameras,
        clock=lambda: 100.0,
    )
    observe = next(tool for tool in backend.tools() if tool["name"] == "piper_observe")
    enum = observe["inputSchema"]["properties"]["cameras"]["items"]["enum"]
    assert enum == ["cam_high", "cam_right_wrist"]
    assert backend.status()["camera_observation"] == {
        "available": True,
        "profile": "rgb-only",
        "cameras": ["cam_high", "cam_right_wrist"],
    }

    value, images = backend.call("piper_observe", {"cameras": ["cam_high"]})
    assert cameras.calls == [(('cam_high',), 3000)]
    assert value["images_in_order"] == ["cam_high"]
    assert value["max_joint_movement_during_capture_rad"] == 0.0
    assert len(images) == 1
    assert images[0]["type"] == "image" and images[0]["mimeType"] == "image/jpeg"
    assert base64.b64decode(images[0]["data"]).startswith(b"\xff\xd8")

    frame_dir = tmp_path / "agent-workspace" / value["directory"]
    assert (frame_dir / "cam_high.jpg").is_file()
    metadata = json.loads((frame_dir / "meta.json").read_text())
    assert metadata["cameras"]["cam_high"]["serial"] == "serial-cam_high"
    assert metadata["cameras"]["cam_high"]["depth"] is None
    assert "data" not in (frame_dir / "meta.json").read_text()
    backend.close()
    assert cameras.closed and reader.closed


def test_rgbd_observation_saves_uint16_depth(tmp_path):
    backend = PiperBackend(
        config(
            root=tmp_path,
            observation_profile="rgbd",
            cameras=[{"name": "cam_high", "serial": "serial-cam_high"}],
        ),
        reader=FakeReader(),
        cameras=FakeCameras(depth=True),
    )
    value, _ = backend.call("piper_observe", {})
    frame_dir = tmp_path / "agent-workspace" / value["directory"]
    metadata = json.loads((frame_dir / "meta.json").read_text())
    assert metadata["depth_units"] == "mm (uint16, 0 = invalid)"
    assert (frame_dir / metadata["cameras"]["cam_high"]["depth"]).is_file()
    with Image.open(frame_dir / "cam_high_depth.png") as depth:
        assert depth.getpixel((0, 0)) == 500


def test_camera_configuration_is_explicit_and_validated(tmp_path):
    with pytest.raises(ValueError, match="non-empty list"):
        PiperBackend(
            config(root=tmp_path, observation_profile="rgb-only"),
            reader=FakeReader(),
        )
    with pytest.raises(ValueError, match="camera names must be unique"):
        camera_specs([
            {"name": "cam_high", "serial": "one"},
            {"name": "cam_high", "serial": "two"},
        ])
    with pytest.raises(ValueError, match="exactly name and serial"):
        camera_specs([{"name": "cam_high", "serial": "one", "position": "fixed"}])


def test_state_only_observe_neither_accepts_camera_selection_nor_opens_cameras():
    backend = PiperBackend(config(), reader=FakeReader())
    value, images = backend.call("piper_observe", {})
    assert images == []
    assert value["camera_observation"]["available"] is False
    with pytest.raises(ValueError, match="state-only"):
        backend.call("piper_observe", {"cameras": ["cam_high"]})


def test_sdk_reader_converts_units_and_disconnects_both_arms():
    interfaces = []

    class Interface:
        def __init__(self, can_name):
            self.can_name = can_name
            self.connected = False
            self.disconnected = False
            interfaces.append(self)

        def ConnectPort(self, *, piper_init=True):
            self.connected = True
            self.piper_init = piper_init

        def DisconnectPort(self):
            self.disconnected = True

        def GetArmJointMsgs(self):
            joints = {f"joint_{index}": index * 1000 for index in range(1, 7)}
            return SimpleNamespace(
                joint_state=SimpleNamespace(**joints), time_stamp=99.9, Hz=200.0
            )

        def GetArmStatus(self):
            errors = SimpleNamespace()
            status = SimpleNamespace(
                ctrl_mode=1,
                arm_status=0,
                motion_status=0,
                err_status=errors,
            )
            return SimpleNamespace(arm_status=status, time_stamp=99.8)

        def GetArmGripperMsgs(self):
            return SimpleNamespace(
                gripper_state=SimpleNamespace(grippers_angle=40_000),
                time_stamp=99.7,
            )

    reader = PiperSdkReader(
        {"left": {"can_name": "can_l"}, "right": {"can_name": "can_r"}},
        interface_factory=Interface,
        clock=lambda: 100.0,
    )
    snapshot = reader.snapshot()
    assert all(interface.connected for interface in interfaces)
    assert all(interface.piper_init is False for interface in interfaces)
    assert snapshot["arms"]["left"]["joints_rad"][0] == pytest.approx(math.radians(1))
    assert snapshot["arms"]["right"]["gripper_width_m"] == pytest.approx(0.04)
    reader.close()
    assert all(interface.disconnected for interface in interfaces)


def make_link(sys_class_net, name, *, flags="0x1", rx_packets=10):
    root = sys_class_net / name
    (root / "statistics").mkdir(parents=True)
    (root / "flags").write_text(flags)
    (root / "operstate").write_text("up")
    values = {
        "rx_packets": rx_packets,
        "tx_packets": 2,
        "rx_errors": 0,
        "tx_errors": 0,
        "rx_dropped": 0,
    }
    for key, value in values.items():
        (root / "statistics" / key).write_text(str(value))


def test_readonly_probe_collects_ready_samples_and_has_no_mutating_tools(tmp_path):
    sys_class_net = tmp_path / "net"
    make_link(sys_class_net, "can_l")
    make_link(sys_class_net, "can_r")
    options = {
        "arms": {
            "left": {"can_name": "can_l"},
            "right": {"can_name": "can_r"},
        }
    }

    def factory(config):
        return PiperBackend(config, reader=FakeReader())

    receipt = run_readonly_probe(
        options,
        samples=3,
        startup_seconds=0,
        interval_seconds=0,
        backend_factory=factory,
        sys_class_net=sys_class_net,
        sleeper=lambda _: None,
    )
    assert receipt["status"] == "passed"
    assert receipt["can_queries_sent"] is False
    assert receipt["motion_commands_sent"] is False
    assert receipt["tool_surface"]["mutating_tools"] == []
    assert receipt["summary"]["ready_samples"] == 3
    assert inspect_can_interface("can_l", sys_class_net=sys_class_net)["up"] is True


def test_readonly_probe_refuses_missing_or_duplicate_interfaces(tmp_path):
    options = {
        "arms": {
            "left": {"can_name": "missing_l"},
            "right": {"can_name": "missing_r"},
        }
    }
    receipt = run_readonly_probe(options, sys_class_net=tmp_path / "net")
    assert receipt["status"] == "failed"
    assert len(receipt["failure_reasons"]) == 2

    options["arms"]["right"]["can_name"] = "missing_l"
    with pytest.raises(ValueError, match="different CAN interfaces"):
        run_readonly_probe(options, sys_class_net=tmp_path / "net")


def test_readonly_probe_fails_if_any_can_transmission_is_observed(tmp_path):
    sys_class_net = tmp_path / "net"
    make_link(sys_class_net, "can_l")
    make_link(sys_class_net, "can_r")
    options = {
        "arms": {
            "left": {"can_name": "can_l"},
            "right": {"can_name": "can_r"},
        }
    }

    def factory(config):
        (sys_class_net / "can_r" / "statistics" / "tx_packets").write_text("3")
        return PiperBackend(config, reader=FakeReader())

    receipt = run_readonly_probe(
        options,
        samples=1,
        startup_seconds=0,
        backend_factory=factory,
        sys_class_net=sys_class_net,
        sleeper=lambda _: None,
    )
    assert receipt["status"] == "failed"
    assert receipt["can_interfaces"]["right"]["delta"]["tx_packets"] == 1
    assert "right" in receipt["failure_reasons"][0]


def test_sdk_reader_disconnects_first_arm_if_second_connection_fails():
    interfaces = []

    class Interface:
        def __init__(self, name):
            self.name = name
            self.disconnected = False
            interfaces.append(self)

        def ConnectPort(self, *, piper_init=True):
            if self.name == "bad":
                raise RuntimeError("connection failed")

        def DisconnectPort(self):
            self.disconnected = True

    with pytest.raises(RuntimeError, match="connection failed"):
        PiperSdkReader(
            {"left": {"can_name": "good"}, "right": {"can_name": "bad"}},
            interface_factory=Interface,
        )
    assert interfaces[0].disconnected is True
