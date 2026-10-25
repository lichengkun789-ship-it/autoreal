"""Piper backend contract tests use fakes and never open CAN or command hardware."""

from copy import deepcopy
import math
from types import SimpleNamespace

import pytest

from services.piper.backend import PiperBackend
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


def config(**options):
    return SimpleNamespace(backend_options=options)


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


def test_sdk_reader_converts_units_and_disconnects_both_arms():
    interfaces = []

    class Interface:
        def __init__(self, can_name):
            self.can_name = can_name
            self.connected = False
            self.disconnected = False
            interfaces.append(self)

        def ConnectPort(self):
            self.connected = True

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
    assert snapshot["arms"]["left"]["joints_rad"][0] == pytest.approx(math.radians(1))
    assert snapshot["arms"]["right"]["gripper_width_m"] == pytest.approx(0.04)
    reader.close()
    assert all(interface.disconnected for interface in interfaces)
