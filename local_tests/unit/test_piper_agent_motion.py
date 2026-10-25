"""Agent motion tests use SDK-shaped fakes and never open CAN."""

from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from services import arming
from services.piper.agent_motion import (
    AgentMotionSettings,
    AgentMotionStopped,
    PiperAgentMotionController,
)
from services.piper.backend import PiperBackend


def arm_state(joints=None):
    return {
        "joints_rad": joints or [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
        "gripper_width_m": 0.04,
        "feedback": {
            "joint_age_s": 0.01,
            "status_age_s": 0.01,
            "gripper_age_s": 0.01,
        },
        "controller": {"arm_status": 0, "motion_status": 0, "errors": {}},
    }


class Interface:
    def __init__(self, on_joint):
        self.on_joint = on_joint
        self.calls = []

    def GetArmLowSpdInfoMsgs(self):
        motors = {
            f"motor_{index}": SimpleNamespace(
                can_id=0x260 + index,
                foc_status=SimpleNamespace(driver_enable_status=True),
            )
            for index in range(1, 7)
        }
        return SimpleNamespace(**motors)

    def MotionCtrl_2(self, *values):
        self.calls.append(("mode", values))

    def JointCtrl(self, *values):
        self.calls.append(("joint", values))
        self.on_joint([value * math.pi / 180_000 for value in values])


class Reader:
    def __init__(self):
        self.value = {
            "measured_at": 100.0,
            "arms": {"left": arm_state(), "right": arm_state()},
        }
        self.interfaces = {
            side: Interface(
                lambda joints, side=side: self.value["arms"][side].__setitem__(
                    "joints_rad", joints
                )
            )
            for side in ("left", "right")
        }
        self.closed = False

    def snapshot(self):
        return deepcopy(self.value)

    def close(self):
        self.closed = True


def characterization(path: Path, *, delta=0.005, speed=3):
    value = {
        "kind": "piper_minimum_motion_characterization",
        "status": "passed",
        "motion_command_attempted": True,
        "requested": {
            "arm": "right",
            "joint": 1,
            "delta_rad": delta,
            "speed_percent": speed,
            "execute": True,
            "allow_enable": False,
        },
        "can_interfaces": {
            "left": {"name": "can_l"},
            "right": {"name": "can_r"},
        },
        "hold": {
            "state": "hold_latched",
            "hold_sdk_calls_returned": True,
            "hold_target_source": "last_commanded_target",
            "hold_sides": ["right"],
        },
        "measurements": {
            "active_joint_travel_rad": abs(delta) * 0.9,
            "command_window_s": 0.5,
        },
        "operator_confirmation": {"robot_asset_id": "test-piper"},
    }
    path.write_text(json.dumps(value))
    return path


def options(receipt: Path, **overrides):
    motion = {
        "enabled": True,
        "allowed_moves": [
            {"arm": "right", "joint": 1, "directions": ["positive"]}
        ],
        "max_delta_rad": 0.005,
        "speed_percent": 3,
        "command_window_s": 0.5,
        "max_commands_per_arming": 2,
        "max_cumulative_delta_rad": 0.01,
        "characterization_receipts": [str(receipt)],
    }
    motion.update(overrides)
    return motion


def settings(tmp_path, **overrides):
    receipt = characterization(tmp_path / "characterization.json")
    return AgentMotionSettings.from_options(
        options(receipt, **overrides),
        arms={"left": {"can_name": "can_l"}, "right": {"can_name": "can_r"}},
    )


def test_enabled_motion_requires_matching_characterization(tmp_path):
    value = settings(tmp_path)
    assert value.enabled
    assert value.allowed == frozenset({("right", 1, "positive")})

    with pytest.raises(ValueError, match="no characterization covers"):
        settings(
            tmp_path,
            allowed_moves=[
                {"arm": "right", "joint": 1, "directions": ["negative"]}
            ],
        )
    with pytest.raises(ValueError, match="max_delta_rad"):
        settings(tmp_path, max_delta_rad=0.006)


def test_disabled_motion_is_a_readonly_fallback_when_evidence_was_removed(tmp_path):
    value = AgentMotionSettings.from_options(
        {
            "enabled": False,
            "characterization_receipts": [str(tmp_path / "removed.json")],
        },
        arms={"left": {"can_name": "can_l"}, "right": {"can_name": "can_r"}},
    )
    assert value.enabled is False and value.evidence == ()


def test_agent_executes_one_allowlisted_step_then_holds(tmp_path):
    reader = Reader()
    root = tmp_path / "session"
    arming.write(root, True, minutes=5, by="operator")
    controller = PiperAgentMotionController(
        root, reader, settings(tmp_path), sleeper=lambda _: None
    )

    value = controller.execute("right", 1, 0.005)

    assert value["executed"] is True
    assert value["measured_active_joint_travel_rad"] == pytest.approx(0.005, abs=1e-5)
    assert value["hold"]["state"] == "hold_latched"
    assert value["budget"]["commands_remaining"] == 1
    assert Path(value["receipt"]).is_file()
    assert not reader.interfaces["left"].calls
    assert [kind for kind, _ in reader.interfaces["right"].calls] == [
        "mode",
        "joint",
        "mode",
        "joint",
    ]


def test_disarm_during_command_window_requests_hold_and_stops(tmp_path):
    reader = Reader()
    root = tmp_path / "session"
    arming.write(root, True, minutes=5, by="operator")
    sleeps = 0

    def disarm_after_dispatch(_):
        nonlocal sleeps
        sleeps += 1
        # Four sleeps collect the stable baseline; the fifth follows dispatch.
        if sleeps == 5:
            arming.write(root, False, reason="operator disarmed")

    controller = PiperAgentMotionController(
        root, reader, settings(tmp_path), sleeper=disarm_after_dispatch
    )
    with pytest.raises(AgentMotionStopped, match="not armed") as stopped:
        controller.execute("right", 1, 0.002)

    assert stopped.value.control_certain is True
    assert [kind for kind, _ in reader.interfaces["right"].calls] == [
        "mode",
        "joint",
        "mode",
        "joint",
    ]
    assert controller.commands_used == 1


def test_allowlist_arming_and_per_lease_budgets_fail_closed(tmp_path):
    reader = Reader()
    root = tmp_path / "session"
    controller = PiperAgentMotionController(
        root, reader, settings(tmp_path), sleeper=lambda _: None
    )
    with pytest.raises(PermissionError, match="not armed"):
        controller.validate_request("right", 1, 0.002)
    arming.write(root, True, minutes=5)
    with pytest.raises(PermissionError, match="not in the operator allowlist"):
        controller.validate_request("right", 1, -0.002)
    controller.execute("right", 1, 0.005)
    controller.execute("right", 1, 0.005)
    with pytest.raises(PermissionError, match="command budget"):
        controller.validate_request("right", 1, 0.001)

    arming.write(root, False)
    arming.write(root, True, minutes=5)
    controller.validate_request("right", 1, 0.001)
    assert controller.commands_used == 0


def test_backend_tool_surface_is_config_gated(tmp_path):
    reader = Reader()
    receipt = characterization(tmp_path / "characterization.json")
    backend_options = {
        "arms": {"left": {"can_name": "can_l"}, "right": {"can_name": "can_r"}},
        "agent_motion": options(receipt),
    }
    config = SimpleNamespace(root=tmp_path / "session", backend_options=backend_options)
    backend = PiperBackend(config, reader=reader)
    try:
        assert backend.mutating == frozenset({"piper_move_joint"})
        assert "piper_move_joint" in [tool["name"] for tool in backend.tools()]
        status = backend.status()
        assert status["motion_tools_available"] is True
        assert status["agent_motion"]["arming"]["armed"] is False
    finally:
        backend.close()
