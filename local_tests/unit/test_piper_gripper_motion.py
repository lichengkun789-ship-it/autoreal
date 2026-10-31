from pathlib import Path

import pytest

from services.piper.gripper_characterization import GripperCycleSettings
from services.piper.gripper_motion import (
    GripperMotionSettings,
    GripperProfile,
    PiperGripperMotionController,
)


def armed(_root):
    return {"armed": True, "since": 10.0, "until": 20.0, "by": "operator"}


def settings(maximum=3):
    profile = GripperProfile(
        arm="right",
        open_width_m=0.098,
        closed_width_m=0.0,
        runner_settings=GripperCycleSettings(),
        receipt_path=Path("/tmp/commissioning.json"),
        receipt_sha256="a" * 64,
    )
    return GripperMotionSettings(True, {"right": profile}, maximum)


class FakeRunner:
    calls = []

    def __init__(self, _reader, _settings, *, sleeper):
        self.sleeper = sleeper

    def run(self, *, arm, target_m, execute, authorization_guard):
        authorization_guard()
        self.calls.append((arm, target_m, execute))
        return {
            "status": "passed",
            "gripper_command_attempted": True,
            "final_width_m": target_m,
        }


def test_generic_gripper_supports_task_independent_open_close_and_move(tmp_path):
    FakeRunner.calls = []
    controller = PiperGripperMotionController(
        tmp_path,
        object(),
        settings(),
        arming_reader=armed,
        sleeper=lambda _seconds: None,
        runner_factory=FakeRunner,
    )

    closed = controller.execute("right", "close")
    moved = controller.execute("right", "move", 0.04)
    opened = controller.execute("right", "open")

    assert [row[1] for row in FakeRunner.calls] == [0.0, 0.04, 0.098]
    assert closed["action"] == "close"
    assert moved["target_width_m"] == 0.04
    assert opened["commands_remaining"] == 0
    assert list((tmp_path / "piper-motion").glob("agent_gripper_*.json"))


def test_generic_gripper_rejects_uncommissioned_arm_and_exhausted_budget(tmp_path):
    FakeRunner.calls = []
    controller = PiperGripperMotionController(
        tmp_path,
        object(),
        settings(maximum=1),
        arming_reader=armed,
        sleeper=lambda _seconds: None,
        runner_factory=FakeRunner,
    )
    with pytest.raises(PermissionError, match="not commissioned"):
        controller.validate_request("left", "open")
    with pytest.raises(ValueError, match="requires width_m"):
        controller.validate_request("right", "move")
    controller.execute("right", "close")
    with pytest.raises(PermissionError, match="budget is exhausted"):
        controller.validate_request("right", "open")
