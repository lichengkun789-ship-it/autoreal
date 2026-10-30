from types import SimpleNamespace

import pytest

from services.piper.gripper import PiperGripperCommandAdapter, validate_gripper_command


class Interface:
    def __init__(self):
        self.calls = []

    def GripperCtrl(self, *args):
        self.calls.append(args)


def test_gripper_adapter_converts_metres_and_preserves_sdk_effort():
    interfaces = {"left": Interface(), "right": Interface()}
    adapter = PiperGripperCommandAdapter(interfaces)

    report = adapter.command_opening("right", 0.025, effort=800)

    assert report == {"opening_m": 0.025, "effort": 800, "native_opening": 25_000}
    assert interfaces["right"].calls == [(25_000, 800, 0x01, 0x00)]
    assert interfaces["left"].calls == []


@pytest.mark.parametrize("opening", [-0.001, 0.101, float("nan")])
def test_gripper_adapter_rejects_invalid_opening_before_sdk(opening):
    interfaces = {"left": Interface(), "right": Interface()}
    adapter = PiperGripperCommandAdapter(interfaces)

    with pytest.raises(ValueError):
        adapter.command_opening("left", opening)

    assert interfaces["left"].calls == []


def test_gripper_validation_rejects_out_of_range_effort():
    with pytest.raises(ValueError, match="0..5000"):
        validate_gripper_command(0.05, 5001)
