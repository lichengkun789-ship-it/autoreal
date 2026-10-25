from copy import deepcopy
import math

import pytest

from services.piper.hold import (
    HOLD_LATCHED,
    MOTION_ARMED,
    READ_ONLY,
    RECOVERY_REQUIRED,
    HoldDispatchError,
    MotionBlocked,
    PiperHoldController,
    PiperJointCommandAdapter,
)


def snapshot(*, age=0.01):
    def arm(joints):
        return {
            "joints_rad": joints,
            "feedback": {"joint_age_s": age, "status_age_s": age},
            "controller": {"arm_status": 0, "errors": {}},
        }

    return {
        "measured_at": 100.0,
        "arms": {
            "left": arm([0.1, 0.2, -0.3, 0.4, -0.5, 0.6]),
            "right": arm([-0.1, 0.3, -0.4, -0.2, 0.5, -0.6]),
        },
    }


class FakeReader:
    def __init__(self, value=None):
        self.value = value or snapshot()

    def snapshot(self):
        return deepcopy(self.value)


class FakeSdkInterface:
    def __init__(self, *, fail=False, before_send=None):
        self.fail = fail
        self.before_send = before_send
        self.commands = []

    def JointCtrl(self, *joints):
        if self.before_send:
            self.before_send()
        if self.fail:
            raise RuntimeError("fake SDK send failed")
        self.commands.append(joints)


def controller(*, reader=None, left=None, right=None):
    interfaces = {
        "left": left or FakeSdkInterface(),
        "right": right or FakeSdkInterface(),
    }
    value = PiperHoldController(
        reader or FakeReader(), PiperJointCommandAdapter(interfaces)
    )
    return value, interfaces


def arm(value, lease="lease-1"):
    return value.operator_arm_for_characterization(
        operator_lease_id=lease,
        operator_protocol_acknowledged=True,
        passive_probe_passed=True,
    )


def recover(value):
    return value.operator_recover_to_read_only(
        arm_mechanically_supported=True,
        workspace_inspected=True,
        operator_recovery_completed=True,
        faults_clear=True,
        passive_probe_passed=True,
    )


def test_hold_latches_before_sdk_calls_and_records_measured_targets():
    value, interfaces = controller()
    interfaces["left"].before_send = lambda: value.state == HOLD_LATCHED or 1 / 0
    interfaces["right"].before_send = lambda: value.state == HOLD_LATCHED or 1 / 0
    assert arm(value)["state"] == MOTION_ARMED

    result = value.request_position_hold("operator pause")

    assert result["state"] == HOLD_LATCHED
    assert result["motion_allowed"] is False
    assert result["hold_sdk_calls_returned"] is True
    assert interfaces["left"].commands == [
        tuple(round(v * 180_000 / math.pi) for v in snapshot()["arms"]["left"]["joints_rad"])
    ]
    assert len(interfaces["right"].commands) == 1
    with pytest.raises(MotionBlocked, match="hold_latched"):
        value.require_motion_allowed()


def test_repeated_hold_is_idempotent_and_sends_no_second_command():
    value, interfaces = controller()
    arm(value)
    first = value.request_position_hold("pause")
    second = value.request_position_hold("duplicate")
    assert first == second
    assert len(interfaces["left"].commands) == 1
    assert len(interfaces["right"].commands) == 1


def test_stale_feedback_latches_recovery_and_sends_nothing():
    value, interfaces = controller(reader=FakeReader(snapshot(age=1.0)))
    arm(value)
    with pytest.raises(HoldDispatchError, match="stale or missing") as error:
        value.request_position_hold("feedback watchdog")
    assert error.value.control_certain is False
    assert value.state == RECOVERY_REQUIRED
    assert not interfaces["left"].commands and not interfaces["right"].commands
    with pytest.raises(MotionBlocked, match="recovery_required"):
        value.require_motion_allowed()


def test_partial_sdk_failure_requires_manual_recovery():
    right = FakeSdkInterface(fail=True)
    value, interfaces = controller(right=right)
    arm(value)
    with pytest.raises(HoldDispatchError, match="not confirmed for both arms"):
        value.request_position_hold("command failure")
    assert value.state == RECOVERY_REQUIRED
    assert len(interfaces["left"].commands) == 1
    assert not interfaces["right"].commands


def test_manual_recovery_returns_only_to_read_only_and_needs_new_lease():
    value, _ = controller()
    arm(value)
    value.request_position_hold("test complete")
    with pytest.raises(PermissionError, match="workspace_inspected"):
        value.operator_recover_to_read_only(
            arm_mechanically_supported=True,
            workspace_inspected=False,
            operator_recovery_completed=True,
            faults_clear=True,
            passive_probe_passed=True,
        )
    assert recover(value)["state"] == READ_ONLY
    with pytest.raises(MotionBlocked, match="read_only"):
        value.require_motion_allowed()
    with pytest.raises(PermissionError, match="cannot be reused"):
        arm(value)
    assert arm(value, lease="lease-2")["state"] == MOTION_ARMED


def test_characterization_arm_is_fail_closed_and_sends_no_sdk_commands():
    value, interfaces = controller()
    with pytest.raises(PermissionError, match="operator protocol"):
        value.operator_arm_for_characterization(
            operator_lease_id="lease-1",
            operator_protocol_acknowledged=False,
            passive_probe_passed=True,
        )
    assert value.state == READ_ONLY
    assert not interfaces["left"].commands and not interfaces["right"].commands


def test_command_adapter_rejects_bad_values_before_calling_sdk():
    value, interfaces = controller()
    with pytest.raises(ValueError, match="finite"):
        value.commander.command_position("left", [0, 0, 0, 0, 0, math.nan])
    with pytest.raises(ValueError, match="side"):
        value.commander.command_position("middle", [0] * 6)
    assert not interfaces["left"].commands
    assert not hasattr(value, "reset")
    assert not hasattr(value, "enable")
    assert not hasattr(value, "emergency_stop")


def test_recorded_last_target_is_reused_for_only_the_selected_arm():
    value, interfaces = controller()
    arm(value)
    targets = {
        "left": [0.1, 0.2, -0.3, 0.4, -0.5, 0.6],
        "right": [-0.08, 0.3, -0.4, -0.2, 0.5, -0.6],
    }
    value.record_last_commanded_targets(targets, hold_sides=("right",))
    result = value.request_position_hold("bounded move complete")
    assert result["hold_target_source"] == "last_commanded_target"
    assert result["hold_sides"] == ["right"]
    assert not interfaces["left"].commands
    assert interfaces["right"].commands == [
        tuple(round(v * 180_000 / math.pi) for v in targets["right"])
    ]
