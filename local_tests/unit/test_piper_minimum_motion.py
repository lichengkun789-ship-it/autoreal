from copy import deepcopy
from datetime import datetime, timedelta, timezone
import json
import math
from types import SimpleNamespace

import pytest

from services.piper.minimum_motion import (
    PiperMinimumMotionRunner,
    validate_probe_receipt,
    validate_software_hold_confirmation,
)


def arm_state(joints=None):
    return {
        "joints_rad": joints or [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
        "feedback": {"joint_age_s": 0.01, "status_age_s": 0.01},
        "controller": {"arm_status": 0, "motion_status": 0, "errors": {}},
    }


class Interface:
    def __init__(self, *, enabled=True, on_joint=None, enable_feedback_delay=0):
        self.enabled = enabled
        self.on_joint = on_joint
        self.enable_feedback_delay = enable_feedback_delay
        self.enable_feedback_reads = 0
        self.calls = []

    def GetArmLowSpdInfoMsgs(self):
        self.enable_feedback_reads += 1
        ready = self.enable_feedback_reads > self.enable_feedback_delay
        values = {
            f"motor_{index}": SimpleNamespace(
                can_id=0x260 + index if ready else 0,
                foc_status=SimpleNamespace(driver_enable_status=self.enabled)
            )
            for index in range(1, 7)
        }
        return SimpleNamespace(**values)

    def EnableArm(self, motor):
        self.calls.append(("enable", motor))
        self.enabled = True

    def MotionCtrl_2(self, *values):
        self.calls.append(("mode", values))

    def JointCtrl(self, *values):
        self.calls.append(("joint", values))
        if self.on_joint:
            self.on_joint([value * math.pi / 180_000 for value in values])


class Reader:
    def __init__(self, *, enabled=True):
        self.value = {
            "measured_at": 100.0,
            "arms": {"left": arm_state(), "right": arm_state()},
        }
        self.interfaces = {
            side: Interface(
                enabled=enabled,
                on_joint=lambda joints, side=side: self.value["arms"][side].__setitem__(
                    "joints_rad", joints
                ),
            )
            for side in ("left", "right")
        }

    def snapshot(self):
        return deepcopy(self.value)


class DelayedRightReader(Reader):
    def __init__(self):
        super().__init__()
        self.snapshots = 0

    def snapshot(self):
        self.snapshots += 1
        value = super().snapshot()
        if self.snapshots == 1:
            value["arms"]["right"]["feedback"] = {
                "joint_age_s": None,
                "status_age_s": None,
            }
        return value


def confirmation():
    return {
        "schema_version": 1,
        "kind": "piper_software_hold_only_confirmation",
        "operator": "operator",
        "robot_asset_id": "dual-piper-01",
        "acknowledgements": {
            "operator_at_keyboard": True,
            "workspace_clear": True,
            "robot_base_secured": True,
            "no_payload": True,
            "gravity_descent_path_clear": True,
            "no_physical_estop_available": True,
            "software_hold_is_not_safety_rated": True,
            "process_can_fail_before_hold": True,
            "can_or_power_loss_can_defeat_hold": True,
            "no_automatic_return_or_recovery": True,
        },
    }


def test_confirmation_requires_every_software_hold_risk():
    assert validate_software_hold_confirmation(confirmation())["operator"] == "operator"
    value = confirmation()
    value["acknowledgements"]["process_can_fail_before_hold"] = False
    with pytest.raises(ValueError, match="process_can_fail_before_hold"):
        validate_software_hold_confirmation(value)


def test_probe_receipt_must_be_recent_and_match_can_interfaces(tmp_path):
    path = tmp_path / "probe.json"
    receipt = {
        "kind": "piper_readonly_probe",
        "status": "passed",
        "motion_commands_sent": False,
        "tool_surface": {"mutating_tools": []},
        "can_interfaces": {
            "left": {"before": {"name": "can_l"}},
            "right": {"before": {"name": "can_r"}},
        },
        "finished_at": datetime.now(timezone.utc).isoformat(),
    }
    path.write_text(json.dumps(receipt))
    options = {
        "arms": {"left": {"can_name": "can_l"}, "right": {"can_name": "can_r"}}
    }
    assert validate_probe_receipt(path, options)["age_seconds"] >= 0

    receipt["finished_at"] = (
        datetime.now(timezone.utc) - timedelta(minutes=11)
    ).isoformat()
    path.write_text(json.dumps(receipt))
    with pytest.raises(ValueError, match="no older than"):
        validate_probe_receipt(path, options)


def test_plan_only_sends_no_command():
    reader = Reader()
    receipt = PiperMinimumMotionRunner(reader).run(
        arm="right",
        joint=1,
        delta_rad=0.002,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "planned"
    assert all(not interface.calls for interface in reader.interfaces.values())


def test_plan_waits_passively_for_both_sdk_receive_threads():
    reader = DelayedRightReader()
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=1,
        delta_rad=0.002,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "planned"
    assert reader.snapshots == 6
    assert all(not interface.calls for interface in reader.interfaces.values())


def test_execute_commands_one_joint_then_holds_the_exact_last_target():
    reader = Reader()
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=1,
        delta_rad=0.002,
        speed_percent=3,
        operator_lease_id="lease",
        execute=True,
    )
    assert receipt["status"] == "passed"
    assert receipt["hold"]["hold_target_source"] == "last_commanded_target"
    assert not reader.interfaces["left"].calls
    right = reader.interfaces["right"].calls
    assert [kind for kind, _ in right] == ["mode", "joint", "mode", "joint"]
    assert right[1][1] == right[3][1]


def test_interrupt_after_dispatch_still_latches_last_target_hold():
    reader = Reader()
    calls = 0

    def interrupt_first_sleep(_):
        nonlocal calls
        calls += 1
        if calls == 5:
            raise KeyboardInterrupt()

    receipt = PiperMinimumMotionRunner(reader, sleeper=interrupt_first_sleep).run(
        arm="left",
        joint=1,
        delta_rad=-0.002,
        speed_percent=2,
        operator_lease_id="lease",
        execute=True,
        drift_samples=1,
    )
    assert receipt["status"] == "failed"
    assert receipt["hold"]["state"] == "hold_latched"
    joints = [
        value for kind, value in reader.interfaces["left"].calls if kind == "joint"
    ]
    assert len(joints) == 2 and joints[0] == joints[1]
    assert "KeyboardInterrupt" in receipt["failure_reasons"][0]


def test_disabled_arm_refuses_unless_operator_allows_enable():
    reader = Reader(enabled=False)
    refused = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="left",
        joint=1,
        delta_rad=0.001,
        speed_percent=2,
        operator_lease_id="lease",
        execute=True,
    )
    assert refused["status"] == "failed"
    assert not reader.interfaces["left"].calls

    accepted = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="left",
        joint=1,
        delta_rad=0.001,
        speed_percent=2,
        operator_lease_id="lease-2",
        execute=True,
        allow_enable=True,
    )
    assert accepted["status"] == "passed"
    assert reader.interfaces["left"].calls[0] == ("enable", 7)


def test_hard_caps_reject_before_any_command():
    reader = Reader()
    receipt = PiperMinimumMotionRunner(reader).run(
        arm="right",
        joint=2,
        delta_rad=0.006,
        speed_percent=6,
        operator_lease_id="lease",
        execute=True,
    )
    assert receipt["status"] == "failed"
    assert len(receipt["failure_reasons"]) == 2
    assert all(not interface.calls for interface in reader.interfaces.values())


def test_exact_maximum_delta_is_not_rejected_by_float_roundoff():
    reader = Reader()
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=1,
        delta_rad=0.005,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "planned"
    assert receipt["target_validation"]["max_delta_rad"] == pytest.approx(0.005)


def test_plan_waits_for_complete_motor_enable_feedback():
    reader = Reader()
    reader.interfaces["right"].enable_feedback_delay = 2
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=1,
        delta_rad=0.002,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "planned"
    assert receipt["selected_arm_enabled_before"] is True
    assert reader.interfaces["right"].enable_feedback_reads == 3


def test_plan_rejects_an_unstable_pre_command_baseline():
    reader = Reader()
    original_snapshot = reader.snapshot

    def drifting_snapshot():
        value = original_snapshot()
        reader.value["arms"]["left"]["joints_rad"][1] += 0.0002
        return value

    reader.snapshot = drifting_snapshot
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=1,
        delta_rad=0.002,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "failed"
    assert "pre-command joint drift" in receipt["failure_reasons"][0]
    assert all(not interface.calls for interface in reader.interfaces.values())


def test_plan_records_but_does_not_gate_on_target_attainment_status():
    reader = Reader()
    reader.value["arms"]["right"]["controller"]["motion_status"] = 1
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=1,
        delta_rad=0.002,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "planned"
    assert receipt["baseline"]["motion_statuses"]["right"] == [1] * 5
    assert receipt["baseline"]["motion_status_is_advisory"] is True
    assert all(not interface.calls for interface in reader.interfaces.values())


def test_plan_allows_an_unchanged_joint_inside_the_hard_limit_margin():
    reader = Reader()
    reader.value["arms"]["right"]["joints_rad"][1] = 0.0086
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=1,
        delta_rad=0.002,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "planned"
    joint_2 = receipt["target_validation"]["joints"][1]
    assert joint_2["target_rad"] == pytest.approx(0.0086)
    assert joint_2["safe_min_rad"] == 0.0


def test_plan_keeps_the_extra_limit_margin_on_the_active_joint():
    reader = Reader()
    reader.value["arms"]["right"]["joints_rad"][1] = 0.018
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=2,
        delta_rad=0.001,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "failed"
    assert "outside_joint_limit_margin" in receipt["failure_reasons"][0]
    assert all(not interface.calls for interface in reader.interfaces.values())


def test_plan_rejects_an_unchanged_joint_outside_the_hard_limit():
    reader = Reader()
    reader.value["arms"]["right"]["joints_rad"][1] = -0.001
    receipt = PiperMinimumMotionRunner(reader, sleeper=lambda _: None).run(
        arm="right",
        joint=1,
        delta_rad=0.002,
        speed_percent=3,
        operator_lease_id="lease",
        execute=False,
    )
    assert receipt["status"] == "failed"
    assert "joint 2 feedback is outside known limits" in receipt["failure_reasons"][0]
    assert all(not interface.calls for interface in reader.interfaces.values())
