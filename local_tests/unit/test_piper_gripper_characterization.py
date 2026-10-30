from copy import deepcopy
import math

import pytest

from services.piper.gripper_characterization import (
    GripperCycleSettings,
    PiperGripperCycleRunner,
    PiperGripperSetRunner,
    incremental_targets,
)


def arm(width=0.0988):
    return {
        "joints_rad": [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
        "gripper_width_m": width,
        "gripper": {"width_m": width, "effort_nm": 0.1, "errors": {}},
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
        self.reader.value["arms"][self.side]["gripper_width_m"] = width
        self.reader.value["arms"][self.side]["gripper"]["width_m"] = width


class Reader:
    def __init__(self):
        self.value = {"arms": {"left": arm(), "right": arm()}}
        self.interfaces = {
            side: Interface(self, side) for side in ("left", "right")
        }

    def snapshot(self):
        return deepcopy(self.value)


class DelayedReader(Reader):
    def __init__(self):
        super().__init__()
        self.snapshots = 0

    def snapshot(self):
        self.snapshots += 1
        value = super().snapshot()
        if self.snapshots == 1:
            value["arms"]["left"]["feedback"]["status_age_s"] = None
        return value


def test_incremental_targets_include_exact_goal():
    assert incremental_targets(0.025, 0.0, 0.01) == [0.015, 0.005, 0.0]
    assert incremental_targets(0.0, 0.025, 0.01) == [0.01, 0.02, 0.025]


def test_gripper_cycle_closes_and_restores_without_arm_commands():
    reader = Reader()
    result = PiperGripperCycleRunner(
        reader,
        GripperCycleSettings(step_m=0.02, effort=500),
        sleeper=lambda _: None,
    ).run(arm="right", execute=True)

    assert result["status"] == "passed"
    assert result["restored"] is True
    assert result["arm_motion_commanded"] is False
    assert reader.value["arms"]["right"]["gripper_width_m"] == pytest.approx(0.0988)
    assert reader.interfaces["left"].calls == []
    assert all(call[1] == 500 for call in reader.interfaces["right"].calls)


def test_gripper_cycle_plan_only_sends_nothing():
    reader = Reader()
    result = PiperGripperCycleRunner(reader, sleeper=lambda _: None).run(
        arm="right", execute=False
    )
    assert result["status"] == "planned"
    assert reader.interfaces["right"].calls == []


def test_gripper_cycle_waits_for_initial_feedback():
    reader = DelayedReader()
    result = PiperGripperCycleRunner(reader, sleeper=lambda _: None).run(
        arm="right", execute=False
    )
    assert result["status"] == "planned"
    assert reader.snapshots == 2


def test_gripper_set_closes_and_holds_without_arm_commands():
    reader = Reader()
    result = PiperGripperSetRunner(
        reader,
        GripperCycleSettings(step_m=0.02, effort=500),
        sleeper=lambda _: None,
    ).run(arm="right", target_m=0.0, execute=True)

    assert result["status"] == "passed"
    assert result["final_width_m"] == pytest.approx(0.0)
    assert result["arm_motion_commanded"] is False
    assert reader.interfaces["left"].calls == []


def test_gripper_set_observes_samples_and_rechecks_authorization():
    reader = Reader()
    samples = []
    authorizations = []
    result = PiperGripperSetRunner(
        reader,
        GripperCycleSettings(step_m=0.02, effort=500),
        sleeper=lambda _: None,
        sample_observer=lambda arm_name, state: samples.append(
            (arm_name, state["gripper_width_m"])
        ),
    ).run(
        arm="right",
        target_m=0.0,
        execute=True,
        authorization_guard=lambda: authorizations.append(True),
    )

    assert result["status"] == "passed"
    assert samples
    assert len(authorizations) >= len(reader.interfaces["right"].calls)


def test_gripper_set_plan_only_sends_nothing():
    reader = Reader()
    result = PiperGripperSetRunner(reader, sleeper=lambda _: None).run(
        arm="right", target_m=0.0, execute=False
    )
    assert result["status"] == "planned"
    assert reader.interfaces["right"].calls == []
