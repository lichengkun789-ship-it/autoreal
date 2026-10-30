from copy import deepcopy
import math
from types import SimpleNamespace

import pytest

from services.piper.contact_motion import ContactStopSettings, PiperGuardedPressExecutor


def arm_state(joints, *, effort=None, speed=None):
    return {
        "joints_rad": list(joints),
        "gripper_width_m": 0.0,
        "motor": {
            "effort_nm": list(effort or [0.0] * 6),
            "speed_rad_s": list(speed or [0.0] * 6),
        },
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
        joints = [value * math.pi / 180_000.0 for value in native]
        if self.reader.freeze_at_j1 is None or not math.isclose(
            joints[0], self.reader.freeze_at_j1, abs_tol=2e-5
        ):
            self.reader.value["arms"][self.side]["joints_rad"] = joints

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
        zero = [0.0, 0.2, -0.2, 0.0, 0.0, 0.0]
        self.value = {
            "arms": {"left": arm_state(zero), "right": arm_state(zero)}
        }
        self.freeze_at_j1 = None
        self.contact_after_snapshots = None
        self.snapshots = 0
        self.interfaces = {
            side: Interface(self, side) for side in ("left", "right")
        }

    def snapshot(self):
        self.snapshots += 1
        if (
            self.contact_after_snapshots is not None
            and self.snapshots >= self.contact_after_snapshots
        ):
            self.value["arms"]["right"]["motor"]["effort_nm"][2] = 1.0
        return deepcopy(self.value)


def waypoint(j1):
    return {
        "joints_rad": [j1, 0.2, -0.2, 0.0, 0.0, 0.0],
        "accepted": True,
    }


def press_plan(*, admitted=True):
    return {
        "kind": "click_bell_readonly_press_path",
        "status": "passed",
        "arm": "right",
        "frame_id": "frame1",
        "geometry_admissible": admitted,
        "start_joints_rad": [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
        "stages": [
            {
                "approach_distance_m": 0.005,
                "plan": {"status": "passed", "waypoints": [waypoint(0.002)]},
            },
            {
                "approach_distance_m": 0.0,
                "plan": {"status": "passed", "waypoints": [waypoint(0.004)]},
            },
        ],
    }


def no_contact_plan(*, admitted=True):
    value = press_plan(admitted=admitted)
    value["stages"][0] = {
        "approach_distance_m": 0.06,
        "plan": {
            "status": "passed",
            "waypoints": [waypoint(0.002), waypoint(0.004)],
        },
    }
    return value


def staged_no_contact_plan():
    value = no_contact_plan()
    value["stages"].insert(
        1,
        {
            "approach_distance_m": 0.02,
            "plan": {"status": "passed", "waypoints": [waypoint(0.006)]},
        },
    )
    return value


def prior_stage_receipt():
    return {
        "kind": "piper_operator_no_contact_approach",
        "status": "passed",
        "arm": "right",
        "contact_intended": False,
        "motion_command_attempted": True,
        "retreat_attempted": True,
        "retreat": {"status": "passed"},
        "plan_summary": {
            "frame_id": "frame1",
            "approach_distance_m": 0.06,
            "waypoints": 2,
        },
        "final_start_error_rad": 0.001,
    }


def contact_stage_receipt():
    value = prior_stage_receipt()
    value["plan_summary"].update(approach_distance_m=0.005, waypoints=1)
    return value


def settings():
    return ContactStopSettings(
        effort_delta_nm=0.5,
        consecutive_samples=2,
        poll_interval_s=0.001,
        waypoint_timeout_s=0.01,
        settle_tolerance_rad=0.0005,
    )


def test_guarded_press_plan_only_sends_no_commands():
    reader = Reader()
    result = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None).run(
        press_plan(),
        arm="right",
        operator_lease_id="lease",
        execute=False,
        authorization_guard=None,
    )

    assert result["status"] == "planned"
    assert result["plan_summary"]["contact_travel_m"] == pytest.approx(0.005)
    assert reader.interfaces["right"].calls == []


def test_guarded_press_rejects_unadmitted_geometry_before_motion():
    reader = Reader()
    result = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None).run(
        press_plan(admitted=False),
        arm="right",
        operator_lease_id="lease",
        execute=True,
        authorization_guard=lambda: None,
    )

    assert result["status"] == "failed"
    assert "geometry is not admitted" in result["failure_reasons"][0]
    assert reader.interfaces["right"].calls == []


def test_guarded_press_rejects_open_gripper_before_motion():
    reader = Reader()
    reader.value["arms"]["right"]["gripper_width_m"] = 0.0988
    result = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None).run(
        press_plan(),
        arm="right",
        operator_lease_id="lease",
        execute=True,
        authorization_guard=lambda: None,
    )

    assert result["status"] == "failed"
    assert "requires gripper width" in result["failure_reasons"][0]
    assert reader.interfaces["right"].calls == []


def test_effort_contact_holds_then_retreats_to_measured_origin():
    reader = Reader()
    reader.freeze_at_j1 = 0.004
    reader.contact_after_snapshots = 8
    executor = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None)
    result = executor.run(
        press_plan(),
        arm="right",
        operator_lease_id="lease",
        execute=True,
        authorization_guard=lambda: None,
        prior_stage_receipt=contact_stage_receipt(),
    )

    assert result["status"] == "passed"
    assert result["contact_event"]["outcome"] == "contact_effort"
    assert result["contact_hold"]
    assert result["retreat"]["status"] == "passed"
    assert result["retreat_attempted"] is True


def test_guarded_press_requires_previous_no_contact_stage_before_motion():
    reader = Reader()
    result = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None).run(
        press_plan(),
        arm="right",
        operator_lease_id="lease",
        execute=True,
        authorization_guard=lambda: None,
    )

    assert result["status"] == "failed"
    assert "previous passed no-contact stage" in result["failure_reasons"][0]
    assert reader.interfaces["right"].calls == []


def test_no_contact_plan_only_sends_no_commands():
    reader = Reader()
    result = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None).run_no_contact(
        no_contact_plan(),
        arm="right",
        approach_distance_m=0.06,
        max_waypoints=2,
        operator_lease_id="lease",
        execute=False,
        authorization_guard=None,
    )
    assert result["status"] == "planned"
    assert reader.interfaces["right"].calls == []


def test_no_contact_prefix_reverses_to_start_and_records_effort():
    reader = Reader()
    result = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None).run_no_contact(
        no_contact_plan(),
        arm="right",
        approach_distance_m=0.06,
        max_waypoints=2,
        operator_lease_id="lease",
        execute=True,
        authorization_guard=lambda: None,
    )
    assert result["status"] == "passed"
    assert result["contact_intended"] is False
    assert result["retreat_attempted"] is True
    assert result["final_start_error_rad"] < 1e-4
    assert result["dynamic_effort"]["statistics"]


def test_no_contact_rejects_unadmitted_geometry_before_motion():
    reader = Reader()
    result = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None).run_no_contact(
        no_contact_plan(admitted=False),
        arm="right",
        approach_distance_m=0.06,
        max_waypoints=1,
        operator_lease_id="lease",
        execute=True,
        authorization_guard=lambda: None,
    )
    assert result["status"] == "failed"
    assert result["motion_command_attempted"] is False


def test_closer_no_contact_stage_requires_previous_full_stage_receipt():
    reader = Reader()
    executor = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None)
    missing = executor.run_no_contact(
        staged_no_contact_plan(),
        arm="right",
        approach_distance_m=0.02,
        max_waypoints=3,
        operator_lease_id="lease",
        execute=False,
        authorization_guard=None,
    )
    accepted = executor.run_no_contact(
        staged_no_contact_plan(),
        arm="right",
        approach_distance_m=0.02,
        max_waypoints=3,
        operator_lease_id="lease",
        execute=False,
        authorization_guard=None,
        prior_stage_receipt=prior_stage_receipt(),
    )

    assert missing["status"] == "failed"
    assert "previous passed stage receipt" in missing["failure_reasons"][0]
    assert accepted["status"] == "planned"
    assert accepted["plan_summary"]["waypoints"] == 3


def test_plan_start_recovery_returns_in_one_validated_step():
    reader = Reader()
    reader.value["arms"]["right"]["joints_rad"][0] = 0.02
    result = PiperGuardedPressExecutor(reader, settings(), sleeper=lambda _: None).return_to_plan_start(
        no_contact_plan(),
        arm="right",
        operator_lease_id="lease",
        execute=True,
        authorization_guard=lambda: None,
    )
    assert result["status"] == "passed"
    assert result["tracking_error_rad"] < 1e-4
    assert reader.value["arms"]["right"]["joints_rad"][0] == pytest.approx(0.0)
