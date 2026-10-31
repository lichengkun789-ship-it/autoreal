"""Interrupted ClickBell paths can be retraced without inventing a new path."""

from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from services import arming
from services.piper.click_bell.recovery import ClickBellRecoveryController


START = [0.0, 0.2, -0.2, 0.0, 0.0, 0.0]
PATH = [
    START,
    [0.04, 0.24, -0.22, 0.0, 0.0, 0.0],
    [0.08, 0.28, -0.24, 0.0, 0.0, 0.0],
]


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
    def __init__(self, joints):
        self.value = {
            "arms": {
                "left": arm_state(START),
                "right": arm_state(joints),
            }
        }
        self.interfaces = {
            side: Interface(self, side) for side in ("left", "right")
        }

    def snapshot(self):
        return deepcopy(self.value)


def settings():
    return SimpleNamespace(
        effort_delta_nm=1.0,
        speed_percent=2,
        feedback_max_age_s=0.25,
        waypoint_timeout_s=1.0,
        settle_tolerance_rad=0.01,
        joint_margin_rad=0.02,
        max_joint_step_rad=0.1,
    )


def interrupted_session(sessions: Path, name="previous"):
    root = sessions / name
    plans = root / "click-bell-plans"
    motion = root / "piper-motion"
    plans.mkdir(parents=True)
    motion.mkdir()
    plan_id = "a1b2c3d4e5f6"
    plan = {
        "kind": "click_bell_readonly_press_path",
        "status": "passed",
        "dynamic": True,
        "execution_admissible": True,
        "geometry_admissible": True,
        "arm": "right",
        "plan_id": plan_id,
        "start_joints_rad": PATH[0],
        "stages": [
            {
                "approach_distance_m": 0.005,
                "plan": {
                    "status": "passed",
                    "waypoints": [{"joints_rad": point} for point in PATH[1:]],
                },
            },
            {
                "approach_distance_m": 0.0,
                "plan": {
                    "status": "passed",
                    "waypoints": [{"joints_rad": [0.09, 0.29, -0.245, 0, 0, 0]}],
                },
            },
        ],
    }
    (plans / f"{plan_id}.json").write_text(json.dumps(plan))
    receipt = {
        "kind": "piper_agent_click_bell_guarded_press",
        "status": "failed",
        "arm": "right",
        "motion_command_attempted": True,
        "retreat": {"status": "failed"},
        "admission": {"plan_id": plan_id},
    }
    (motion / "agent_click_bell_press_failed.json").write_text(json.dumps(receipt))
    return root


def test_agent_retraces_matching_interrupted_path_to_observation_pose(tmp_path):
    sessions = tmp_path / "sessions"
    interrupted_session(sessions)
    current = sessions / "current"
    current.mkdir()
    arming.write(current, True, minutes=5, by="operator")
    reader = Reader([0.081, 0.279, -0.241, 0.0, 0.0, 0.0])
    controller = ClickBellRecoveryController(
        current, reader, settings(), sleeper=lambda _: None
    )

    status = controller.status()
    result = controller.execute()

    assert status["available"] is True
    assert status["candidate"]["anchor_index"] == 2
    assert result["status"] == "passed"
    assert result["reverse_waypoints"] == 2
    assert reader.value["arms"]["right"]["joints_rad"] == pytest.approx(
        START, abs=2e-5
    )
    receipt = json.loads(Path(result["receipt"]).read_text())
    assert receipt["kind"] == "piper_agent_click_bell_recovery"
    assert receipt["source_plan_id"] == "a1b2c3d4e5f6"
    with pytest.raises(PermissionError, match="budget is exhausted"):
        controller.validate_request()


def test_recovery_rejects_live_pose_that_does_not_match_interrupted_path(tmp_path):
    sessions = tmp_path / "sessions"
    interrupted_session(sessions)
    current = sessions / "current"
    current.mkdir()
    arming.write(current, True, minutes=5, by="operator")
    controller = ClickBellRecoveryController(
        current, Reader([0.5, 0.5, -0.5, 0.0, 0.0, 0.0]), settings()
    )

    with pytest.raises(PermissionError, match="no failed admitted"):
        controller.validate_request()


def test_agent_can_recover_a_failed_path_from_the_current_session(tmp_path):
    sessions = tmp_path / "sessions"
    current = interrupted_session(sessions, "current")
    arming.write(current, True, minutes=5, by="operator")
    reader = Reader([0.081, 0.279, -0.241, 0.0, 0.0, 0.0])
    controller = ClickBellRecoveryController(
        current, reader, settings(), sleeper=lambda _: None
    )

    status = controller.status()
    result = controller.execute()

    assert status["available"] is True
    assert status["candidate"]["source_scope"] == "current_session"
    assert result["source_scope"] == "current_session"
    assert result["status"] == "passed"
    assert reader.value["arms"]["right"]["joints_rad"] == pytest.approx(
        START, abs=2e-5
    )
