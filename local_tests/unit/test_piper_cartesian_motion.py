from copy import deepcopy
import math
from types import SimpleNamespace

import pytest

from services.piper.cartesian_motion import (
    CartesianMotionSettings,
    PiperCartesianMotionController,
)
from services.piper.trajectory import PiperJointTrajectoryExecutor, TrajectorySettings


def backend_options():
    return {
        "feedback_max_age_s": 0.25,
        "joint_limit_margin_rad": 0.02,
        "max_joint_step_rad": 0.25,
        "link_min_z_m": 0.1,
        "workspace_bounds": {
            "right": {"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]}
        },
    }


def settings():
    return CartesianMotionSettings.from_options(
        {
            "enabled": True,
            "allowed_arms": ["right"],
            "max_translation_m": 0.25,
            "max_rotation_rad": 0.75,
            "max_waypoints": 16,
            "max_executions_per_arming": 3,
        },
        backend_options=backend_options(),
        arms={"left": {}, "right": {}},
    )


def pose_plan(*, target_x=0.1):
    return {
        "kind": "piper_readonly_pose_plan",
        "status": "passed",
        "planning_accepted": True,
        "checks": {"workspace_bounds": True, "distal_link_min_z": True},
        "start": {"position_m": [0.0, 0.0, 0.2], "rpy_rad": [0.0, 0.0, 0.0]},
        "target": {
            "position_m": [target_x, 0.0, 0.2],
            "rpy_rad": [0.0, 0.0, 0.0],
        },
        "start_joints_rad": [0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
        "waypoints": [
            {"joints_rad": [0.01, 0.2, -0.2, 0.0, 0.0, 0.0]},
            {"joints_rad": [0.02, 0.2, -0.2, 0.0, 0.0, 0.0]},
        ],
    }


class FakeExecutor:
    def __init__(self):
        self.calls = []

    def execute(self, *, arm, start, waypoints, authorization_guard):
        authorization_guard()
        self.calls.append(("execute", arm, start, waypoints))
        return {
            "status": "passed",
            "motion_command_attempted": True,
            "control_certain": True,
            "reached_path": [start, *waypoints],
            "final_joints_rad": waypoints[-1],
            "final_hold": {"target_joints_rad": waypoints[-1]},
        }

    def retreat(self, *, arm, path, authorization_guard):
        authorization_guard()
        self.calls.append(("retreat", arm, path))
        return {
            "status": "passed",
            "motion_command_attempted": True,
            "control_certain": True,
            "final_joints_rad": path[0],
            "final_hold": {"target_joints_rad": path[0]},
        }

    def hold(self, *, arm):
        self.calls.append(("hold", arm))
        return {
            "status": "passed",
            "arm": arm,
            "target_joints_rad": [0.0] * 6,
        }


def armed(_root):
    return {"armed": True, "since": 100.0, "until": 200.0, "by": "operator"}


def test_settings_require_workspace_for_every_enabled_arm():
    with pytest.raises(ValueError, match="workspace bounds"):
        CartesianMotionSettings.from_options(
            {"enabled": True, "allowed_arms": ["left", "right"]},
            backend_options=backend_options(),
            arms={"left": {}, "right": {}},
        )


def test_settings_allow_explicit_operator_supervised_unchecked_workspace():
    value = backend_options()
    value["workspace_bounds"] = None
    value["link_min_z_m"] = None
    result = CartesianMotionSettings.from_options(
        {
            "enabled": True,
            "allowed_arms": ["right"],
            "require_workspace_checks": False,
        },
        backend_options=value,
        arms={"right": {}},
    )
    assert result.require_workspace_checks is False


def test_controller_stages_executes_and_reverses_one_plan(tmp_path):
    executor = FakeExecutor()
    controller = PiperCartesianMotionController(
        tmp_path,
        SimpleNamespace(),
        settings(),
        executor=executor,
        clock=lambda: 110.0,
        arming_reader=armed,
    )

    staged = controller.stage(pose_plan(), arm="right")
    assert staged["execution_admissible"] is True
    assert (tmp_path / "piper-cartesian-plans" / f"{staged['plan_id']}.json").is_file()

    executed = controller.execute(staged["plan_id"])
    assert executed["executed"] is True
    assert executed["retreat_available"] is True
    assert executed["budget"]["executions_used"] == 1

    retreated = controller.retreat(staged["plan_id"])
    assert retreated["retreated"] is True
    assert [call[0] for call in executor.calls] == ["execute", "retreat"]
    assert controller.status()["plans"][0]["state"] == "retreated"


def test_controller_discards_only_an_unexecuted_plan(tmp_path):
    controller = PiperCartesianMotionController(
        tmp_path,
        SimpleNamespace(),
        settings(),
        executor=FakeExecutor(),
        clock=lambda: 110.0,
        arming_reader=armed,
    )
    staged = controller.stage(pose_plan(), arm="right")
    discarded = controller.discard(staged["plan_id"])
    assert discarded["discarded"] is True
    assert discarded["motion_command_attempted"] is False
    assert controller.plans[staged["plan_id"]]["state"] == "discarded"

    staged = controller.stage(pose_plan(), arm="right")
    controller.execute(staged["plan_id"], waypoint_count=1)
    with pytest.raises(PermissionError, match="unexecuted"):
        controller.discard(staged["plan_id"])


def test_agent_can_advance_and_retreat_a_selected_number_of_waypoints(tmp_path):
    executor = FakeExecutor()
    controller = PiperCartesianMotionController(
        tmp_path,
        SimpleNamespace(),
        settings(),
        executor=executor,
        clock=lambda: 110.0,
        arming_reader=armed,
    )
    staged = controller.stage(pose_plan(), arm="right")

    first = controller.execute(staged["plan_id"], waypoint_count=1)
    assert first["progress_waypoints"] == 1
    assert first["continue_available"] is True
    assert first["retreat_available"] is True
    assert controller.status()["plans"][0]["state"] == "partially_executed"

    controller.execute(staged["plan_id"], waypoint_count=1)
    back = controller.retreat(staged["plan_id"], waypoint_count=1)
    assert back["progress_waypoints"] == 1
    assert back["continue_available"] is True
    assert back["retreat_available"] is True

    finished = controller.execute(staged["plan_id"], waypoint_count=1)
    assert finished["progress_waypoints"] == 2
    assert finished["continue_available"] is False

    held = controller.hold("right")
    assert held["status"] == "passed"
    assert [call[0] for call in executor.calls] == [
        "execute",
        "execute",
        "retreat",
        "execute",
        "hold",
    ]


def test_segment_validation_rejects_more_waypoints_than_available(tmp_path):
    controller = PiperCartesianMotionController(
        tmp_path,
        SimpleNamespace(),
        settings(),
        executor=FakeExecutor(),
        clock=lambda: 110.0,
        arming_reader=armed,
    )
    staged = controller.stage(pose_plan(), arm="right")
    with pytest.raises(ValueError, match="only 2 are available"):
        controller.validate_execute(staged["plan_id"], 3)
    with pytest.raises(PermissionError, match="state is planned"):
        controller.validate_retreat(staged["plan_id"], 1)


def test_controller_refuses_oversized_translation_and_parallel_plan(tmp_path):
    controller = PiperCartesianMotionController(
        tmp_path,
        SimpleNamespace(),
        settings(),
        executor=FakeExecutor(),
        clock=lambda: 110.0,
        arming_reader=armed,
    )
    with pytest.raises(ValueError, match="translation"):
        controller.stage(pose_plan(target_x=0.3), arm="right")

    controller.stage(pose_plan(), arm="right")
    with pytest.raises(PermissionError, match="current Cartesian plan"):
        controller.stage(pose_plan(), arm="right")


def test_new_live_plan_can_continue_from_an_executed_endpoint(tmp_path):
    controller = PiperCartesianMotionController(
        tmp_path,
        SimpleNamespace(),
        settings(),
        executor=FakeExecutor(),
        clock=lambda: 110.0,
        arming_reader=armed,
    )
    first = controller.stage(pose_plan(), arm="right")
    controller.execute(first["plan_id"])

    continued = pose_plan(target_x=0.15)
    continued["start"]["position_m"] = [0.1, 0.0, 0.2]
    continued["start_joints_rad"] = [0.02, 0.2, -0.2, 0.0, 0.0, 0.0]
    continued["waypoints"] = [
        {"joints_rad": [0.03, 0.2, -0.2, 0.0, 0.0, 0.0]}
    ]
    second = controller.stage(continued, arm="right")

    states = {
        plan["plan_id"]: plan["state"] for plan in controller.status()["plans"]
    }
    assert states[first["plan_id"]] == "superseded"
    assert states[second["plan_id"]] == "planned"
    with pytest.raises(PermissionError, match="superseded"):
        controller.validate_retreat(first["plan_id"])


def arm_state(joints):
    return {
        "joints_rad": list(joints),
        "feedback": {"joint_age_s": 0.01, "status_age_s": 0.01},
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
    def __init__(self):
        start = [0.0, 0.2, -0.2, 0.0, 0.0, 0.0]
        self.value = {"arms": {"left": arm_state(start), "right": arm_state(start)}}
        self.interfaces = {side: Interface(self, side) for side in ("left", "right")}

    def snapshot(self):
        return deepcopy(self.value)


def test_generic_trajectory_executes_and_retreats_measured_path():
    reader = Reader()
    executor = PiperJointTrajectoryExecutor(
        reader,
        TrajectorySettings(
            poll_interval_s=0.001,
            waypoint_timeout_s=0.01,
            settle_tolerance_rad=0.0005,
        ),
        sleeper=lambda _seconds: None,
    )
    start = reader.value["arms"]["right"]["joints_rad"]
    targets = [
        [0.01, 0.2, -0.2, 0.0, 0.0, 0.0],
        [0.02, 0.2, -0.2, 0.0, 0.0, 0.0],
    ]

    executed = executor.execute(
        arm="right",
        start=start,
        waypoints=targets,
        authorization_guard=lambda: None,
    )
    assert executed["status"] == "passed"
    assert len(executed["reached_path"]) == 3

    retreated = executor.retreat(
        arm="right",
        path=executed["reached_path"],
        authorization_guard=lambda: None,
    )
    assert retreated["status"] == "passed"
    assert retreated["final_start_error_rad"] < 0.0005
