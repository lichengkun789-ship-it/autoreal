"""Piper MCP adapters for the ClickBell task."""

from pathlib import Path

from services.piper.click_bell.gripper import (
    ClickBellGripperController,
    ClickBellGripperSettings,
)
from services.piper.click_bell.press import (
    ClickBellPressController,
    ClickBellPressSettings,
)
from services.piper.episode_recording import (
    EpisodeRecordingSettings,
    TaskEpisodeRecorder,
)

from tasks.click_bell.outcome import ClickBellOutcomeController
from tasks.click_bell.perception import localize_saved_frame

from .dynamic import ClickBellDynamicPlanner
from .recovery import ClickBellRecoveryController


def _schema(properties=None, required=()):
    return {
        "type": "object",
        "properties": properties or {},
        "required": list(required),
        "additionalProperties": False,
    }


class ClickBellTools:
    @classmethod
    def describe_options(cls, task_options):
        if not isinstance(task_options, dict):
            raise ValueError("click_bell task_options must be an object")
        unknown = set(task_options) - {"guarded_press", "gripper", "recording"}
        if unknown:
            raise ValueError(f"unknown click_bell task options: {sorted(unknown)}")
        names = ["click_bell_localize_press_cap"]
        motion = []
        press = task_options.get("guarded_press")
        gripper = task_options.get("gripper")
        if isinstance(press, dict) and press.get("enabled") is True:
            motion.extend(
                ["click_bell_recover_observation_pose", "click_bell_guarded_press"]
            )
            names.extend(["click_bell_recover_observation_pose", "click_bell_plan_press"])
            if isinstance(gripper, dict) and gripper.get("enabled") is True:
                names.append("click_bell_set_gripper")
                motion.append("click_bell_set_gripper")
            names.extend(
                ["click_bell_guarded_press", "click_bell_confirm_visual_result"]
            )
        return {"tools": tuple(names), "motion_tools": tuple(motion)}

    def __init__(self, config, *, backend=None):
        if config.robot != "piper" or config.observation_profile != "rgbd":
            raise ValueError("click_bell task tools require a Piper RGB-D session")
        cameras = config.backend_options.get("cameras") or []
        self.camera_names = tuple(
            camera["name"]
            for camera in cameras
            if isinstance(camera, dict)
            and camera.get("name")
            and camera.get("mount")
            and camera.get("extrinsics")
        )
        if not self.camera_names:
            raise ValueError("click_bell task tools require at least one calibrated camera")
        self.frames_root = Path(config.root) / "agent-workspace" / "runtime" / "frames"
        task_options = getattr(config, "task_options", {}) or {}
        if not isinstance(task_options, dict):
            raise ValueError("click_bell task_options must be an object")
        unknown = set(task_options) - {"guarded_press", "gripper", "recording"}
        if unknown:
            raise ValueError(f"unknown click_bell task options: {sorted(unknown)}")
        press_settings = ClickBellPressSettings.from_options(
            task_options.get("guarded_press"), backend_options=config.backend_options
        )
        gripper_settings = ClickBellGripperSettings.from_options(
            task_options.get("gripper"), backend_options=config.backend_options
        )
        if gripper_settings is not None and press_settings is None:
            raise ValueError("click_bell gripper requires guarded_press to be enabled")
        recording = EpisodeRecordingSettings.from_options(
            task_options.get("recording"),
            required=press_settings is not None or gripper_settings is not None,
        )
        if press_settings is not None or gripper_settings is not None:
            if backend is None or not hasattr(backend, "reader"):
                raise ValueError("click_bell motion requires the Piper backend reader")
        self.press = None
        self.dynamic_plan = None
        self.recovery = None
        self.gripper = None
        self.outcome = None
        self.task_recorder = None
        mutating = set()
        if press_settings is not None:
            if getattr(backend, "cameras", None) is None:
                raise ValueError("click_bell guarded press recording requires live cameras")
            if gripper_settings is not None:
                self.task_recorder = TaskEpisodeRecorder(
                    config.root,
                    backend.cameras,
                    self.camera_names,
                    recording,
                    camera_timeout_ms=int(
                        config.backend_options.get("camera_timeout_ms", 3000)
                    ),
                )
            self.press = ClickBellPressController(
                config.root,
                backend.reader,
                backend.cameras,
                self.camera_names,
                press_settings,
                recording,
                camera_timeout_ms=int(config.backend_options.get("camera_timeout_ms", 3000)),
                task_recorder=self.task_recorder,
            )
            self.dynamic_plan = ClickBellDynamicPlanner(
                config.root,
                self.frames_root,
                backend.reader,
                press_settings,
                config.backend_options,
            )
            self.recovery = ClickBellRecoveryController(
                config.root,
                backend.reader,
                press_settings,
            )
            mutating.add("click_bell_guarded_press")
            mutating.add("click_bell_recover_observation_pose")
        if gripper_settings is not None:
            if getattr(backend, "cameras", None) is None:
                raise ValueError("click_bell gripper recording requires live cameras")
            self.gripper = ClickBellGripperController(
                config.root,
                backend.reader,
                backend.cameras,
                self.camera_names,
                gripper_settings,
                recording,
                self.press,
                camera_timeout_ms=int(config.backend_options.get("camera_timeout_ms", 3000)),
                task_recorder=self.task_recorder,
            )
            mutating.add("click_bell_set_gripper")
        if self.press is not None:
            self.outcome = ClickBellOutcomeController(
                config.root, self.press, gripper_controller=self.gripper
            )
        self.mutating = frozenset(mutating)

    def tools(self):
        tools = [
            {
                "name": "click_bell_localize_press_cap",
                "description": (
                    "Find the yellow press cap above the blue bell in one saved Piper RGB-D "
                    "frame, then fit its interior depth points and return the cap surface "
                    "center, normal, fit quality, and 3-D position in the camera's Piper "
                    "base frame. Moves nothing."
                ),
                "inputSchema": _schema(
                    {
                        "frame_id": {"type": "string"},
                        "camera": {
                            "type": "string",
                            "enum": list(self.camera_names),
                        },
                        "radius": {
                            "type": "integer",
                            "minimum": 0,
                            "maximum": 15,
                        },
                    },
                    ("frame_id", "camera"),
                ),
            }
        ]
        if self.dynamic_plan is not None:
            tools.append(
                {
                    "name": "click_bell_recover_observation_pose",
                    "description": (
                        "Recover the right Piper arm when a ClickBell trajectory cannot "
                        "continue. Finds the newest failed admitted path, including one from "
                        "this session, verifies it against live joints, retraces only that path "
                        "to its observation start, and writes a receipt. Use after a partial "
                        "failed approach or when the right wrist is too close or occluded. "
                        "Requires a live operator arming lease and moves the robot at the "
                        "commissioned speed."
                    ),
                    "inputSchema": _schema(),
                }
            )
            tools.append(
                {
                    "name": "click_bell_plan_press",
                    "description": (
                        "Localize the yellow ClickBell press cap in one fresh saved RGB-D "
                        "frame and compute a collision-gated Piper IK press path from the "
                        "live joint state. Moves nothing. Returns a short-lived one-shot "
                        "plan_id bound to the current operator arming lease."
                    ),
                    "inputSchema": _schema(
                        {
                            "frame_id": {"type": "string"},
                            "camera": {
                                "type": "string",
                                "enum": list(self.camera_names),
                            },
                            "radius": {
                                "type": "integer",
                                "minimum": 0,
                                "maximum": 15,
                            },
                        },
                        ("frame_id", "camera"),
                    ),
                }
            )
        if self.gripper is not None:
            tools.append(
                {
                    "name": "click_bell_set_gripper",
                    "description": (
                        "Execute one commissioned right-gripper action for ClickBell. Only close "
                        "then open are admitted; open is rejected until the guarded press has "
                        "retreated. Width, step, effort, and call budgets are fixed. Each action "
                        "participates in the one continuous task recording."
                    ),
                    "inputSchema": _schema(
                        {"action": {"type": "string", "enum": ["close", "open"]}},
                        ("action",),
                    ),
                }
            )
        if self.press is not None:
            tools.extend(
                [
                    {
                        "name": "click_bell_guarded_press",
                        "description": (
                            "Consume one fresh click_bell_plan_press plan, execute one direct "
                            "guarded Piper ClickBell approach and 5 mm press, then reverse "
                            "retreat. Records the "
                            "complete motion. Requires a live operator arming lease; success "
                            "remains pending until the human visually confirms it."
                        ),
                        "inputSchema": _schema(
                            {
                                "plan_id": {
                                    "type": "string",
                                    "pattern": "^[0-9a-f]{12}$",
                                }
                            },
                            ("plan_id",),
                        ),
                    },
                    {
                        "name": "click_bell_confirm_visual_result",
                        "description": (
                            "Bind the latest guarded press to an answered request_operator call of "
                            "kind visual_confirmation. The operator reply must begin with success, "
                            "failure, or unclear. This moves nothing and cannot invent a result."
                        ),
                        "inputSchema": _schema(
                            {
                                "press_id": {
                                    "type": "string",
                                    "pattern": "^[0-9a-f]{12}$",
                                },
                                "request_id": {
                                    "type": "string",
                                    "pattern": "^[0-9a-f]{12}$",
                                },
                            },
                            ("press_id", "request_id"),
                        ),
                    },
                ]
            )
        return tools

    def _frame_directory(self, frame_id):
        if not isinstance(frame_id, str) or not frame_id.isalnum():
            raise ValueError("invalid frame_id")
        directory = self.frames_root / frame_id
        if not (directory / "meta.json").is_file():
            raise ValueError(f"unknown frame_id {frame_id}")
        return directory

    def validate(self, name, arguments):
        if name == "click_bell_recover_observation_pose":
            if self.recovery is None:
                raise PermissionError("ClickBell observation recovery is not enabled")
            if arguments:
                raise ValueError(
                    "click_bell_recover_observation_pose takes no arguments"
                )
            self.recovery.validate_request()
            return
        if name == "click_bell_guarded_press":
            if self.press is None:
                raise PermissionError("click_bell guarded press is not enabled")
            if set(arguments) != {"plan_id"}:
                raise ValueError("click_bell_guarded_press requires plan_id")
            if self.gripper is not None:
                self.gripper.require_closed_for_press()
            self.dynamic_plan.validate(arguments["plan_id"])
            self.press.validate_request()
            return
        if name == "click_bell_plan_press":
            if self.dynamic_plan is None:
                raise PermissionError("dynamic ClickBell planning is not enabled")
            self._validate_localization_arguments(arguments, name)
            return
        if name == "click_bell_set_gripper":
            if self.gripper is None:
                raise PermissionError("click_bell gripper is not enabled")
            if set(arguments) != {"action"}:
                raise ValueError("click_bell_set_gripper requires action")
            self.gripper.validate_request(arguments["action"])
            return
        if name == "click_bell_confirm_visual_result":
            if self.outcome is None:
                raise PermissionError("click_bell visual confirmation is not enabled")
            if set(arguments) != {"press_id", "request_id"}:
                raise ValueError(
                    "click_bell_confirm_visual_result requires press_id and request_id"
                )
            self.outcome.validate_request(
                arguments["press_id"], arguments["request_id"]
            )
            return
        if name != "click_bell_localize_press_cap":
            raise ValueError(f"unknown click_bell tool {name}")
        self._validate_localization_arguments(arguments, name)

    def _validate_localization_arguments(self, arguments, name):
        if set(arguments) - {"frame_id", "camera", "radius"} or not {
            "frame_id",
            "camera",
        }.issubset(arguments):
            raise ValueError(f"{name} requires frame_id and camera")
        if arguments["camera"] not in self.camera_names:
            raise ValueError("camera is not calibrated for click_bell localization")
        radius = arguments.get("radius", 3)
        if type(radius) is not int or not 0 <= radius <= 15:
            raise ValueError("radius must be an integer from 0 to 15")
        self._frame_directory(arguments["frame_id"])

    def call(self, name, arguments):
        self.validate(name, arguments)
        if name == "click_bell_recover_observation_pose":
            return self.recovery.execute(), []
        if name == "click_bell_plan_press":
            return self.dynamic_plan.plan(
                arguments["frame_id"],
                arguments["camera"],
                radius=arguments.get("radius", 3),
            ), []
        if name == "click_bell_guarded_press":
            plan = self.dynamic_plan.consume(arguments["plan_id"])
            return self.press.execute(plan), []
        if name == "click_bell_set_gripper":
            return self.gripper.execute(arguments["action"]), []
        if name == "click_bell_confirm_visual_result":
            return self.outcome.confirm(
                arguments["press_id"], arguments["request_id"]
            ), []
        result = localize_saved_frame(
            self._frame_directory(arguments["frame_id"]),
            arguments["camera"],
            radius=arguments.get("radius", 3),
        )
        return result, []

    def status(self):
        return {
            "click_bell_guarded_press": None
            if self.press is None
            else self.press.status(),
            "click_bell_dynamic_plan": None
            if self.dynamic_plan is None
            else self.dynamic_plan.status(),
            "click_bell_observation_recovery": None
            if self.recovery is None
            else self.recovery.status(),
            "click_bell_gripper": None
            if self.gripper is None
            else self.gripper.status(),
            "click_bell_visual_result": None
            if self.outcome is None
            else self.outcome.status(),
            "click_bell_task_recording": None
            if self.task_recorder is None
            else {
                **self.task_recorder.reference(),
                "events": len(self.task_recorder.events),
            },
        }

    def close(self):
        if self.task_recorder is not None and self.task_recorder.state == "recording":
            self.task_recorder.abort("task_toolset_closed_before_open")
