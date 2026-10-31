"""RAX backend for a dual-arm Piper system.

Observation and target validation are always available. Operator configuration may
independently expose characterized single-joint motion or lease-bound Cartesian plans.
"""

from __future__ import annotations

import base64
from datetime import datetime, timezone
import io
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np
from PIL import Image

from .agent_motion import AgentMotionSettings, PiperAgentMotionController
from .calibration import camera_models, deproject_pixel, transform_point
from .camera import RealSenseRig, camera_specs
from .cartesian_motion import (
    CartesianMotionSettings,
    PiperCartesianMotionController,
)
from .ik import plan_pose
from .kinematics import end_transform, pose_report, tool_pose_report
from .gripper_motion import GripperMotionSettings, PiperGripperMotionController
from .episode_recording import EpisodeRecordingSettings, SessionEpisodeRecorder
from .safety import DEFAULT_MAX_JOINT_STEP_RAD, validate_joint_target
from .sdk_reader import PiperSdkReader


DEFAULTS = {
    "arms": None,
    "feedback_max_age_s": 0.25,
    "feedback_startup_timeout_s": 2.0,
    "joint_limit_margin_rad": 0.02,
    "max_joint_step_rad": DEFAULT_MAX_JOINT_STEP_RAD,
    "workspace_bounds": None,
    "link_min_z_m": None,
    "tool_offset_tcp_m": [0.0, 0.0, 0.0],
    "agent_motion": None,
    "cartesian_motion": None,
    "gripper_motion": None,
    "episode_recording": None,
    "agent_python": None,
    "cameras": None,
    "camera_width": 640,
    "camera_height": 480,
    "camera_fps": 30,
    "camera_warmup_frames": 15,
    "camera_timeout_ms": 3000,
    "agent_image_width": 640,
    "agent_jpeg_quality": 85,
    "protected_paths": [],
}


def _schema(properties=None, required=()):
    return {
        "type": "object",
        "properties": properties or {},
        "required": list(required),
        "additionalProperties": False,
    }


class PiperBackend:
    """RAX backend contract backed by Piper SDK state and explicit motion admission."""

    def __init__(
        self,
        config,
        *,
        reader=None,
        cameras=None,
        cartesian_controller=None,
        gripper_controller=None,
        clock=time.time,
    ):
        unknown = set(config.backend_options) - set(DEFAULTS)
        if unknown:
            raise ValueError(f"unknown piper backend options: {sorted(unknown)}")
        self.config = config
        self.options = {**DEFAULTS, **config.backend_options}
        if float(self.options["feedback_max_age_s"]) <= 0:
            raise ValueError("feedback_max_age_s must be positive")
        if float(self.options["feedback_startup_timeout_s"]) <= 0:
            raise ValueError("feedback_startup_timeout_s must be positive")
        if float(self.options["joint_limit_margin_rad"]) < 0:
            raise ValueError("joint_limit_margin_rad must be non-negative")
        if float(self.options["max_joint_step_rad"]) <= 0:
            raise ValueError("max_joint_step_rad must be positive")
        self.workspace_bounds = self._workspace_bounds(self.options["workspace_bounds"])
        link_min_z = self.options["link_min_z_m"]
        if link_min_z is not None and not math.isfinite(float(link_min_z)):
            raise ValueError("link_min_z_m must be finite or null")
        tool_offset = np.asarray(self.options["tool_offset_tcp_m"], dtype=np.float64)
        if tool_offset.shape != (3,) or not np.all(np.isfinite(tool_offset)):
            raise ValueError("tool_offset_tcp_m must be a finite 3-D vector")
        self.tool_offset_tcp_m = tool_offset.tolist()
        for name in (
            "camera_width",
            "camera_height",
            "camera_fps",
            "camera_timeout_ms",
            "agent_image_width",
        ):
            if type(self.options[name]) is not int or self.options[name] <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if (
            type(self.options["camera_warmup_frames"]) is not int
            or self.options["camera_warmup_frames"] < 0
        ):
            raise ValueError("camera_warmup_frames must be a non-negative integer")
        quality = self.options["agent_jpeg_quality"]
        if type(quality) is not int or not 1 <= quality <= 100:
            raise ValueError("agent_jpeg_quality must be an integer from 1 to 100")
        self.clock = clock
        self.observation_profile = getattr(config, "observation_profile", "state-only")
        if self.observation_profile not in {"state-only", "rgb-only", "rgbd"}:
            raise ValueError("unsupported Piper observation profile")
        arms = self.options["arms"]
        if not isinstance(arms, dict):
            motion_options = self.options["agent_motion"] or {}
            cartesian_options = self.options["cartesian_motion"] or {}
            gripper_options = self.options["gripper_motion"] or {}
            if reader is None or (
                isinstance(motion_options, dict) and motion_options.get("enabled") is True
            ) or (
                isinstance(cartesian_options, dict)
                and cartesian_options.get("enabled") is True
            ) or (
                isinstance(gripper_options, dict)
                and gripper_options.get("enabled") is True
            ):
                raise ValueError(
                    "piper backend option 'arms' is required; see examples/piper.json"
                )
            arms = {}
        self.motion_settings = AgentMotionSettings.from_options(
            self.options["agent_motion"], arms=arms
        )
        self.cartesian_settings = CartesianMotionSettings.from_options(
            self.options["cartesian_motion"],
            backend_options=self.options,
            arms=arms,
        )
        self.gripper_settings = GripperMotionSettings.from_options(
            self.options["gripper_motion"], backend_options=self.options
        )
        self.episode_recording_settings = EpisodeRecordingSettings.from_options(
            self.options["episode_recording"]
        )
        if reader is None:
            reader = PiperSdkReader(arms, clock=clock)
        self.reader = reader

        self.camera_names: tuple[str, ...] = ()
        self.camera_models = {}
        self.cameras = None
        self.frames_root: Path | None = None
        if self.observation_profile != "state-only":
            specs = camera_specs(self.options["cameras"])
            root = getattr(config, "root", None)
            if root is None:
                raise ValueError("camera observation requires a session root")
            self.camera_names = tuple(spec.name for spec in specs)
            self.camera_models = camera_models(specs)
            self.frames_root = Path(root) / "agent-workspace" / "runtime" / "frames"
            self.cameras = cameras or RealSenseRig(
                specs,
                width=self.options["camera_width"],
                height=self.options["camera_height"],
                fps=self.options["camera_fps"],
                depth=self.observation_profile == "rgbd",
                warmup_frames=self.options["camera_warmup_frames"],
                clock=clock,
            )
        elif cameras is not None:
            raise ValueError("injected cameras require the rgb-only or rgbd observation profile")

        root = getattr(config, "root", None)
        if self.motion_settings.enabled:
            if root is None:
                raise ValueError("enabled agent motion requires a session root")
            self.agent_motion = PiperAgentMotionController(
                root, reader, self.motion_settings
            )
        else:
            self.agent_motion = None
        if self.cartesian_settings.enabled:
            if root is None:
                raise ValueError("enabled Cartesian motion requires a session root")
            self.cartesian_motion = cartesian_controller or PiperCartesianMotionController(
                root,
                reader,
                self.cartesian_settings,
            )
        else:
            self.cartesian_motion = None
        if self.gripper_settings.enabled:
            if root is None:
                raise ValueError("enabled gripper motion requires a session root")
            self.gripper_motion = gripper_controller or PiperGripperMotionController(
                root, reader, self.gripper_settings
            )
        else:
            self.gripper_motion = None
        mutating = set()
        if self.agent_motion is not None:
            mutating.add("piper_move_joint")
        if self.cartesian_motion is not None:
            mutating.update({"piper_execute_plan", "piper_retreat_plan", "piper_hold"})
        if self.gripper_motion is not None:
            mutating.add("piper_gripper")
        self.mutating = frozenset(mutating)
        self.episode_recorder = None
        if self.episode_recording_settings is not None:
            if root is None:
                raise ValueError("episode recording requires a session root")
            if self.cameras is None or not self.camera_names:
                raise ValueError("episode recording requires a camera observation profile")
            self.episode_recorder = SessionEpisodeRecorder(
                root,
                self.reader,
                self.cameras,
                self.camera_names,
                self.episode_recording_settings,
                camera_timeout_ms=int(self.options["camera_timeout_ms"]),
            )
            self.episode_recorder.start()

    def tools(self):
        arm = {"type": "string", "enum": ["left", "right"]}
        positions = {
            "type": "array",
            "items": {"type": "number"},
            "minItems": 6,
            "maxItems": 6,
            "description": "Six absolute joint targets in radians.",
        }
        if self.cameras is None:
            observe_description = (
                "Capture a fresh robot-state observation. This state-only session returns no "
                "camera images and moves nothing."
            )
            observe_schema = _schema()
        else:
            depth_text = (
                " and aligned uint16-millimetre depth"
                if self.observation_profile == "rgbd"
                else ""
            )
            observe_description = (
                f"Capture fresh RGB{depth_text} from the configured Piper cameras plus measured "
                "arm state. Full frames and metadata are saved under runtime/frames/<frame_id>/. "
                "Moves nothing."
            )
            observe_schema = _schema(
                {
                    "cameras": {
                        "type": "array",
                        "items": {"type": "string", "enum": list(self.camera_names)},
                        "minItems": 1,
                        "uniqueItems": True,
                        "description": "Optional camera subset; omit to capture every configured camera.",
                    }
                }
            )
        tools = [
            {
                "name": "piper_status",
                "description": (
                    "Fresh measured state for both Piper arms, including joint angles, "
                    "gripper width, feedback age, controller status, and readiness. Moves nothing."
                ),
                "inputSchema": _schema(),
            },
            {
                "name": "piper_observe",
                "description": observe_description,
                "inputSchema": observe_schema,
            },
            {
                "name": "piper_check_joints",
                "description": (
                    "Validate one arm's six absolute joint targets against measured state, "
                    "joint-limit margins, and the per-call step cap. Moves nothing."
                ),
                "inputSchema": _schema({"arm": arm, "positions": positions}, ("arm", "positions")),
            },
            {
                "name": "piper_plan_pose",
                "description": (
                    "Solve Piper inverse kinematics for the configured tool point and validate "
                    "a straight Cartesian path "
                    "from fresh measured joints. Returns joint waypoints and refusal reasons. "
                    "When generic Cartesian motion is enabled and operator-armed, a passed "
                    "plan also returns a short-lived one-shot plan_id. Planning never moves "
                    "the robot."
                ),
                "inputSchema": _schema(
                    {
                        "arm": arm,
                        "position_m": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": (
                                "Absolute configured-tool-point target in the selected base "
                                "frame, or an XYZ "
                                "offset when relative is true."
                            ),
                        },
                        "rpy_rad": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": (
                                "Optional absolute XYZ roll, pitch, yaw. Omit to preserve "
                                "the measured end-frame orientation."
                            ),
                        },
                        "relative": {"type": "boolean"},
                        "waypoints": {
                            "type": "integer",
                            "minimum": 2,
                            "maximum": 64,
                        },
                    },
                    ("arm", "position_m"),
                ),
            },
        ]
        if self.cartesian_motion is not None:
            plan_id = {
                "type": "string",
                "pattern": "^[0-9a-f]{12}$",
                "description": "One-shot identifier returned by piper_plan_pose.",
            }
            segment_waypoints = {
                "type": "integer",
                "minimum": 1,
                "maximum": self.cartesian_settings.max_waypoints,
                "description": (
                    "Optional number of path waypoints for this call. Omit to use all "
                    "currently available waypoints. Smaller segments return control to the "
                    "Agent for observation and replanning."
                ),
            }
            tools.extend(
                [
                    {
                        "name": "piper_execute_plan",
                        "description": (
                            "Advance through one or more waypoints of a staged Piper Cartesian "
                            "plan, then hold and return control to the Agent. "
                            "Requires the same live operator arming lease used for planning. "
                            "On interruption it holds and attempts to reverse only reached "
                            "waypoints. Returns a durable motion receipt."
                        ),
                        "inputSchema": _schema(
                            {"plan_id": plan_id, "waypoints": segment_waypoints},
                            ("plan_id",),
                        ),
                    },
                    {
                        "name": "piper_retreat_plan",
                        "description": (
                            "Reverse a caller-selected number of already executed waypoints and "
                            "hold. Omit waypoints to return to the original plan start. The Agent "
                            "may choose the retreat depth from observations and plan progress."
                        ),
                        "inputSchema": _schema(
                            {"plan_id": plan_id, "waypoints": segment_waypoints},
                            ("plan_id",),
                        ),
                    },
                    {
                        "name": "piper_hold",
                        "description": (
                            "Latch one Piper arm at its fresh measured joint pose and return a "
                            "receipt. This does not continue a previous target and is available "
                            "without operator arming."
                        ),
                        "inputSchema": _schema({"arm": arm}, ("arm",)),
                    },
                    {
                        "name": "piper_discard_plan",
                        "description": (
                            "Discard an unexecuted staged Cartesian plan without moving the "
                            "robot. Use this after a planning probe or when fresh evidence "
                            "invalidates a plan."
                        ),
                        "inputSchema": _schema({"plan_id": plan_id}, ("plan_id",)),
                    },
                ]
            )
        if self.observation_profile == "rgbd" and self.camera_models:
            tools.append(
                {
                    "name": "piper_pixel_to_position",
                    "description": (
                        "Convert full-resolution pixels from a saved RGB-D frame to points in the "
                        "selected arm base frame. Uses aligned depth, camera intrinsics, and "
                        "operator-supplied camera extrinsics. Moves nothing."
                    ),
                    "inputSchema": _schema(
                        {
                            "frame_id": {"type": "string"},
                            "camera": {
                                "type": "string",
                                "enum": sorted(self.camera_models),
                            },
                            "pixels": {
                                "type": "array",
                                "minItems": 1,
                                "maxItems": 64,
                                "items": {
                                    "type": "array",
                                    "items": {"type": "number"},
                                    "minItems": 2,
                                    "maxItems": 2,
                                },
                            },
                            "radius": {
                                "type": "integer",
                                "minimum": 0,
                                "maximum": 15,
                            },
                        },
                        ("frame_id", "camera", "pixels"),
                    ),
                }
            )
        if self.agent_motion is not None:
            tools.append(
                {
                    "name": "piper_move_joint",
                    "description": (
                        "Move one operator-allowlisted Piper joint by a small relative delta in radians, "
                        "then latch the exact target as a software-only hold. Requires a live operator "
                        "arming lease. It cannot enable, reset, home, disable, or command the gripper."
                    ),
                    "inputSchema": _schema(
                        {
                            "arm": arm,
                            "joint": {"type": "integer", "minimum": 1, "maximum": 6},
                            "delta_rad": {
                                "type": "number",
                                "minimum": -self.motion_settings.max_delta_rad,
                                "maximum": self.motion_settings.max_delta_rad,
                            },
                        },
                        ("arm", "joint", "delta_rad"),
                    ),
                }
            )
        if self.gripper_motion is not None:
            tools.append(
                {
                    "name": "piper_gripper",
                    "description": (
                        "Control one commissioned Piper gripper independently of any task. "
                        "open and close use commissioned endpoints; move accepts an absolute "
                        "opening in metres. Requires operator arming and returns a durable receipt."
                    ),
                    "inputSchema": _schema(
                        {
                            "arm": {
                                "type": "string",
                                "enum": sorted(self.gripper_settings.profiles),
                            },
                            "action": {
                                "type": "string",
                                "enum": ["open", "close", "move"],
                            },
                            "width_m": {
                                "type": "number",
                                "minimum": 0.0,
                                "maximum": 0.10,
                            },
                        },
                        ("arm", "action"),
                    ),
                }
            )
        return tools

    def validate(self, name, arguments):
        if name == "piper_status":
            if arguments:
                raise ValueError(f"{name} takes no arguments")
            return
        if name == "piper_observe":
            if self.cameras is None:
                if arguments:
                    raise ValueError("piper_observe takes no arguments in a state-only session")
                return
            if set(arguments) - {"cameras"}:
                raise ValueError("piper_observe accepts only cameras")
            selected = arguments.get("cameras")
            if selected is None:
                return
            if not isinstance(selected, list) or not selected:
                raise ValueError("cameras must be a non-empty list")
            if len(set(selected)) != len(selected):
                raise ValueError("camera selection contains duplicates")
            unknown = sorted(set(selected) - set(self.camera_names))
            if unknown:
                raise ValueError(f"unknown Piper cameras: {unknown}")
            return
        if name == "piper_check_joints":
            if set(arguments) != {"arm", "positions"}:
                raise ValueError("piper_check_joints requires arm and positions")
            if arguments["arm"] not in {"left", "right"}:
                raise ValueError("arm must be left or right")
            if not isinstance(arguments["positions"], list):
                raise ValueError("positions must be a list")
            return
        if name == "piper_plan_pose":
            allowed = {"arm", "position_m", "rpy_rad", "relative", "waypoints"}
            if set(arguments) - allowed or not {"arm", "position_m"}.issubset(arguments):
                raise ValueError("piper_plan_pose requires arm and position_m")
            if arguments["arm"] not in {"left", "right"}:
                raise ValueError("arm must be left or right")
            if not isinstance(arguments["position_m"], list):
                raise ValueError("position_m must be a list")
            if "rpy_rad" in arguments and not isinstance(arguments["rpy_rad"], list):
                raise ValueError("rpy_rad must be a list")
            if "relative" in arguments and type(arguments["relative"]) is not bool:
                raise ValueError("relative must be a boolean")
            if "waypoints" in arguments and type(arguments["waypoints"]) is not int:
                raise ValueError("waypoints must be an integer")
            return
        if name == "piper_pixel_to_position":
            if self.observation_profile != "rgbd" or not self.camera_models:
                raise PermissionError("calibrated Piper RGB-D localization is unavailable")
            if set(arguments) - {"frame_id", "camera", "pixels", "radius"} or not {
                "frame_id",
                "camera",
                "pixels",
            }.issubset(arguments):
                raise ValueError(
                    "piper_pixel_to_position requires frame_id, camera, and pixels"
                )
            if arguments["camera"] not in self.camera_models:
                raise ValueError("camera is not calibrated for Piper base-frame localization")
            pixels = arguments["pixels"]
            if not isinstance(pixels, list) or not 1 <= len(pixels) <= 64:
                raise ValueError("pixels must contain 1..64 pixel pairs")
            if any(not isinstance(pixel, list) or len(pixel) != 2 for pixel in pixels):
                raise ValueError("each pixel must contain u and v")
            radius = arguments.get("radius", 3)
            if type(radius) is not int or not 0 <= radius <= 15:
                raise ValueError("radius must be an integer from 0 to 15")
            self._frame_dir(arguments["frame_id"])
            return
        if name == "piper_move_joint":
            if self.agent_motion is None:
                raise PermissionError("Piper Agent motion is disabled by operator configuration")
            if set(arguments) != {"arm", "joint", "delta_rad"}:
                raise ValueError("piper_move_joint requires arm, joint, and delta_rad")
            self.agent_motion.validate_request(
                arguments["arm"], arguments["joint"], arguments["delta_rad"]
            )
            return
        if name == "piper_gripper":
            if self.gripper_motion is None:
                raise PermissionError("Piper gripper motion is disabled")
            if set(arguments) - {"arm", "action", "width_m"} or not {
                "arm",
                "action",
            }.issubset(arguments):
                raise ValueError("piper_gripper requires arm and action")
            self.gripper_motion.validate_request(
                arguments["arm"], arguments["action"], arguments.get("width_m")
            )
            return
        if name == "piper_hold":
            if self.cartesian_motion is None:
                raise PermissionError("Piper Cartesian motion is disabled")
            if set(arguments) != {"arm"} or arguments["arm"] not in {"left", "right"}:
                raise ValueError("piper_hold requires arm=left or arm=right")
            if arguments["arm"] not in self.cartesian_settings.allowed_arms:
                raise PermissionError(
                    f"Cartesian motion is not enabled for the {arguments['arm']} arm"
                )
            return
        if name in {"piper_execute_plan", "piper_retreat_plan", "piper_discard_plan"}:
            if self.cartesian_motion is None:
                raise PermissionError("Piper Cartesian motion is disabled")
            allowed = {"plan_id"} if name == "piper_discard_plan" else {"plan_id", "waypoints"}
            if set(arguments) - allowed or "plan_id" not in arguments:
                raise ValueError(f"{name} requires plan_id")
            plan_id = arguments["plan_id"]
            if (
                not isinstance(plan_id, str)
                or len(plan_id) != 12
                or any(character not in "0123456789abcdef" for character in plan_id)
            ):
                raise ValueError("plan_id must contain 12 lowercase hexadecimal characters")
            if name == "piper_discard_plan":
                return
            waypoint_count = arguments.get("waypoints")
            if name == "piper_execute_plan":
                self.cartesian_motion.validate_execute(plan_id, waypoint_count)
            else:
                self.cartesian_motion.validate_retreat(plan_id, waypoint_count)
            return
        raise ValueError(f"unknown tool {name}")

    def _snapshot(self) -> dict:
        snapshot = self.reader.snapshot()
        max_age = float(self.options["feedback_max_age_s"])
        ready = True
        reasons = []
        for side, arm in snapshot["arms"].items():
            arm["tcp_pose"] = pose_report(arm["joints_rad"])
            arm["tool_pose"] = tool_pose_report(
                arm["joints_rad"], self.tool_offset_tcp_m
            )
            for source in ("joint", "status", "gripper"):
                age = arm["feedback"][f"{source}_age_s"]
                if age is None or age > max_age:
                    ready = False
                    reasons.append(f"{side} {source} feedback is stale or missing")
            if arm["controller"]["arm_status"] != 0:
                ready = False
                reasons.append(f"{side} controller reports arm_status={arm['controller']['arm_status']}")
            if any(arm["controller"]["errors"].values()):
                ready = False
                reasons.append(f"{side} controller reports joint errors")
        return {
            "robot": "dual_piper",
            "phase": (
                "bounded_agent_motion"
                if any(
                    controller is not None
                    for controller in (
                        self.agent_motion,
                        self.cartesian_motion,
                        self.gripper_motion,
                    )
                )
                else "read_only_bringup"
            ),
            "motion_tools_available": (
                any(
                    controller is not None
                    for controller in (
                        self.agent_motion,
                        self.cartesian_motion,
                        self.gripper_motion,
                    )
                )
            ),
            "agent_motion": (
                self.motion_settings.public()
                if self.agent_motion is None
                else self.agent_motion.status()
            ),
            "cartesian_motion": (
                self.cartesian_settings.public()
                if self.cartesian_motion is None
                else self.cartesian_motion.status()
            ),
            "gripper_motion": (
                self.gripper_settings.public()
                if self.gripper_motion is None
                else self.gripper_motion.status()
            ),
            "episode_recording": (
                {"enabled": False}
                if self.episode_recorder is None
                else {"enabled": True, **self.episode_recorder.status()}
            ),
            "camera_observation": {
                "available": self.cameras is not None,
                "profile": self.observation_profile,
                "cameras": list(self.camera_names),
                "calibrated_cameras": sorted(self.camera_models),
                "pixel_localization_available": bool(
                    self.observation_profile == "rgbd" and self.camera_models
                ),
            },
            "ready": ready,
            "readiness_reasons": reasons,
            **snapshot,
        }

    def status(self):
        return self._snapshot()

    def call(self, name, arguments):
        self.validate(name, arguments)
        if name == "piper_status":
            return self._snapshot(), []
        if name == "piper_observe":
            if self.cameras is None:
                return self._snapshot(), []
            return self._observe(arguments.get("cameras"))
        if name == "piper_pixel_to_position":
            return self._pixel_to_position(
                arguments["frame_id"],
                arguments["camera"],
                arguments["pixels"],
                arguments.get("radius", 3),
            )
        if name == "piper_move_joint":
            return self.agent_motion.execute(
                arguments["arm"], arguments["joint"], arguments["delta_rad"]
            ), []
        if name == "piper_gripper":
            return self.gripper_motion.execute(
                arguments["arm"], arguments["action"], arguments.get("width_m")
            ), []
        if name == "piper_execute_plan":
            return self.cartesian_motion.execute(
                arguments["plan_id"], waypoint_count=arguments.get("waypoints")
            ), []
        if name == "piper_retreat_plan":
            return self.cartesian_motion.retreat(
                arguments["plan_id"], waypoint_count=arguments.get("waypoints")
            ), []
        if name == "piper_discard_plan":
            return self.cartesian_motion.discard(arguments["plan_id"]), []
        if name == "piper_hold":
            return self.cartesian_motion.hold(arguments["arm"]), []
        snapshot = self._snapshot()
        side = arguments["arm"]
        if name == "piper_plan_pose":
            require_workspace = self.cartesian_settings.require_workspace_checks
            bounds = self.workspace_bounds.get(side) if require_workspace else None
            report = plan_pose(
                snapshot["arms"][side]["joints_rad"],
                arguments["position_m"],
                rpy_rad=arguments.get("rpy_rad"),
                relative=arguments.get("relative", False),
                waypoints=arguments.get("waypoints", 12),
                joint_margin_rad=float(self.options["joint_limit_margin_rad"]),
                max_joint_step_rad=float(self.options["max_joint_step_rad"]),
                workspace_min=None if bounds is None else bounds["min"],
                workspace_max=None if bounds is None else bounds["max"],
                link_min_z_m=(
                    self.options["link_min_z_m"] if require_workspace else None
                ),
                tool_offset_tcp_m=self.tool_offset_tcp_m,
            )
            report.update(
                {
                    "arm": side,
                    "frame": f"piper_{side}_base",
                    "state_ready": snapshot["ready"],
                    "state_readiness_reasons": snapshot["readiness_reasons"],
                    "planning_accepted": snapshot["ready"] and report["status"] == "passed",
                }
            )
            report["execution_admissible"] = False
            if self.cartesian_motion is not None and report["planning_accepted"]:
                try:
                    report.update(self.cartesian_motion.stage(report, arm=side))
                except (PermissionError, ValueError) as exc:
                    report["execution_reasons"] = [str(exc)]
            return report, []
        report = validate_joint_target(
            snapshot["arms"][side]["joints_rad"],
            arguments["positions"],
            margin_rad=float(self.options["joint_limit_margin_rad"]),
            max_step_rad=float(self.options["max_joint_step_rad"]),
        )
        target_accepted = report["accepted"]
        report["accepted"] = snapshot["ready"] and target_accepted
        return {
            "arm": side,
            "dry_run": True,
            "state_ready": snapshot["ready"],
            "state_readiness_reasons": snapshot["readiness_reasons"],
            "target_within_limits": target_accepted,
            **report,
        }, []

    @staticmethod
    def _workspace_bounds(value) -> dict:
        if value is None:
            return {}
        if (
            not isinstance(value, dict)
            or not value
            or not set(value).issubset({"left", "right"})
        ):
            raise ValueError("workspace_bounds must configure left, right, or both")
        result = {}
        for side, bounds in value.items():
            if not isinstance(bounds, dict) or set(bounds) != {"min", "max"}:
                raise ValueError(f"workspace_bounds.{side} requires min and max")
            minimum = np.asarray(bounds["min"], dtype=np.float64)
            maximum = np.asarray(bounds["max"], dtype=np.float64)
            if (
                minimum.shape != (3,)
                or maximum.shape != (3,)
                or not np.all(np.isfinite(minimum))
                or not np.all(np.isfinite(maximum))
                or np.any(minimum >= maximum)
            ):
                raise ValueError(
                    f"workspace_bounds.{side} must contain ordered finite 3-D vectors"
                )
            result[side] = {"min": minimum.tolist(), "max": maximum.tolist()}
        return result

    @staticmethod
    def _rgb_image(value) -> Image.Image:
        if isinstance(value, Image.Image):
            return value.convert("RGB")
        return Image.fromarray(value).convert("RGB")

    @staticmethod
    def _depth_image(value) -> Image.Image:
        if isinstance(value, Image.Image):
            return value
        return Image.fromarray(value)

    @staticmethod
    def _max_joint_movement(before: dict, after: dict) -> float:
        deltas = []
        for side in ("left", "right"):
            start = before["arms"][side]["joints_rad"]
            finish = after["arms"][side]["joints_rad"]
            deltas.extend(abs(float(b) - float(a)) for a, b in zip(start, finish))
        return max(deltas, default=0.0)

    def _observe(self, selected=None):
        names = tuple(selected or self.camera_names)
        frame_id = (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            + uuid.uuid4().hex[:6]
        )
        out_dir = self.frames_root / frame_id
        before = self._wait_for_ready_snapshot()
        captured = self.cameras.capture(names, timeout_ms=self.options["camera_timeout_ms"])
        after = self._wait_for_ready_snapshot()
        moved = self._max_joint_movement(before, after)
        out_dir.mkdir(parents=True, mode=0o755)

        images = []
        report = {}
        metadata = {
            "schema_version": 1,
            "frame_id": frame_id,
            "captured_at": float(self.clock()),
            "observation_profile": self.observation_profile,
            "depth_units": (
                "mm (uint16, 0 = invalid)"
                if self.observation_profile == "rgbd"
                else None
            ),
            "max_joint_movement_during_capture_rad": moved,
            "state_before": before,
            "state_after": after,
            "cameras": {},
        }
        arm_end_transforms = {
            side: end_transform(after["arms"][side]["joints_rad"])
            for side in ("left", "right")
        }
        for name in names:
            frame = captured[name]
            rgb = self._rgb_image(frame.color_rgb)
            rgb_name = f"{name}.jpg"
            rgb.save(out_dir / rgb_name, format="JPEG", quality=95)
            depth_name = None
            if frame.depth_mm is not None:
                depth_name = f"{name}_depth.png"
                self._depth_image(frame.depth_mm).save(out_dir / depth_name, format="PNG")

            preview = rgb.copy()
            if preview.width > self.options["agent_image_width"]:
                height = max(
                    1,
                    round(
                        preview.height
                        * self.options["agent_image_width"]
                        / preview.width
                    ),
                )
                preview = preview.resize(
                    (self.options["agent_image_width"], height), Image.Resampling.LANCZOS
                )
            encoded = io.BytesIO()
            preview.save(
                encoded,
                format="JPEG",
                quality=self.options["agent_jpeg_quality"],
            )
            images.append(
                {
                    "type": "image",
                    "mimeType": "image/jpeg",
                    "data": base64.b64encode(encoded.getvalue()).decode("ascii"),
                }
            )
            camera_metadata = {
                "serial": frame.serial,
                "rgb": rgb_name,
                "depth": depth_name,
                "width": rgb.width,
                "height": rgb.height,
                "captured_at": frame.captured_at,
                "device_timestamp_ms": frame.device_timestamp_ms,
                "intrinsics": frame.intrinsics,
            }
            model = self.camera_models.get(name)
            if model is not None:
                camera_metadata["mount"] = model.mount
                camera_metadata["base_frame"] = model.base_frame
                camera_metadata["T_base_camera"] = model.base_from_camera(
                    arm_end_transforms
                ).tolist()
            metadata["cameras"][name] = camera_metadata
            report[name] = {
                "width": rgb.width,
                "height": rgb.height,
                "preview_width": preview.width,
                "preview_height": preview.height,
                "captured_at": frame.captured_at,
                "device_timestamp_ms": frame.device_timestamp_ms,
                "depth_available": depth_name is not None,
            }
        (out_dir / "meta.json").write_text(json.dumps(metadata, indent=2) + "\n")
        return {
            **after,
            "frame_id": frame_id,
            "directory": f"runtime/frames/{frame_id}",
            "images_in_order": list(names),
            "cameras": report,
            "max_joint_movement_during_capture_rad": moved,
        }, images

    def _wait_for_ready_snapshot(self) -> dict:
        deadline = time.monotonic() + float(self.options["feedback_startup_timeout_s"])
        while True:
            snapshot = self._snapshot()
            if snapshot["ready"]:
                return snapshot
            if time.monotonic() >= deadline:
                reasons = "; ".join(snapshot["readiness_reasons"])
                raise RuntimeError(
                    "Piper feedback did not become ready before RGB-D capture: " + reasons
                )
            time.sleep(0.01)

    def _frame_dir(self, frame_id: str) -> Path:
        if not isinstance(frame_id, str) or not frame_id.isalnum():
            raise ValueError("invalid frame_id")
        path = self.frames_root / frame_id
        if not (path / "meta.json").is_file():
            raise ValueError(f"unknown frame_id {frame_id}")
        return path

    def _pixel_to_position(self, frame_id, camera, pixels, radius=3):
        directory = self._frame_dir(frame_id)
        metadata = json.loads((directory / "meta.json").read_text())
        entry = metadata["cameras"].get(camera)
        if entry is None or not entry.get("depth"):
            raise ValueError(f"frame {frame_id} has no depth for camera {camera}")
        if "T_base_camera" not in entry:
            raise ValueError(f"frame {frame_id} has no base-frame extrinsics for camera {camera}")
        depth = np.asarray(Image.open(directory / entry["depth"]), dtype=np.uint16)
        height, width = depth.shape
        points = []
        for u_value, v_value in pixels:
            u, v = float(u_value), float(v_value)
            if not np.isfinite(u) or not np.isfinite(v) or not (0 <= u < width and 0 <= v < height):
                points.append({"pixel": [u_value, v_value], "error": "outside the image"})
                continue
            ui = min(width - 1, max(0, int(round(u))))
            vi = min(height - 1, max(0, int(round(v))))
            window = depth[
                max(0, vi - radius) : min(height, vi + radius + 1),
                max(0, ui - radius) : min(width, ui + radius + 1),
            ]
            valid = window[window > 0]
            if valid.size == 0:
                points.append({"pixel": [u_value, v_value], "error": "no valid depth here"})
                continue
            depth_m = float(np.median(valid)) / 1000.0
            point_camera = deproject_pixel(u, v, depth_m, entry["intrinsics"])
            point_base = transform_point(entry["T_base_camera"], point_camera)
            points.append(
                {
                    "pixel": [u_value, v_value],
                    "position_m": np.round(point_base, 5).tolist(),
                    "depth_m": round(depth_m, 5),
                    "valid_fraction": round(float(valid.size / window.size), 3),
                }
            )
        return {
            "frame_id": frame_id,
            "camera": camera,
            "frame": entry["base_frame"],
            "points": points,
        }, []

    def close(self):
        try:
            try:
                if self.episode_recorder is not None:
                    self.episode_recorder.stop()
            finally:
                if self.cameras is not None:
                    self.cameras.close()
        finally:
            self.reader.close()
