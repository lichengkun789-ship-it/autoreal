"""RAX backend for a dual-arm Piper system.

Observation and target validation are always available. A single-joint relative motion
tool appears only when operator configuration binds it to passed characterization
evidence and an explicit move allowlist.
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
from .ik import plan_pose
from .kinematics import end_transform, pose_report
from .safety import validate_joint_target
from .sdk_reader import PiperSdkReader


DEFAULTS = {
    "arms": None,
    "feedback_max_age_s": 0.25,
    "feedback_startup_timeout_s": 2.0,
    "joint_limit_margin_rad": 0.02,
    "max_joint_step_rad": 0.05,
    "workspace_bounds": None,
    "link_min_z_m": None,
    "agent_motion": None,
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
    """RAX backend contract backed by Piper SDK state and narrow motion admission."""

    def __init__(self, config, *, reader=None, cameras=None, clock=time.time):
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
            if reader is None or (
                isinstance(motion_options, dict) and motion_options.get("enabled") is True
            ):
                raise ValueError(
                    "piper backend option 'arms' is required; see examples/piper.json"
                )
            arms = {}
        self.motion_settings = AgentMotionSettings.from_options(
            self.options["agent_motion"], arms=arms
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

        if self.motion_settings.enabled:
            root = getattr(config, "root", None)
            if root is None:
                raise ValueError("enabled agent motion requires a session root")
            self.agent_motion = PiperAgentMotionController(
                root, reader, self.motion_settings
            )
            self.mutating = frozenset({"piper_move_joint"})
        else:
            self.agent_motion = None
            self.mutating = frozenset()

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
                    "Solve Piper inverse kinematics and validate a straight Cartesian path "
                    "from fresh measured joints. Returns joint waypoints and refusal reasons; "
                    "it is a read-only plan and never authorizes or sends motion."
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
                                "Absolute target in the selected base frame, or an XYZ "
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
        raise ValueError(f"unknown tool {name}")

    def _snapshot(self) -> dict:
        snapshot = self.reader.snapshot()
        max_age = float(self.options["feedback_max_age_s"])
        ready = True
        reasons = []
        for side, arm in snapshot["arms"].items():
            arm["tcp_pose"] = pose_report(arm["joints_rad"])
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
                if self.agent_motion is not None
                else "read_only_bringup"
            ),
            "motion_tools_available": self.agent_motion is not None,
            "agent_motion": (
                self.motion_settings.public()
                if self.agent_motion is None
                else self.agent_motion.status()
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
        snapshot = self._snapshot()
        side = arguments["arm"]
        if name == "piper_plan_pose":
            bounds = self.workspace_bounds.get(side)
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
                link_min_z_m=self.options["link_min_z_m"],
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
            if self.cameras is not None:
                self.cameras.close()
        finally:
            self.reader.close()
