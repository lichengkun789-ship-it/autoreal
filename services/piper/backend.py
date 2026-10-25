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
from pathlib import Path
import time
import uuid

from PIL import Image

from .agent_motion import AgentMotionSettings, PiperAgentMotionController
from .camera import RealSenseRig, camera_specs
from .safety import validate_joint_target
from .sdk_reader import PiperSdkReader


DEFAULTS = {
    "arms": None,
    "feedback_max_age_s": 0.25,
    "joint_limit_margin_rad": 0.02,
    "max_joint_step_rad": 0.05,
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
        if float(self.options["joint_limit_margin_rad"]) < 0:
            raise ValueError("joint_limit_margin_rad must be non-negative")
        if float(self.options["max_joint_step_rad"]) <= 0:
            raise ValueError("max_joint_step_rad must be positive")
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
        self.cameras = None
        self.frames_root: Path | None = None
        if self.observation_profile != "state-only":
            specs = camera_specs(self.options["cameras"])
            root = getattr(config, "root", None)
            if root is None:
                raise ValueError("camera observation requires a session root")
            self.camera_names = tuple(spec.name for spec in specs)
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
        ]
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
        if name == "piper_move_joint":
            return self.agent_motion.execute(
                arguments["arm"], arguments["joint"], arguments["delta_rad"]
            ), []
        snapshot = self._snapshot()
        side = arguments["arm"]
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
        before = self._snapshot()
        captured = self.cameras.capture(names, timeout_ms=self.options["camera_timeout_ms"])
        after = self._snapshot()
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
            metadata["cameras"][name] = {
                "serial": frame.serial,
                "rgb": rgb_name,
                "depth": depth_name,
                "width": rgb.width,
                "height": rgb.height,
                "captured_at": frame.captured_at,
                "device_timestamp_ms": frame.device_timestamp_ms,
                "intrinsics": frame.intrinsics,
            }
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

    def close(self):
        try:
            if self.cameras is not None:
                self.cameras.close()
        finally:
            self.reader.close()
