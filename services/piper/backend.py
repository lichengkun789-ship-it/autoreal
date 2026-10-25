"""RAX backend for a dual-arm Piper system.

Observation and target validation are always available. A single-joint relative motion
tool appears only when operator configuration binds it to passed characterization
evidence and an explicit move allowlist.
"""

from __future__ import annotations

import time

from .agent_motion import AgentMotionSettings, PiperAgentMotionController
from .safety import validate_joint_target
from .sdk_reader import PiperSdkReader


DEFAULTS = {
    "arms": None,
    "feedback_max_age_s": 0.25,
    "joint_limit_margin_rad": 0.02,
    "max_joint_step_rad": 0.05,
    "agent_motion": None,
    "agent_python": None,
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

    def __init__(self, config, *, reader=None, clock=time.time):
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
        self.clock = clock
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
                "description": (
                    "Capture a fresh robot-state observation. Phase 1 returns measured arm state "
                    "without camera images and moves nothing."
                ),
                "inputSchema": _schema(),
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
        if name in {"piper_status", "piper_observe"}:
            if arguments:
                raise ValueError(f"{name} takes no arguments")
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
            "ready": ready,
            "readiness_reasons": reasons,
            **snapshot,
        }

    def status(self):
        return self._snapshot()

    def call(self, name, arguments):
        self.validate(name, arguments)
        if name in {"piper_status", "piper_observe"}:
            return self._snapshot(), []
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

    def close(self):
        self.reader.close()
