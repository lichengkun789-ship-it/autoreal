"""Read-only RAX backend for a dual-arm Piper system.

The first bring-up milestone deliberately exposes observation and dry-run validation
only. No tool in this backend can command hardware yet.
"""

from __future__ import annotations

import time

from .safety import validate_joint_target
from .sdk_reader import PiperSdkReader


DEFAULTS = {
    "arms": None,
    "feedback_max_age_s": 0.25,
    "joint_limit_margin_rad": 0.02,
    "max_joint_step_rad": 0.05,
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
    """RAX backend contract backed by read-only Piper SDK state."""

    mutating = frozenset()

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
        if reader is None:
            if not isinstance(arms, dict):
                raise ValueError(
                    "piper backend option 'arms' is required; see examples/piper.json"
                )
            reader = PiperSdkReader(arms, clock=clock)
        self.reader = reader

    def tools(self):
        arm = {"type": "string", "enum": ["left", "right"]}
        positions = {
            "type": "array",
            "items": {"type": "number"},
            "minItems": 6,
            "maxItems": 6,
            "description": "Six absolute joint targets in radians.",
        }
        return [
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
            "phase": "read_only_bringup",
            "motion_tools_available": False,
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
