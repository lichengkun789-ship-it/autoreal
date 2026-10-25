"""Configuration-gated, bounded Piper motion for the RAX Agent.

This module deliberately exposes only a relative single-joint step. It reuses the
hardware-characterized minimum-motion runner and never enables motors, clears faults,
homes, resets, disables, or commands the gripper.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

from services import arming
from services.json_io import write_private_json

from .minimum_motion import MAX_DELTA_RAD, MAX_SPEED_PERCENT, PiperMinimumMotionRunner


HARD_MAX_COMMANDS_PER_ARMING = 10
HARD_MAX_CUMULATIVE_DELTA_RAD = 0.05
HARD_MAX_COMMAND_WINDOW_S = 0.5
DIRECTIONS = frozenset({"positive", "negative"})


class AgentMotionStopped(RuntimeError):
    """A bounded Agent motion was refused or ended in a known hold state."""

    def __init__(self, message: str, *, control_certain: bool, robot_state=None):
        super().__init__(message)
        self.control_certain = control_certain
        self.robot_state = robot_state


def _direction(delta_rad: float) -> str:
    return "positive" if delta_rad > 0 else "negative"


def _read_characterization(path: Path, arms: dict) -> dict:
    raw = path.read_bytes()
    value = json.loads(raw)
    if value.get("kind") != "piper_minimum_motion_characterization":
        raise ValueError(f"{path}: not a Piper minimum-motion characterization")
    if value.get("status") != "passed" or value.get("motion_command_attempted") is not True:
        raise ValueError(f"{path}: characterization must be a passed executed motion")
    requested = value.get("requested", {})
    arm = requested.get("arm")
    joint = requested.get("joint")
    delta = requested.get("delta_rad")
    speed = requested.get("speed_percent")
    if (
        arm not in {"left", "right"}
        or type(joint) is not int
        or not 1 <= joint <= 6
        or not isinstance(delta, (int, float))
        or not math.isfinite(delta)
        or delta == 0
        or type(speed) is not int
    ):
        raise ValueError(f"{path}: characterization request is incomplete")
    if requested.get("execute") is not True or requested.get("allow_enable") is not False:
        raise ValueError(f"{path}: characterization must execute without enabling the arm")
    expected_can = {side: spec.get("can_name") for side, spec in arms.items()}
    observed_can = {
        side: value.get("can_interfaces", {}).get(side, {}).get("name")
        for side in ("left", "right")
    }
    if observed_can != expected_can:
        raise ValueError(f"{path}: characterization CAN interfaces do not match this robot")
    hold = value.get("hold", {})
    if (
        hold.get("state") != "hold_latched"
        or hold.get("hold_sdk_calls_returned") is not True
        or hold.get("hold_target_source") != "last_commanded_target"
        or hold.get("hold_sides") != [arm]
    ):
        raise ValueError(f"{path}: characterization did not confirm selected-arm last-target hold")
    measurements = value.get("measurements", {})
    if not isinstance(measurements.get("active_joint_travel_rad"), (int, float)):
        raise ValueError(f"{path}: characterization is missing measured joint travel")
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "arm": arm,
        "joint": joint,
        "direction": _direction(float(delta)),
        "delta_rad": abs(float(delta)),
        "speed_percent": speed,
        "robot_asset_id": value.get("operator_confirmation", {}).get("robot_asset_id"),
        "active_joint_travel_rad": float(measurements["active_joint_travel_rad"]),
        "command_window_s": measurements.get("command_window_s"),
    }


@dataclass(frozen=True)
class AgentMotionSettings:
    enabled: bool
    allowed: frozenset[tuple[str, int, str]]
    max_delta_rad: float
    speed_percent: int
    command_window_s: float
    max_commands_per_arming: int
    max_cumulative_delta_rad: float
    evidence: tuple[dict, ...]

    @classmethod
    def from_options(cls, value, *, arms: dict) -> "AgentMotionSettings":
        value = {} if value is None else value
        if not isinstance(value, dict):
            raise ValueError("agent_motion must be an object")
        allowed_keys = {
            "enabled",
            "allowed_moves",
            "max_delta_rad",
            "speed_percent",
            "command_window_s",
            "max_commands_per_arming",
            "max_cumulative_delta_rad",
            "characterization_receipts",
        }
        unknown = set(value) - allowed_keys
        if unknown:
            raise ValueError(f"unknown agent_motion options: {sorted(unknown)}")
        enabled = value.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("agent_motion.enabled must be true or false")

        max_delta = float(value.get("max_delta_rad", 0.002))
        speed = value.get("speed_percent", 3)
        window = float(value.get("command_window_s", 0.5))
        max_commands = value.get("max_commands_per_arming", 5)
        cumulative = float(value.get("max_cumulative_delta_rad", 0.025))
        if not math.isfinite(max_delta) or not 0 < max_delta <= MAX_DELTA_RAD:
            raise ValueError(f"agent max_delta_rad must be in (0, {MAX_DELTA_RAD}]")
        if type(speed) is not int or not 1 <= speed <= MAX_SPEED_PERCENT:
            raise ValueError(f"agent speed_percent must be in 1..{MAX_SPEED_PERCENT}")
        if not math.isfinite(window) or not 0 < window <= HARD_MAX_COMMAND_WINDOW_S:
            raise ValueError(
                f"agent command_window_s must be in (0, {HARD_MAX_COMMAND_WINDOW_S}]"
            )
        if type(max_commands) is not int or not 1 <= max_commands <= HARD_MAX_COMMANDS_PER_ARMING:
            raise ValueError(
                "agent max_commands_per_arming must be in "
                f"1..{HARD_MAX_COMMANDS_PER_ARMING}"
            )
        if not math.isfinite(cumulative) or not 0 < cumulative <= HARD_MAX_CUMULATIVE_DELTA_RAD:
            raise ValueError(
                "agent max_cumulative_delta_rad must be in "
                f"(0, {HARD_MAX_CUMULATIVE_DELTA_RAD}]"
            )
        if cumulative + 1e-12 < max_delta:
            raise ValueError("agent cumulative delta budget must cover at least one maximum step")

        allowed = set()
        moves = value.get("allowed_moves", [])
        if not isinstance(moves, list):
            raise ValueError("agent_motion.allowed_moves must be a list")
        for index, move in enumerate(moves):
            if not isinstance(move, dict) or set(move) != {"arm", "joint", "directions"}:
                raise ValueError(
                    f"agent_motion.allowed_moves[{index}] requires arm, joint, and directions"
                )
            arm, joint, directions = move["arm"], move["joint"], move["directions"]
            if arm not in {"left", "right"} or type(joint) is not int or not 1 <= joint <= 6:
                raise ValueError(f"agent_motion.allowed_moves[{index}] has an invalid arm or joint")
            if (
                not isinstance(directions, list)
                or not directions
                or set(directions) - DIRECTIONS
                or len(directions) != len(set(directions))
            ):
                raise ValueError(
                    f"agent_motion.allowed_moves[{index}].directions must contain positive and/or negative"
                )
            allowed.update((arm, joint, direction) for direction in directions)

        receipts = value.get("characterization_receipts", [])
        if not isinstance(receipts, list) or any(not isinstance(path, str) or not path for path in receipts):
            raise ValueError("agent_motion.characterization_receipts must be a list of paths")
        # Explicitly disabling motion must always provide a read-only fallback, even if
        # an old evidence path was removed from this workstation.
        evidence = (
            tuple(_read_characterization(Path(path).expanduser(), arms) for path in receipts)
            if enabled
            else ()
        )
        if enabled:
            if not allowed:
                raise ValueError("enabled agent motion requires at least one explicitly allowed move")
            if not evidence:
                raise ValueError("enabled agent motion requires characterization receipts")
            for arm, joint, direction in allowed:
                covered = any(
                    item["arm"] == arm
                    and item["joint"] == joint
                    and item["direction"] == direction
                    and item["delta_rad"] + 1e-12 >= max_delta
                    and item["speed_percent"] >= speed
                    for item in evidence
                )
                if not covered:
                    raise ValueError(
                        f"no characterization covers {arm} J{joint} {direction} at "
                        f"{max_delta} rad and speed {speed}%"
                    )
        return cls(
            enabled=enabled,
            allowed=frozenset(allowed),
            max_delta_rad=max_delta,
            speed_percent=speed,
            command_window_s=window,
            max_commands_per_arming=max_commands,
            max_cumulative_delta_rad=cumulative,
            evidence=evidence,
        )

    def public(self) -> dict:
        moves = [
            {"arm": arm, "joint": joint, "direction": direction}
            for arm, joint, direction in sorted(self.allowed)
        ]
        return {
            "enabled": self.enabled,
            "allowed_moves": moves,
            "max_delta_rad": self.max_delta_rad,
            "speed_percent": self.speed_percent,
            "command_window_s": self.command_window_s,
            "max_commands_per_arming": self.max_commands_per_arming,
            "max_cumulative_delta_rad": self.max_cumulative_delta_rad,
            "characterization_receipts": [
                {key: item[key] for key in ("path", "sha256", "arm", "joint", "direction")}
                for item in self.evidence
            ],
        }


class PiperAgentMotionController:
    """Execute and account for one bounded joint step at a time."""

    def __init__(
        self,
        config_root: Path,
        reader,
        settings: AgentMotionSettings,
        *,
        sleeper=time.sleep,
        arming_reader=arming.read,
    ):
        if not settings.enabled:
            raise ValueError("cannot create an Agent motion controller while motion is disabled")
        self.root = Path(config_root)
        self.reader = reader
        self.settings = settings
        self.sleeper = sleeper
        self.arming_reader = arming_reader
        self.lease_since = None
        self.commands_used = 0
        self.cumulative_delta_rad = 0.0
        self.last_motion = None

    def _armed_lease(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is not True:
            raise PermissionError(f"motion is not armed ({state.get('reason', 'unknown reason')})")
        since = state.get("since")
        if not isinstance(since, (int, float)) or not math.isfinite(since):
            raise PermissionError("operator arming lease has no valid creation time")
        if self.lease_since != since:
            self.lease_since = since
            self.commands_used = 0
            self.cumulative_delta_rad = 0.0
        return state

    def _require_same_lease(self, since: float) -> None:
        state = self._armed_lease()
        if state["since"] != since:
            raise PermissionError("operator arming lease changed during motion")

    def _check_budget(self, delta_rad: float) -> None:
        if self.commands_used >= self.settings.max_commands_per_arming:
            raise PermissionError("operator arming command budget is exhausted; disarm and inspect")
        proposed = self.cumulative_delta_rad + abs(delta_rad)
        if proposed > self.settings.max_cumulative_delta_rad + 1e-12:
            raise PermissionError("operator arming cumulative motion budget would be exceeded")

    def validate_request(self, arm, joint, delta_rad) -> tuple[str, int, float]:
        if arm not in {"left", "right"}:
            raise ValueError("arm must be left or right")
        if type(joint) is not int or not 1 <= joint <= 6:
            raise ValueError("joint must be an integer in 1..6")
        if not isinstance(delta_rad, (int, float)) or not math.isfinite(delta_rad) or delta_rad == 0:
            raise ValueError("delta_rad must be a finite non-zero number")
        delta_rad = float(delta_rad)
        if abs(delta_rad) > self.settings.max_delta_rad + 1e-12:
            raise ValueError(
                f"absolute delta exceeds the configured {self.settings.max_delta_rad} rad cap"
            )
        if (arm, joint, _direction(delta_rad)) not in self.settings.allowed:
            raise PermissionError(
                f"{arm} J{joint} {_direction(delta_rad)} is not in the operator allowlist"
            )
        self._armed_lease()
        self._check_budget(delta_rad)
        return arm, joint, delta_rad

    def status(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is True:
            try:
                self._armed_lease()
            except PermissionError:
                pass
        return {
            **self.settings.public(),
            "arming": state,
            "commands_used": self.commands_used,
            "commands_remaining": max(
                0, self.settings.max_commands_per_arming - self.commands_used
            ),
            "cumulative_delta_rad": self.cumulative_delta_rad,
            "cumulative_delta_remaining_rad": max(
                0.0,
                self.settings.max_cumulative_delta_rad - self.cumulative_delta_rad,
            ),
            "last_motion": self.last_motion,
        }

    def _save(self, receipt: dict) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = self.root / "piper-motion" / f"agent_joint_step_{stamp}_{uuid.uuid4().hex[:8]}.json"
        write_private_json(path, receipt, directory_mode=0o700)
        return path

    @staticmethod
    def _last_state(receipt: dict):
        samples = receipt.get("post_hold_samples") or []
        return samples[-1] if samples else receipt.get("pre_hold_state") or receipt.get("initial_state")

    def execute(self, arm, joint, delta_rad) -> dict:
        arm, joint, delta_rad = self.validate_request(arm, joint, delta_rad)
        lease = self._armed_lease()
        lease_since = lease["since"]
        runner = PiperMinimumMotionRunner(self.reader, sleeper=self.sleeper)
        receipt = runner.run(
            arm=arm,
            joint=joint,
            delta_rad=delta_rad,
            speed_percent=self.settings.speed_percent,
            operator_lease_id=f"agent:{lease_since}:{self.commands_used + 1}",
            execute=True,
            allow_enable=False,
            observation_seconds=self.settings.command_window_s,
            authorization_guard=lambda: self._require_same_lease(lease_since),
        )
        receipt["kind"] = "piper_agent_joint_step"
        receipt["agent_tool_exposed"] = True
        receipt["authorization"] = {
            "lease_since": lease_since,
            "lease_until": lease.get("until"),
            "lease_by": lease.get("by"),
        }
        receipt["admission"] = self.settings.public()
        if receipt.get("motion_command_attempted") is True:
            self.commands_used += 1
            self.cumulative_delta_rad += abs(delta_rad)
        path = self._save(receipt)
        self.last_motion = {
            "status": receipt.get("status"),
            "arm": arm,
            "joint": joint,
            "delta_rad": delta_rad,
            "receipt": str(path),
            "finished_at": receipt.get("finished_at"),
        }
        if receipt.get("status") != "passed":
            hold = receipt.get("hold", {})
            certain = receipt.get("motion_command_attempted") is not True or (
                hold.get("state") == "hold_latched"
                and hold.get("hold_sdk_calls_returned") is True
            )
            reasons = "; ".join(receipt.get("failure_reasons", ["bounded motion failed"]))
            raise AgentMotionStopped(
                f"Piper Agent motion stopped: {reasons}; receipt: {path}",
                control_certain=certain,
                robot_state=self._last_state(receipt),
            )
        measurements = receipt.get("measurements", {})
        return {
            "executed": True,
            "arm": arm,
            "joint": joint,
            "requested_delta_rad": delta_rad,
            "measured_active_joint_travel_rad": measurements.get("active_joint_travel_rad"),
            "max_other_joint_travel_rad": measurements.get("max_other_joint_travel_rad"),
            "max_post_hold_drift_rad": measurements.get("max_post_hold_drift_rad"),
            "hold": receipt.get("hold"),
            "final_state": self._last_state(receipt),
            "budget": {
                "commands_used": self.commands_used,
                "commands_remaining": self.settings.max_commands_per_arming - self.commands_used,
                "cumulative_delta_rad": self.cumulative_delta_rad,
                "cumulative_delta_remaining_rad": (
                    self.settings.max_cumulative_delta_rad - self.cumulative_delta_rad
                ),
            },
            "receipt": str(path),
        }
