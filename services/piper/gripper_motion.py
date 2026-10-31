"""Commissioned, task-independent Piper gripper tool for the RAX backend."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

from services import arming
from services.json_io import write_private_json

from .agent_motion import AgentMotionStopped
from .gripper_characterization import GripperCycleSettings, PiperGripperSetRunner


HARD_MAX_COMMANDS_PER_ARMING = 50
HARD_MAX_STEP_M = 0.01
HARD_MAX_EFFORT = 500


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class GripperProfile:
    arm: str
    open_width_m: float
    closed_width_m: float
    runner_settings: GripperCycleSettings
    receipt_path: Path
    receipt_sha256: str

    @classmethod
    def from_receipt(cls, arm: str, path, *, backend_options: dict):
        receipt_path = Path(path).expanduser().resolve()
        try:
            raw = receipt_path.read_bytes()
            receipt = json.loads(raw)
            settings = GripperCycleSettings(**receipt["settings"])
            open_width = float(receipt["initial_width_m"])
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"cannot load {arm} gripper commissioning receipt {receipt_path}: {exc}"
            ) from exc
        close = receipt.get("close", ())
        restore = receipt.get("restore", ())
        if (
            receipt.get("kind") != "piper_operator_gripper_cycle"
            or receipt.get("status") != "passed"
            or receipt.get("arm") != arm
            or receipt.get("gripper_command_attempted") is not True
            or receipt.get("restored") is not True
            or not close
            or not restore
        ):
            raise ValueError(f"{arm} gripper commissioning receipt did not pass")
        if not 0 < open_width <= 0.10:
            raise ValueError(f"{arm} commissioned open width must be in (0, 0.10] m")
        if (
            settings.step_m > HARD_MAX_STEP_M
            or settings.effort > HARD_MAX_EFFORT
            or settings.target_tolerance_m > 0.003
            or settings.max_arm_joint_drift_rad > 0.002
        ):
            raise ValueError(f"{arm} gripper commissioning settings exceed backend limits")
        if (
            float(close[-1].get("command", {}).get("opening_m", math.nan)) != 0.0
            or close[-1].get("feedback", {}).get("reached") is not True
            or not math.isclose(
                float(restore[-1].get("command", {}).get("opening_m", math.nan)),
                open_width,
                abs_tol=1e-6,
            )
            or restore[-1].get("feedback", {}).get("reached") is not True
        ):
            raise ValueError(f"{arm} receipt did not prove close and restore targets")
        try:
            commissioned = json.loads(Path(receipt["backend_options_path"]).read_text())
        except (OSError, json.JSONDecodeError, KeyError, TypeError) as exc:
            raise ValueError(
                f"{arm} gripper receipt has no readable backend configuration"
            ) from exc
        expected_can = (backend_options.get("arms") or {}).get(arm, {}).get("can_name")
        observed_can = (commissioned.get("arms") or {}).get(arm, {}).get("can_name")
        if expected_can is None or expected_can != observed_can:
            raise ValueError(f"{arm} gripper was commissioned on a different CAN interface")
        return cls(
            arm=arm,
            open_width_m=open_width,
            closed_width_m=0.0,
            runner_settings=settings,
            receipt_path=receipt_path,
            receipt_sha256=hashlib.sha256(raw).hexdigest(),
        )

    def public(self) -> dict:
        return {
            "open_width_m": self.open_width_m,
            "closed_width_m": self.closed_width_m,
            "step_m": self.runner_settings.step_m,
            "effort": self.runner_settings.effort,
            "commissioning_receipt": {
                "path": str(self.receipt_path),
                "sha256": self.receipt_sha256,
            },
        }


@dataclass(frozen=True)
class GripperMotionSettings:
    enabled: bool
    profiles: dict[str, GripperProfile]
    max_commands_per_arming: int

    @classmethod
    def from_options(cls, value, *, backend_options: dict):
        value = {} if value is None else value
        if not isinstance(value, dict):
            raise ValueError("gripper_motion must be an object")
        unknown = set(value) - {
            "enabled",
            "commissioning_receipts",
            "max_commands_per_arming",
        }
        if unknown:
            raise ValueError(f"unknown gripper_motion options: {sorted(unknown)}")
        enabled = value.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("gripper_motion.enabled must be true or false")
        maximum = value.get("max_commands_per_arming", 20)
        if type(maximum) is not int or not 1 <= maximum <= HARD_MAX_COMMANDS_PER_ARMING:
            raise ValueError(
                "gripper max_commands_per_arming must be in "
                f"1..{HARD_MAX_COMMANDS_PER_ARMING}"
            )
        receipts = value.get("commissioning_receipts", {})
        if not isinstance(receipts, dict) or set(receipts) - {"left", "right"}:
            raise ValueError("gripper commissioning_receipts must map left/right to files")
        profiles = {
            arm: GripperProfile.from_receipt(
                arm, path, backend_options=backend_options
            )
            for arm, path in receipts.items()
        }
        if enabled and not profiles:
            raise ValueError("enabled gripper motion requires a commissioning receipt")
        return cls(enabled, profiles, maximum)

    def public(self) -> dict:
        return {
            "enabled": self.enabled,
            "allowed_arms": sorted(self.profiles),
            "actions": ["open", "close", "move"],
            "max_commands_per_arming": self.max_commands_per_arming,
            "profiles": {arm: profile.public() for arm, profile in self.profiles.items()},
        }


class PiperGripperMotionController:
    """Execute generic gripper targets under one operator arming lease."""

    def __init__(
        self,
        root: Path,
        reader,
        settings: GripperMotionSettings,
        *,
        arming_reader=arming.read,
        sleeper=time.sleep,
        runner_factory=PiperGripperSetRunner,
    ):
        if not settings.enabled:
            raise ValueError("cannot create a gripper controller while motion is disabled")
        self.root = Path(root)
        self.reader = reader
        self.settings = settings
        self.arming_reader = arming_reader
        self.sleeper = sleeper
        self.runner_factory = runner_factory
        self.lease_since = None
        self.commands_used = 0
        self.last_action = None

    def _armed(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is not True:
            raise PermissionError(
                f"gripper motion is not armed ({state.get('reason', 'unknown reason')})"
            )
        since = state.get("since")
        if not isinstance(since, (int, float)) or not math.isfinite(since):
            raise PermissionError("operator arming lease has no valid creation time")
        if since != self.lease_since:
            self.lease_since = since
            self.commands_used = 0
        return state

    def _require_same_lease(self, since: float) -> None:
        if self._armed().get("since") != since:
            raise PermissionError("operator arming lease changed during gripper motion")

    def _target(self, arm: str, action: str, width_m=None) -> tuple[GripperProfile, float]:
        profile = self.settings.profiles.get(arm)
        if profile is None:
            raise PermissionError(f"gripper motion is not commissioned for the {arm} arm")
        if action == "open":
            if width_m is not None:
                raise ValueError("open does not accept width_m")
            target = profile.open_width_m
        elif action == "close":
            if width_m is not None:
                raise ValueError("close does not accept width_m")
            target = profile.closed_width_m
        elif action == "move":
            if width_m is None:
                raise ValueError("move requires width_m")
            target = float(width_m)
            if not math.isfinite(target):
                raise ValueError("width_m must be finite")
        else:
            raise ValueError("gripper action must be open, close, or move")
        if not profile.closed_width_m <= target <= profile.open_width_m:
            raise ValueError(
                f"{arm} gripper target must be in "
                f"{profile.closed_width_m:.4f}..{profile.open_width_m:.4f} m"
            )
        return profile, target

    def validate_request(self, arm: str, action: str, width_m=None) -> None:
        self._armed()
        self._target(arm, action, width_m)
        if self.commands_used >= self.settings.max_commands_per_arming:
            raise PermissionError("gripper command budget is exhausted; disarm and inspect")

    def execute(self, arm: str, action: str, width_m=None) -> dict:
        self.validate_request(arm, action, width_m)
        lease = self._armed()
        profile, target = self._target(arm, action, width_m)
        runner = self.runner_factory(
            self.reader,
            profile.runner_settings,
            sleeper=self.sleeper,
        )
        result = runner.run(
            arm=arm,
            target_m=target,
            execute=True,
            authorization_guard=lambda: self._require_same_lease(lease["since"]),
        )
        if result.get("gripper_command_attempted") is True:
            self.commands_used += 1
        result.update(
            {
                "kind": "piper_agent_gripper_motion",
                "agent_tool_exposed": True,
                "action": action,
                "authorization": {
                    "lease_since": lease["since"],
                    "lease_until": lease.get("until"),
                    "lease_by": lease.get("by"),
                },
                "commissioning": profile.public(),
            }
        )
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = self.root / "piper-motion" / (
            f"agent_gripper_{arm}_{action}_{stamp}_{self.commands_used:02d}.json"
        )
        write_private_json(path, result, directory_mode=0o700)
        self.last_action = {
            "arm": arm,
            "action": action,
            "target_width_m": target,
            "final_width_m": result.get("final_width_m"),
            "status": result.get("status"),
            "receipt": str(path),
        }
        if result.get("status") != "passed":
            reasons = "; ".join(result.get("failure_reasons", ["gripper motion failed"]))
            raise AgentMotionStopped(
                f"Piper gripper motion stopped: {reasons}; receipt: {path}",
                control_certain=result.get("failure_restore") is not None,
            )
        return {
            "executed": True,
            **self.last_action,
            "commands_remaining": max(
                0, self.settings.max_commands_per_arming - self.commands_used
            ),
        }

    def status(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is True:
            try:
                self._armed()
            except PermissionError:
                pass
        return {
            **self.settings.public(),
            "arming": state,
            "commands_used": self.commands_used,
            "commands_remaining": max(
                0, self.settings.max_commands_per_arming - self.commands_used
            ),
            "last_action": self.last_action,
            "updated_at": _utc_now(),
        }
