"""Evidence-bound ClickBell gripper actions for the RAX Agent."""

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
from .episode_recording import EpisodeRecordingSettings, MotionEpisodeRecorder
from .gripper_characterization import GripperCycleSettings, PiperGripperSetRunner


HARD_CLOSED_WIDTH_M = 0.0
HARD_MAX_OPEN_WIDTH_M = 0.10
HARD_MAX_STEP_M = 0.01
HARD_MAX_EFFORT = 500
ACTIONS = frozenset({"close", "open"})


def _read_receipt(path: Path) -> tuple[dict, bytes]:
    try:
        raw = path.read_bytes()
        value = json.loads(raw)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read gripper commissioning receipt {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError("gripper commissioning receipt must contain an object")
    return value, raw


@dataclass(frozen=True)
class ClickBellGripperSettings:
    arm: str
    open_width_m: float
    closed_width_m: float
    runner_settings: GripperCycleSettings
    receipt_path: Path
    evidence: dict

    @classmethod
    def from_options(cls, value, *, backend_options: dict):
        value = {} if value is None else value
        if not isinstance(value, dict):
            raise ValueError("click_bell gripper options must be an object")
        unknown = set(value) - {"enabled", "arm", "commissioning_receipt"}
        if unknown:
            raise ValueError(f"unknown click_bell gripper options: {sorted(unknown)}")
        enabled = value.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("click_bell gripper enabled must be true or false")
        if not enabled:
            return None
        arm = value.get("arm", "right")
        if arm != "right":
            raise ValueError("the commissioned ClickBell gripper tool only admits the right arm")
        try:
            receipt_path = Path(value["commissioning_receipt"]).expanduser().resolve()
        except (KeyError, TypeError) as exc:
            raise ValueError(
                "enabled click_bell gripper requires commissioning_receipt"
            ) from exc
        receipt, raw = _read_receipt(receipt_path)
        settings = receipt.get("settings", {})
        try:
            runner_settings = GripperCycleSettings(**settings)
            open_width = float(receipt["initial_width_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("gripper commissioning receipt has invalid settings") from exc
        cls._validate_receipt(
            receipt,
            backend_options=backend_options,
            arm=arm,
            open_width=open_width,
            settings=runner_settings,
        )
        return cls(
            arm=arm,
            open_width_m=open_width,
            closed_width_m=HARD_CLOSED_WIDTH_M,
            runner_settings=runner_settings,
            receipt_path=receipt_path,
            evidence={
                "commissioning_receipt": {
                    "path": str(receipt_path),
                    "sha256": hashlib.sha256(raw).hexdigest(),
                },
                "open_width_m": open_width,
                "closed_width_m": HARD_CLOSED_WIDTH_M,
            },
        )

    @staticmethod
    def _validate_receipt(
        receipt,
        *,
        backend_options,
        arm,
        open_width,
        settings,
    ) -> None:
        close = receipt.get("close", ())
        restore = receipt.get("restore", ())
        if (
            receipt.get("kind") != "piper_operator_gripper_cycle"
            or receipt.get("status") != "passed"
            or receipt.get("arm") != arm
            or receipt.get("agent_tool_exposed") is not False
            or receipt.get("arm_motion_commanded") is not False
            or receipt.get("gripper_command_attempted") is not True
            or receipt.get("restored") is not True
            or not close
            or not restore
        ):
            raise ValueError("gripper commissioning receipt does not admit Agent use")
        if not 0 < open_width <= HARD_MAX_OPEN_WIDTH_M:
            raise ValueError("commissioned open gripper width is outside the hard limit")
        if (
            settings.step_m > HARD_MAX_STEP_M
            or settings.effort > HARD_MAX_EFFORT
            or settings.target_tolerance_m > 0.003
            or settings.max_arm_joint_drift_rad > 0.002
        ):
            raise ValueError("commissioned gripper settings exceed Agent hard limits")
        close_last = close[-1]
        restore_last = restore[-1]
        if (
            not math.isclose(
                float(close_last.get("command", {}).get("opening_m", math.nan)),
                HARD_CLOSED_WIDTH_M,
                abs_tol=1e-9,
            )
            or close_last.get("feedback", {}).get("reached") is not True
            or not math.isclose(
                float(restore_last.get("command", {}).get("opening_m", math.nan)),
                open_width,
                abs_tol=1e-6,
            )
            or restore_last.get("feedback", {}).get("reached") is not True
            or abs(float(receipt.get("final_width_m", math.inf)) - open_width)
            > settings.target_tolerance_m
        ):
            raise ValueError("gripper receipt did not prove close and restore targets")
        try:
            commissioned_backend = Path(receipt["backend_options_path"]).resolve()
            commissioned = json.loads(commissioned_backend.read_text())
        except (KeyError, OSError, json.JSONDecodeError, TypeError) as exc:
            raise ValueError("gripper receipt has no readable backend configuration") from exc
        expected_arms = {
            side: (backend_options.get("arms") or {}).get(side, {}).get("can_name")
            for side in ("left", "right")
        }
        observed_arms = {
            side: (commissioned.get("arms") or {}).get(side, {}).get("can_name")
            for side in ("left", "right")
        }
        if expected_arms != observed_arms or any(value is None for value in expected_arms.values()):
            raise ValueError("gripper receipt was commissioned on different CAN interfaces")

    def public(self) -> dict:
        return {
            "enabled": True,
            "arm": self.arm,
            "actions": sorted(ACTIONS),
            "open_width_m": self.open_width_m,
            "closed_width_m": self.closed_width_m,
            "step_m": self.runner_settings.step_m,
            "effort": self.runner_settings.effort,
            "required_sequence": ["close", "guarded_press", "open"],
            "evidence": self.evidence,
        }


class ClickBellGripperController:
    """Allow one commissioned close and one post-retreat open per arming lease."""

    def __init__(
        self,
        config_root: Path,
        reader,
        cameras,
        camera_names,
        settings: ClickBellGripperSettings,
        recording: EpisodeRecordingSettings,
        press_controller,
        *,
        camera_timeout_ms: int = 3000,
        sleeper=time.sleep,
        arming_reader=arming.read,
        recorder_factory=MotionEpisodeRecorder,
        runner_factory=PiperGripperSetRunner,
    ):
        self.root = Path(config_root)
        self.reader = reader
        self.cameras = cameras
        self.camera_names = tuple(camera_names)
        self.settings = settings
        self.recording = recording
        self.press_controller = press_controller
        self.camera_timeout_ms = camera_timeout_ms
        self.sleeper = sleeper
        self.arming_reader = arming_reader
        self.recorder_factory = recorder_factory
        self.runner_factory = runner_factory
        self.lease_since = None
        self.phase = "awaiting_close"
        self.actions = []

    def _armed_lease(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is not True:
            raise PermissionError(
                f"motion is not armed ({state.get('reason', 'unknown reason')})"
            )
        since = state.get("since")
        if not isinstance(since, (int, float)) or not math.isfinite(since):
            raise PermissionError("operator arming lease has no valid creation time")
        if self.lease_since != since:
            self.lease_since = since
            self.phase = "awaiting_close"
            self.actions = []
        return state

    def _require_same_lease(self, since: float) -> None:
        state = self._armed_lease()
        if state["since"] != since:
            raise PermissionError("operator arming lease changed during gripper action")

    def _press_for_lease(self, since: float) -> dict:
        motion = self.press_controller.last_motion
        authorization = motion.get("authorization", {}) if isinstance(motion, dict) else {}
        if (
            not isinstance(motion, dict)
            or authorization.get("lease_since") != since
            or motion.get("status") not in {"passed", "contact_unverified"}
            or motion.get("retreat_status") != "passed"
        ):
            raise PermissionError("gripper can open only after the commissioned press retreated")
        return motion

    def validate_request(self, action: str) -> None:
        if action not in ACTIONS:
            raise ValueError("ClickBell gripper action must be close or open")
        lease = self._armed_lease()
        if action == "close" and self.phase != "awaiting_close":
            raise PermissionError("ClickBell gripper close budget is exhausted")
        if action == "open":
            if self.phase != "closed":
                raise PermissionError("ClickBell gripper must close before it can open")
            self._press_for_lease(lease["since"])
        runner = self.runner_factory(
            self.reader, self.settings.runner_settings, sleeper=self.sleeper
        )
        current = runner.current_state(self.settings.arm)["width_m"]
        expected = (
            self.settings.open_width_m
            if action == "close"
            else self.settings.closed_width_m
        )
        if abs(current - expected) > self.settings.runner_settings.target_tolerance_m:
            raise PermissionError(
                f"ClickBell gripper {action} requires start width {expected:.4f} m; "
                f"measured {current:.4f} m"
            )

    def require_closed_for_press(self) -> None:
        self._armed_lease()
        if self.phase != "closed":
            raise PermissionError("Agent must close the ClickBell gripper before pressing")

    def status(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is True:
            try:
                self._armed_lease()
            except PermissionError:
                pass
        return {
            **self.settings.public(),
            "phase": self.phase,
            "arming": state,
            "actions": list(self.actions),
            "calls_remaining": {
                "close": int(self.phase == "awaiting_close"),
                "open": int(self.phase == "closed"),
            },
        }

    def _save(self, receipt: dict) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = self.root / "piper-motion" / (
            f"agent_click_bell_gripper_{receipt['action']}_{stamp}.json"
        )
        write_private_json(path, receipt, directory_mode=0o700)
        return path

    def execute(self, action: str) -> dict:
        self.validate_request(action)
        lease = self._armed_lease()
        since = lease["since"]
        target = (
            self.settings.closed_width_m
            if action == "close"
            else self.settings.open_width_m
        )
        press = None if action == "close" else self._press_for_lease(since)
        recorder = self.recorder_factory(
            self.root,
            self.cameras,
            self.camera_names,
            self.recording,
            camera_timeout_ms=self.camera_timeout_ms,
        )
        try:
            recorder.start()
        except BaseException as exc:
            raise AgentMotionStopped(
                f"ClickBell gripper recording failed before motion: {type(exc).__name__}: {exc}",
                control_certain=True,
            ) from exc
        runner = self.runner_factory(
            self.reader,
            self.settings.runner_settings,
            sleeper=self.sleeper,
            sample_observer=recorder.record_robot_sample,
        )
        result = runner.run(
            arm=self.settings.arm,
            target_m=target,
            execute=True,
            authorization_guard=lambda: self._require_same_lease(since),
        )
        recording = recorder.stop()
        result.update(
            {
                "kind": "piper_agent_click_bell_gripper",
                "agent_tool_exposed": True,
                "action": action,
                "authorization": {
                    "lease_since": since,
                    "lease_until": lease.get("until"),
                    "lease_by": lease.get("by"),
                },
                "admission": self.settings.public(),
                "recording": recording,
                "press_id": None if press is None else press["press_id"],
            }
        )
        path = self._save(result)
        action_result = {
            "action": action,
            "status": result.get("status"),
            "target_width_m": target,
            "final_width_m": result.get("final_width_m"),
            "press_id": result.get("press_id"),
            "receipt": str(path),
            "recording": recording.get("manifest"),
        }
        if result.get("status") == "passed":
            self.phase = "closed" if action == "close" else "opened"
            self.actions.append(action_result)
        if result.get("status") != "passed" or recording.get("status") != "passed":
            reasons = list(result.get("failure_reasons", ()))
            if recording.get("status") != "passed":
                reasons.append("gripper episode recording was incomplete")
            raise AgentMotionStopped(
                f"Piper ClickBell gripper {action} stopped: {'; '.join(reasons)}; "
                f"receipt: {path}",
                control_certain=result.get("status") == "passed",
            )
        return {
            "executed": True,
            **action_result,
            "phase": self.phase,
            "recording_detail": recording,
        }

    def completed_task_evidence(self, press_id: str) -> dict:
        if self.phase != "opened" or [item["action"] for item in self.actions] != [
            "close",
            "open",
        ]:
            raise ValueError("ClickBell gripper close/open sequence is incomplete")
        if self.actions[-1].get("press_id") != press_id:
            raise ValueError("gripper open is not bound to this guarded press")
        return {
            "actions": list(self.actions),
            "recordings": [item["recording"] for item in self.actions],
        }
