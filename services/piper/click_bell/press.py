"""Evidence-bound guarded ClickBell press for the RAX Agent."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import time
import uuid

from services import arming
from services.json_io import write_private_json
from services.piper.safety import DEFAULT_MAX_JOINT_STEP_RAD

from ..agent_motion import AgentMotionStopped
from ..contact_motion import ContactStopSettings, PiperGuardedPressExecutor
from ..episode_recording import EpisodeRecordingSettings, TaskEpisodeRecorder
from .evidence import MOTION_CONFIG_KEYS, read_json


HARD_MAX_SPEED_PERCENT = 2
HARD_MAX_CONTACT_TRAVEL_M = 0.005
HARD_MAX_WAYPOINT_TIMEOUT_S = 5.0
HARD_MAX_SETTLE_TOLERANCE_RAD = 0.01


@dataclass(frozen=True)
class ClickBellPressSettings:
    arm: str
    effort_delta_nm: float
    speed_percent: int
    waypoint_timeout_s: float
    settle_tolerance_rad: float
    feedback_max_age_s: float
    joint_margin_rad: float
    max_joint_step_rad: float
    dynamic_workspace_check: bool
    plan_path: Path
    receipt_path: Path
    prior_stage_path: Path
    plan: dict
    prior_stage: dict
    evidence: dict

    @classmethod
    def from_options(cls, value, *, backend_options: dict):
        value = {} if value is None else value
        if not isinstance(value, dict):
            raise ValueError("click_bell guarded_press options must be an object")
        allowed = {
            "enabled",
            "mode",
            "arm",
            "plan",
            "commissioning_receipt",
            "prior_stage_receipt",
            "effort_delta_nm",
            "speed_percent",
            "waypoint_timeout_s",
            "settle_tolerance_rad",
            "dynamic_workspace_check",
        }
        unknown = set(value) - allowed
        if unknown:
            raise ValueError(f"unknown click_bell guarded_press options: {sorted(unknown)}")
        enabled = value.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("click_bell guarded_press enabled must be true or false")
        if not enabled:
            return None
        if value.get("mode") != "guarded_press":
            raise ValueError("click_bell guarded_press mode must be guarded_press")
        try:
            plan_path = Path(value["plan"]).expanduser().resolve()
            receipt_path = Path(value["commissioning_receipt"]).expanduser().resolve()
            prior_path = Path(value["prior_stage_receipt"]).expanduser().resolve()
        except (KeyError, TypeError) as exc:
            raise ValueError(
                "enabled click_bell guarded_press requires plan, commissioning_receipt, "
                "and prior_stage_receipt"
            ) from exc

        plan, plan_raw = read_json(plan_path, "ClickBell plan")
        receipt, receipt_raw = read_json(receipt_path, "guarded-press commissioning receipt")
        prior_stage, prior_raw = read_json(prior_path, "5 mm no-contact receipt")
        arm = value.get("arm", "right")
        effort_delta = float(value.get("effort_delta_nm", 1.0))
        speed = value.get("speed_percent", 2)
        timeout = float(value.get("waypoint_timeout_s", 5.0))
        tolerance = float(value.get("settle_tolerance_rad", 0.01))
        dynamic_workspace_check = value.get("dynamic_workspace_check", True)
        if arm not in {"left", "right"}:
            raise ValueError("click_bell guarded press arm must be left or right")
        if not math.isfinite(effort_delta) or effort_delta <= 0:
            raise ValueError("click_bell guarded press effort_delta_nm must be positive")
        if type(speed) is not int or not 1 <= speed <= HARD_MAX_SPEED_PERCENT:
            raise ValueError(
                f"click_bell guarded press speed must be in 1..{HARD_MAX_SPEED_PERCENT}"
            )
        if not 1.0 <= timeout <= HARD_MAX_WAYPOINT_TIMEOUT_S:
            raise ValueError(
                "click_bell guarded press waypoint timeout must be in "
                f"1..{HARD_MAX_WAYPOINT_TIMEOUT_S} s"
            )
        if not 0.0005 <= tolerance <= HARD_MAX_SETTLE_TOLERANCE_RAD:
            raise ValueError(
                "click_bell guarded press settle tolerance must be in "
                f"0.0005..{HARD_MAX_SETTLE_TOLERANCE_RAD} rad"
            )
        if type(dynamic_workspace_check) is not bool:
            raise ValueError(
                "click_bell guarded press dynamic_workspace_check must be true or false"
            )
        cls._validate_evidence(
            plan,
            receipt,
            prior_stage,
            plan_path=plan_path,
            prior_path=prior_path,
            arm=arm,
            effort_delta=effort_delta,
            speed=speed,
            timeout=timeout,
            tolerance=tolerance,
            backend_options=backend_options,
        )
        return cls(
            arm=arm,
            effort_delta_nm=effort_delta,
            speed_percent=speed,
            waypoint_timeout_s=timeout,
            settle_tolerance_rad=tolerance,
            feedback_max_age_s=float(backend_options.get("feedback_max_age_s", 0.25)),
            joint_margin_rad=float(backend_options.get("joint_limit_margin_rad", 0.02)),
            max_joint_step_rad=float(
                backend_options.get("max_joint_step_rad", DEFAULT_MAX_JOINT_STEP_RAD)
            ),
            dynamic_workspace_check=dynamic_workspace_check,
            plan_path=plan_path,
            receipt_path=receipt_path,
            prior_stage_path=prior_path,
            plan=plan,
            prior_stage=prior_stage,
            evidence={
                "plan": {"path": str(plan_path), "sha256": hashlib.sha256(plan_raw).hexdigest()},
                "commissioning_receipt": {
                    "path": str(receipt_path),
                    "sha256": hashlib.sha256(receipt_raw).hexdigest(),
                },
                "prior_stage_receipt": {
                    "path": str(prior_path),
                    "sha256": hashlib.sha256(prior_raw).hexdigest(),
                },
                "frame_id": plan.get("frame_id"),
            },
        )

    @staticmethod
    def _validate_evidence(
        plan,
        receipt,
        prior_stage,
        *,
        plan_path,
        prior_path,
        arm,
        effort_delta,
        speed,
        timeout,
        tolerance,
        backend_options,
    ) -> None:
        stages = list(plan.get("stages", ()))
        if (
            plan.get("kind") != "click_bell_readonly_press_path"
            or plan.get("status") != "passed"
            or plan.get("geometry_admissible") is not True
            or plan.get("arm") not in (None, arm)
            or len(stages) < 2
        ):
            raise ValueError("guarded press requires a passed admitted plan for the selected arm")
        contact_travel = float(stages[-2].get("approach_distance_m", math.nan))
        if not 0 < contact_travel <= HARD_MAX_CONTACT_TRAVEL_M:
            raise ValueError("guarded press contact travel exceeds the 5 mm hard cap")
        approach_waypoints = sum(
            len(stage.get("plan", {}).get("waypoints", ())) for stage in stages[:-1]
        )
        contact_waypoints = len(stages[-1].get("plan", {}).get("waypoints", ()))
        summary = receipt.get("plan_summary", {})
        if (
            receipt.get("kind") != "piper_operator_guarded_press"
            or receipt.get("status") not in {"passed", "contact_unverified"}
            or receipt.get("arm") != arm
            or receipt.get("agent_tool_exposed") is not False
            or receipt.get("motion_command_attempted") is not True
            or receipt.get("retreat_attempted") is not True
            or receipt.get("retreat", {}).get("status") != "passed"
            or summary.get("frame_id") != plan.get("frame_id")
            or summary.get("approach_waypoints") != approach_waypoints
            or summary.get("contact_waypoints") != contact_waypoints
            or not math.isclose(
                float(summary.get("contact_travel_m", math.nan)),
                contact_travel,
                abs_tol=1e-6,
            )
            or float(receipt.get("initial_gripper_width_m", math.inf)) > 0.005
            or float(receipt.get("start_state_error_rad", math.inf)) > 0.01
        ):
            raise ValueError("guarded-press commissioning receipt does not admit this motion")
        try:
            if Path(receipt["press_plan_path"]).expanduser().resolve() != plan_path:
                raise ValueError("guarded-press receipt points to a different plan")
            prior_evidence = receipt["prior_stage_evidence"]
            if Path(prior_evidence["path"]).expanduser().resolve() != prior_path:
                raise ValueError("guarded-press receipt points to a different prior stage")
            if hashlib.sha256(prior_path.read_bytes()).hexdigest() != prior_evidence["sha256"]:
                raise ValueError("5 mm no-contact evidence hash does not match")
        except (KeyError, TypeError) as exc:
            raise ValueError("guarded-press receipt has incomplete evidence paths") from exc

        prior_summary = prior_stage.get("plan_summary", {})
        if (
            prior_stage.get("kind") != "piper_operator_no_contact_approach"
            or prior_stage.get("status") != "passed"
            or prior_stage.get("arm") != arm
            or prior_stage.get("retreat", {}).get("status") != "passed"
            or prior_summary.get("frame_id") != plan.get("frame_id")
            or not math.isclose(
                float(prior_summary.get("approach_distance_m", math.nan)),
                contact_travel,
                abs_tol=1e-6,
            )
            or prior_summary.get("waypoints") != approach_waypoints
        ):
            raise ValueError("guarded press requires the passed 5 mm no-contact receipt")

        commissioned = receipt.get("contact_settings", {})
        if (
            not math.isclose(
                effort_delta,
                float(commissioned.get("effort_delta_nm", math.nan)),
                abs_tol=1e-9,
            )
            or speed > int(commissioned.get("speed_percent", 0))
            or timeout > float(commissioned.get("waypoint_timeout_s", 0.0))
        ):
            raise ValueError("guarded press settings are not admitted by commissioning evidence")
        try:
            admitted_path = Path(receipt["backend_options_path"]).expanduser().resolve()
            admitted, _ = read_json(admitted_path, "commissioned backend options")
        except (KeyError, TypeError) as exc:
            raise ValueError("guarded-press receipt has no valid backend options path") from exc
        mismatched = [
            key for key in MOTION_CONFIG_KEYS if admitted.get(key) != backend_options.get(key)
        ]
        if mismatched:
            raise ValueError(
                "current Piper motion configuration differs from commissioning evidence: "
                + ", ".join(mismatched)
            )

    def public(self) -> dict:
        return {
            "enabled": True,
            "mode": "guarded_press",
            "arm": self.arm,
            "speed_percent": self.speed_percent,
            "settle_tolerance_rad": self.settle_tolerance_rad,
            "runtime_max_joint_step_rad": self.max_joint_step_rad,
            "max_contact_travel_m": HARD_MAX_CONTACT_TRAVEL_M,
            "one_call_per_arming": True,
            "retreat_required": True,
            "visual_confirmation_required": True,
            "dynamic_workspace_check": self.dynamic_workspace_check,
            "evidence": self.evidence,
        }


class ClickBellPressController:
    def __init__(
        self,
        config_root: Path,
        reader,
        cameras,
        camera_names,
        settings: ClickBellPressSettings,
        recording: EpisodeRecordingSettings,
        *,
        camera_timeout_ms: int = 3000,
        sleeper=time.sleep,
        arming_reader=arming.read,
        recorder_factory=None,
        task_recorder=None,
    ):
        self.root = Path(config_root)
        self.reader = reader
        self.cameras = cameras
        self.camera_names = tuple(camera_names)
        self.settings = settings
        self.recording = recording
        self.camera_timeout_ms = camera_timeout_ms
        self.sleeper = sleeper
        self.arming_reader = arming_reader
        self.owns_task_recorder = task_recorder is None
        self.task_recorder = task_recorder or TaskEpisodeRecorder(
            self.root,
            cameras,
            camera_names,
            recording,
            camera_timeout_ms=camera_timeout_ms,
            **({} if recorder_factory is None else {"recorder_factory": recorder_factory}),
        )
        self.lease_since = None
        self.used = False
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
            self.used = False
        return state

    def _require_same_lease(self, since: float) -> None:
        state = self._armed_lease()
        if state["since"] != since:
            raise PermissionError("operator arming lease changed during ClickBell press")

    def validate_request(self) -> None:
        self._armed_lease()
        if self.used:
            raise PermissionError("ClickBell press budget is exhausted; disarm and inspect")

    def status(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is True:
            try:
                self._armed_lease()
            except PermissionError:
                pass
        return {
            **self.settings.public(),
            "recording": {
                "enabled": True,
                "fps": self.recording.fps,
                "save_depth": self.recording.save_depth,
                "cameras": list(self.camera_names),
            },
            "arming": state,
            "calls_used": int(self.used),
            "calls_remaining": 0 if self.used else 1,
            "last_motion": self.last_motion,
        }

    def _save(self, receipt: dict) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        name = f"agent_click_bell_press_{stamp}_{receipt['press_id']}.json"
        path = self.root / "piper-motion" / name
        write_private_json(path, receipt, directory_mode=0o700)
        return path

    def execute(self, plan: dict | None = None) -> dict:
        self.validate_request()
        selected_plan = self.settings.plan if plan is None else plan
        dynamic = plan is not None
        if dynamic and (
            selected_plan.get("dynamic") is not True
            or selected_plan.get("execution_admissible") is not True
            or selected_plan.get("arm") != self.settings.arm
            or not isinstance(selected_plan.get("plan_id"), str)
        ):
            raise PermissionError("guarded press requires an admitted dynamic plan for this arm")
        lease = self._armed_lease()
        lease_since = lease["since"]
        press_id = uuid.uuid4().hex[:12]
        try:
            if self.owns_task_recorder:
                self.task_recorder.start()
            elif self.task_recorder.state != "recording":
                raise RuntimeError(
                    f"ClickBell task recorder is {self.task_recorder.state} before press"
                )
            self.task_recorder.mark(
                "guarded_press_started",
                arm=self.settings.arm,
                plan_id=selected_plan.get("plan_id"),
                dynamic=dynamic,
            )
        except BaseException as exc:
            raise AgentMotionStopped(
                "ClickBell task recording preflight failed before motion: "
                f"{type(exc).__name__}: {exc}",
                control_certain=True,
            ) from exc
        receipt = {
            "schema_version": 1,
            "kind": "piper_agent_click_bell_guarded_press",
            "status": "failed",
            "motion_command_attempted": False,
            "retreat_attempted": False,
        }
        settings = ContactStopSettings(
            effort_delta_nm=self.settings.effort_delta_nm,
            speed_percent=self.settings.speed_percent,
            feedback_max_age_s=self.settings.feedback_max_age_s,
            waypoint_timeout_s=self.settings.waypoint_timeout_s,
            settle_tolerance_rad=self.settings.settle_tolerance_rad,
            max_contact_travel_m=HARD_MAX_CONTACT_TRAVEL_M,
            joint_margin_rad=self.settings.joint_margin_rad,
            max_joint_step_rad=self.settings.max_joint_step_rad,
        )
        try:
            executor = PiperGuardedPressExecutor(
                self.reader,
                settings,
                sleeper=self.sleeper,
                sample_observer=self.task_recorder.record_robot_sample,
            )
            receipt = executor.run(
                selected_plan,
                arm=self.settings.arm,
                operator_lease_id=f"agent:{lease_since}:click_bell_press",
                execute=True,
                authorization_guard=lambda: self._require_same_lease(lease_since),
                prior_stage_receipt=None if dynamic else self.settings.prior_stage,
                require_prior_stage_evidence=not dynamic,
            )
        except BaseException as exc:
            receipt.setdefault("failure_reasons", []).append(
                f"{type(exc).__name__}: {exc}"
            )
        admitted_status = receipt.get("status") in {"passed", "contact_unverified"}
        retreat_passed = receipt.get("retreat", {}).get("status") == "passed"
        self.task_recorder.mark(
            "guarded_press_finished",
            arm=self.settings.arm,
            status=receipt.get("status"),
            retreat_status=receipt.get("retreat", {}).get("status"),
        )
        if self.owns_task_recorder or not (admitted_status and retreat_passed):
            recording = self.task_recorder.stop(
                reason=(
                    "standalone_guarded_press_completed"
                    if admitted_status and retreat_passed
                    else "guarded_press_failed"
                )
            )
        else:
            recording = self.task_recorder.reference()
        receipt["kind"] = "piper_agent_click_bell_guarded_press"
        receipt["agent_tool_exposed"] = True
        receipt["press_id"] = press_id
        receipt["authorization"] = {
            "lease_since": lease_since,
            "lease_until": lease.get("until"),
            "lease_by": lease.get("by"),
        }
        receipt["admission"] = {
            **self.settings.public(),
            "plan_mode": "dynamic" if dynamic else "commissioned_fixed",
            "plan_id": selected_plan.get("plan_id"),
            "source_frame_id": selected_plan.get("frame_id"),
        }
        receipt["recording"] = recording
        receipt["visual_confirmation"] = {"status": "pending"}
        if receipt.get("motion_command_attempted") is True:
            self.used = True
        path = self._save(receipt)
        self.last_motion = {
            "press_id": press_id,
            "plan_id": selected_plan.get("plan_id"),
            "status": receipt.get("status"),
            "arm": self.settings.arm,
            "authorization": receipt["authorization"],
            "receipt": str(path),
            "retreat_status": receipt.get("retreat", {}).get("status"),
            "recording": recording.get("manifest"),
            "finished_at": receipt.get("finished_at"),
        }
        expected_recording_status = "passed" if self.owns_task_recorder else "recording"
        recording_passed = recording.get("status") == expected_recording_status
        if not (admitted_status and retreat_passed and recording_passed):
            reasons = list(receipt.get("failure_reasons", ()))
            if not retreat_passed:
                reasons.append("reverse retreat was not confirmed")
            if not recording_passed:
                reasons.append("episode recording was incomplete")
            certain = retreat_passed or (
                receipt.get("motion_command_attempted") is not True
                and recording.get("camera_samples", 0) > 0
            )
            raise AgentMotionStopped(
                f"Piper ClickBell press stopped: {'; '.join(reasons)}; receipt: {path}",
                control_certain=certain,
            )
        return {
            "executed": True,
            "press_id": press_id,
            "plan_id": selected_plan.get("plan_id"),
            "arm": self.settings.arm,
            "contact_signal_detected": receipt.get("contact_event") is not None,
            "retreat_status": "passed",
            "visual_confirmation_required": True,
            "visual_confirmation_status": "pending",
            "recording": recording,
            "calls_remaining": 0,
            "receipt": str(path),
        }
