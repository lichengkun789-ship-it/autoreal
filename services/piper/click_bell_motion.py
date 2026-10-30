"""Evidence-bound ClickBell approach preview for the RAX Agent.

The Agent can replay only one operator-commissioned no-contact path per arming
lease.  This controller shares the Piper backend's reader and cannot alter the
arm, path, speed, clearance, or waypoint count.
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

from .agent_motion import AgentMotionStopped
from .contact_motion import ContactStopSettings, PiperGuardedPressExecutor


HARD_MAX_SPEED_PERCENT = 2
HARD_MAX_WAYPOINT_TIMEOUT_S = 5.0
HARD_MAX_SETTLE_TOLERANCE_RAD = 0.002
MOTION_CONFIG_KEYS = (
    "arms",
    "joint_limit_margin_rad",
    "max_joint_step_rad",
    "workspace_bounds",
    "link_min_z_m",
)


def _read_json(path: Path, label: str) -> tuple[dict, bytes]:
    try:
        raw = path.read_bytes()
        value = json.loads(raw)
    except (OSError, ValueError) as exc:
        raise ValueError(f"cannot read {label} {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return value, raw


def _same_joints(first, second) -> bool:
    if (
        not isinstance(first, list)
        or not isinstance(second, list)
        or len(first) != 6
        or len(second) != 6
    ):
        return False
    try:
        return all(
            math.isclose(float(a), float(b), abs_tol=1e-9)
            for a, b in zip(first, second)
        )
    except (TypeError, ValueError):
        return False


@dataclass(frozen=True)
class ClickBellPreviewSettings:
    arm: str
    approach_distance_m: float
    waypoints: int
    speed_percent: int
    waypoint_timeout_s: float
    settle_tolerance_rad: float
    feedback_max_age_s: float
    joint_margin_rad: float
    max_joint_step_rad: float
    plan_path: Path
    receipt_path: Path
    plan: dict
    evidence: dict

    @classmethod
    def from_options(cls, value, *, backend_options: dict) -> "ClickBellPreviewSettings | None":
        value = {} if value is None else value
        if not isinstance(value, dict):
            raise ValueError("click_bell task motion options must be an object")
        allowed = {
            "enabled",
            "mode",
            "arm",
            "plan",
            "commissioning_receipt",
            "approach_distance_m",
            "waypoints",
            "speed_percent",
            "waypoint_timeout_s",
            "settle_tolerance_rad",
        }
        unknown = set(value) - allowed
        if unknown:
            raise ValueError(f"unknown click_bell motion options: {sorted(unknown)}")
        enabled = value.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("click_bell motion enabled must be true or false")
        if not enabled:
            return None
        if value.get("mode") != "approach_preview":
            raise ValueError("click_bell motion mode must be approach_preview")

        arm = value.get("arm", "right")
        distance = float(value.get("approach_distance_m", 0.06))
        waypoints = value.get("waypoints", 64)
        speed = value.get("speed_percent", 2)
        timeout = float(value.get("waypoint_timeout_s", 5.0))
        tolerance = float(value.get("settle_tolerance_rad", 0.002))
        feedback_max_age = float(backend_options.get("feedback_max_age_s", 0.25))
        joint_margin = float(backend_options.get("joint_limit_margin_rad", 0.02))
        max_joint_step = float(backend_options.get("max_joint_step_rad", 0.05))
        if arm not in {"left", "right"}:
            raise ValueError("click_bell preview arm must be left or right")
        if not math.isfinite(distance) or distance < 0.005:
            raise ValueError("click_bell preview must remain at least 0.005 m from contact")
        if type(waypoints) is not int or not 1 <= waypoints <= 128:
            raise ValueError("click_bell preview waypoints must be in 1..128")
        if type(speed) is not int or not 1 <= speed <= HARD_MAX_SPEED_PERCENT:
            raise ValueError(f"click_bell preview speed must be in 1..{HARD_MAX_SPEED_PERCENT}")
        if not 1.0 <= timeout <= HARD_MAX_WAYPOINT_TIMEOUT_S:
            raise ValueError(
                f"click_bell preview waypoint timeout must be in 1..{HARD_MAX_WAYPOINT_TIMEOUT_S} s"
            )
        if not 0.0005 <= tolerance <= HARD_MAX_SETTLE_TOLERANCE_RAD:
            raise ValueError(
                "click_bell preview settle tolerance must be in "
                f"0.0005..{HARD_MAX_SETTLE_TOLERANCE_RAD} rad"
            )

        try:
            plan_path = Path(value["plan"]).expanduser().resolve()
            receipt_path = Path(value["commissioning_receipt"]).expanduser().resolve()
        except (KeyError, TypeError) as exc:
            raise ValueError("enabled click_bell preview requires plan and commissioning_receipt") from exc
        plan, plan_raw = _read_json(plan_path, "ClickBell plan")
        receipt, receipt_raw = _read_json(receipt_path, "ClickBell commissioning receipt")
        cls._validate_evidence(
            plan,
            receipt,
            plan_path=plan_path,
            arm=arm,
            distance=distance,
            waypoints=waypoints,
            speed=speed,
            timeout=timeout,
            tolerance=tolerance,
            backend_options=backend_options,
        )
        return cls(
            arm=arm,
            approach_distance_m=distance,
            waypoints=waypoints,
            speed_percent=speed,
            waypoint_timeout_s=timeout,
            settle_tolerance_rad=tolerance,
            feedback_max_age_s=feedback_max_age,
            joint_margin_rad=joint_margin,
            max_joint_step_rad=max_joint_step,
            plan_path=plan_path,
            receipt_path=receipt_path,
            plan=plan,
            evidence={
                "plan": {"path": str(plan_path), "sha256": hashlib.sha256(plan_raw).hexdigest()},
                "commissioning_receipt": {
                    "path": str(receipt_path),
                    "sha256": hashlib.sha256(receipt_raw).hexdigest(),
                },
                "frame_id": plan.get("frame_id"),
            },
        )

    @staticmethod
    def _validate_evidence(
        plan: dict,
        receipt: dict,
        *,
        plan_path: Path,
        arm: str,
        distance: float,
        waypoints: int,
        speed: int,
        timeout: float,
        tolerance: float,
        backend_options: dict,
    ) -> None:
        summary = receipt.get("plan_summary", {})
        if (
            plan.get("kind") != "click_bell_readonly_press_path"
            or plan.get("status") != "passed"
            or plan.get("geometry_admissible") is not True
            or plan.get("arm") not in (None, arm)
        ):
            raise ValueError("click_bell preview requires a passed admitted plan for the selected arm")
        if (
            receipt.get("kind") != "piper_operator_no_contact_approach"
            or receipt.get("status") != "passed"
            or receipt.get("arm") != arm
            or receipt.get("contact_intended") is not False
            or receipt.get("motion_command_attempted") is not True
            or receipt.get("retreat_attempted") is not True
            or receipt.get("retreat", {}).get("status") != "passed"
            or summary.get("frame_id") != plan.get("frame_id")
            or not math.isclose(
                float(summary.get("approach_distance_m", math.nan)),
                distance,
                abs_tol=1e-6,
            )
            or summary.get("waypoints") != waypoints
            or float(receipt.get("final_start_error_rad", math.inf)) > 0.01
        ):
            raise ValueError("click_bell commissioning receipt does not admit this preview")
        try:
            if Path(receipt["press_plan_path"]).expanduser().resolve() != plan_path:
                raise ValueError("commissioning receipt points to a different ClickBell plan")
        except (KeyError, TypeError) as exc:
            raise ValueError("commissioning receipt has no valid press plan path") from exc

        stages = list(plan.get("stages", ()))
        selected = next(
            (
                index
                for index, stage in enumerate(stages)
                if math.isclose(
                    float(stage.get("approach_distance_m", math.nan)),
                    distance,
                    abs_tol=1e-6,
                )
            ),
            None,
        )
        if selected is None:
            raise ValueError("configured preview clearance is absent from the plan")
        planned = [
            row.get("joints_rad")
            for stage in stages[: selected + 1]
            for row in stage.get("plan", {}).get("waypoints", ())
        ][:waypoints]
        executed = [
            row.get("target_joints_rad")
            for row in receipt.get("approach", {}).get("waypoints", ())
        ]
        if len(planned) != waypoints or len(executed) != waypoints or any(
            not _same_joints(expected, observed) for expected, observed in zip(planned, executed)
        ):
            raise ValueError("commissioning receipt did not execute the configured preview waypoints")

        commissioned = receipt.get("motion_settings", {})
        if (
            speed > int(commissioned.get("speed_percent", 0))
            or timeout > float(commissioned.get("waypoint_timeout_s", 0.0))
            or tolerance > float(commissioned.get("settle_tolerance_rad", 0.0))
        ):
            raise ValueError("preview settings are looser than the commissioned motion settings")
        try:
            admitted_path = Path(receipt["backend_options_path"]).expanduser().resolve()
            admitted, _ = _read_json(admitted_path, "commissioned backend options")
        except (KeyError, TypeError) as exc:
            raise ValueError("commissioning receipt has no valid backend options path") from exc
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
            "mode": "approach_preview",
            "arm": self.arm,
            "approach_distance_m": self.approach_distance_m,
            "waypoints": self.waypoints,
            "speed_percent": self.speed_percent,
            "one_call_per_arming": True,
            "contact_intended": False,
            "evidence": self.evidence,
        }


class ClickBellPreviewController:
    def __init__(
        self,
        config_root: Path,
        reader,
        settings: ClickBellPreviewSettings,
        *,
        sleeper=time.sleep,
        arming_reader=arming.read,
    ):
        self.root = Path(config_root)
        self.reader = reader
        self.settings = settings
        self.sleeper = sleeper
        self.arming_reader = arming_reader
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
            raise PermissionError("operator arming lease changed during ClickBell motion")

    def validate_request(self) -> None:
        self._armed_lease()
        if self.used:
            raise PermissionError("ClickBell preview budget is exhausted; disarm and inspect")

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
            "calls_used": int(self.used),
            "calls_remaining": 0 if self.used else 1,
            "last_motion": self.last_motion,
        }

    def _save(self, receipt: dict) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        name = f"agent_click_bell_preview_{stamp}_{uuid.uuid4().hex[:8]}.json"
        path = self.root / "piper-motion" / name
        write_private_json(path, receipt, directory_mode=0o700)
        return path

    def execute(self) -> dict:
        self.validate_request()
        lease = self._armed_lease()
        lease_since = lease["since"]
        settings = ContactStopSettings(
            effort_delta_nm=1.0,
            speed_percent=self.settings.speed_percent,
            feedback_max_age_s=self.settings.feedback_max_age_s,
            waypoint_timeout_s=self.settings.waypoint_timeout_s,
            settle_tolerance_rad=self.settings.settle_tolerance_rad,
            joint_margin_rad=self.settings.joint_margin_rad,
            max_joint_step_rad=self.settings.max_joint_step_rad,
        )
        executor = PiperGuardedPressExecutor(self.reader, settings, sleeper=self.sleeper)
        receipt = executor.run_no_contact(
            self.settings.plan,
            arm=self.settings.arm,
            approach_distance_m=self.settings.approach_distance_m,
            max_waypoints=self.settings.waypoints,
            operator_lease_id=f"agent:{lease_since}:click_bell_preview",
            execute=True,
            authorization_guard=lambda: self._require_same_lease(lease_since),
        )
        receipt["kind"] = "piper_agent_click_bell_approach_preview"
        receipt["agent_tool_exposed"] = True
        receipt["authorization"] = {
            "lease_since": lease_since,
            "lease_until": lease.get("until"),
            "lease_by": lease.get("by"),
        }
        receipt["admission"] = self.settings.public()
        if receipt.get("motion_command_attempted") is True:
            self.used = True
        path = self._save(receipt)
        self.last_motion = {
            "status": receipt.get("status"),
            "arm": self.settings.arm,
            "receipt": str(path),
            "finished_at": receipt.get("finished_at"),
        }
        if receipt.get("status") != "passed":
            certain = receipt.get("motion_command_attempted") is not True or bool(
                receipt.get("failure_hold", {}).get("native_command")
            )
            reasons = "; ".join(receipt.get("failure_reasons", ["ClickBell preview failed"]))
            raise AgentMotionStopped(
                f"Piper ClickBell preview stopped: {reasons}; receipt: {path}",
                control_certain=certain,
            )
        return {
            "executed": True,
            "contact_intended": False,
            "arm": self.settings.arm,
            "approach_distance_m": self.settings.approach_distance_m,
            "approach_waypoints": self.settings.waypoints,
            "retreat_status": receipt.get("retreat", {}).get("status"),
            "final_start_error_rad": receipt.get("final_start_error_rad"),
            "calls_remaining": 0,
            "receipt": str(path),
        }
