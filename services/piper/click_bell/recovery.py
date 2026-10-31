"""Agent-visible recovery of an interrupted ClickBell approach path."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np

from services import arming
from services.json_io import write_private_json
from services.piper.contact_motion import ContactStopSettings, PiperGuardedPressExecutor

from .press import HARD_MAX_CONTACT_TRAVEL_M


RECOVERY_PATH_MATCH_RAD = 0.01


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class ClickBellRecoveryController:
    """Retrace the newest failed dynamic path that matches live feedback."""

    def __init__(
        self,
        root: Path,
        reader,
        press_settings,
        *,
        sleeper=time.sleep,
        arming_reader=arming.read,
    ):
        self.root = Path(root)
        self.reader = reader
        self.press_settings = press_settings
        self.sleeper = sleeper
        self.arming_reader = arming_reader
        self.used_lease = None
        self.last_recovery = None

    def _armed(self) -> dict:
        lease = self.arming_reader(self.root)
        if lease.get("armed") is not True:
            raise PermissionError(lease.get("reason") or "motion is not armed")
        since = lease.get("since")
        if not isinstance(since, (int, float)) or not math.isfinite(since):
            raise PermissionError("operator arming lease has no valid creation time")
        if self.used_lease == since:
            raise PermissionError("ClickBell recovery budget is exhausted for this arming lease")
        return lease

    def _require_same_lease(self, since: float) -> None:
        lease = self.arming_reader(self.root)
        if lease.get("armed") is not True or lease.get("since") != since:
            raise PermissionError(lease.get("reason") or "operator arming lease changed")

    @staticmethod
    def _path(plan: dict) -> list[list[float]]:
        if (
            plan.get("dynamic") is not True
            or plan.get("execution_admissible") is not True
            or plan.get("arm") != "right"
        ):
            raise ValueError("recovery source is not an admitted right-arm dynamic plan")
        start = plan.get("start_joints_rad")
        stages = plan.get("stages")
        if not isinstance(start, list) or len(start) != 6 or not isinstance(stages, list):
            raise ValueError("recovery source plan is incomplete")
        path = [[float(value) for value in start]]
        for stage in stages:
            rows = stage.get("plan", {}).get("waypoints")
            if not isinstance(rows, list) or not rows:
                raise ValueError("recovery source plan has an empty stage")
            path.extend([float(value) for value in row["joints_rad"]] for row in rows)
        return path

    def _live_joints(self) -> np.ndarray:
        joints = np.asarray(
            self.reader.snapshot().get("arms", {}).get("right", {}).get("joints_rad"),
            dtype=np.float64,
        )
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("right-arm recovery requires six live finite joints")
        return joints

    def _candidate(self) -> dict:
        current = self._live_joints()
        candidates = []
        for receipt_path in self.root.parent.glob(
            "*/piper-motion/agent_click_bell_press_*.json"
        ):
            try:
                receipt = json.loads(receipt_path.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            admission = receipt.get("admission", {})
            plan_id = admission.get("plan_id")
            if (
                receipt.get("kind") != "piper_agent_click_bell_guarded_press"
                or receipt.get("arm") != "right"
                or receipt.get("motion_command_attempted") is not True
                or receipt.get("retreat", {}).get("status") == "passed"
                or not isinstance(plan_id, str)
            ):
                continue
            plan_path = receipt_path.parents[1] / "click-bell-plans" / f"{plan_id}.json"
            try:
                plan = json.loads(plan_path.read_text())
                path = self._path(plan)
            except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError):
                continue
            points = np.asarray(path, dtype=np.float64)
            distances = np.max(np.abs(points - current), axis=1)
            anchor_index = int(np.argmin(distances))
            candidates.append(
                {
                    "receipt_path": receipt_path,
                    "plan_path": plan_path,
                    "plan": plan,
                    "path": path,
                    "source_scope": (
                        "current_session"
                        if receipt_path.is_relative_to(self.root)
                        else "prior_session"
                    ),
                    "anchor_index": anchor_index,
                    "anchor_error_rad": float(distances[anchor_index]),
                    "mtime": receipt_path.stat().st_mtime,
                }
            )
        matching = [
            item
            for item in candidates
            if item["anchor_index"] > 0
            and item["anchor_error_rad"] <= RECOVERY_PATH_MATCH_RAD
        ]
        if not matching:
            nearest = min(
                (item["anchor_error_rad"] for item in candidates), default=math.inf
            )
            detail = "none" if not math.isfinite(nearest) else f"{nearest:.6f} rad"
            raise PermissionError(
                "no failed admitted ClickBell path matches the live right arm; "
                f"nearest error={detail}"
            )
        return max(matching, key=lambda item: item["mtime"])

    def validate_request(self) -> dict:
        self._armed()
        return self._candidate()

    def execute(self) -> dict:
        lease = self._armed()
        candidate = self._candidate()
        since = lease["since"]
        settings = ContactStopSettings(
            effort_delta_nm=self.press_settings.effort_delta_nm,
            speed_percent=self.press_settings.speed_percent,
            feedback_max_age_s=self.press_settings.feedback_max_age_s,
            waypoint_timeout_s=self.press_settings.waypoint_timeout_s,
            settle_tolerance_rad=self.press_settings.settle_tolerance_rad,
            max_contact_travel_m=HARD_MAX_CONTACT_TRAVEL_M,
            joint_margin_rad=self.press_settings.joint_margin_rad,
            max_joint_step_rad=self.press_settings.max_joint_step_rad,
        )
        executor = PiperGuardedPressExecutor(
            self.reader, settings, sleeper=self.sleeper
        )
        receipt = executor.recover_along_path(
            candidate["path"],
            arm="right",
            operator_lease_id=f"agent:{since}:click_bell_recovery",
            execute=True,
            authorization_guard=lambda: self._require_same_lease(since),
        )
        receipt.update(
            {
                "kind": "piper_agent_click_bell_recovery",
                "agent_tool_exposed": True,
                "recovery_id": uuid.uuid4().hex[:12],
                "source_session": str(candidate["receipt_path"].parents[1]),
                "source_receipt": str(candidate["receipt_path"]),
                "source_plan": str(candidate["plan_path"]),
                "source_plan_id": candidate["plan"].get("plan_id"),
                "source_scope": candidate["source_scope"],
                "authorization": {
                    "lease_since": since,
                    "lease_until": lease.get("until"),
                    "lease_by": lease.get("by"),
                },
            }
        )
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = self.root / "piper-motion" / f"agent_click_bell_recovery_{stamp}.json"
        write_private_json(path, receipt, directory_mode=0o700)
        self.used_lease = since
        self.last_recovery = {
            "status": receipt.get("status"),
            "receipt": str(path),
            "source_plan_id": receipt.get("source_plan_id"),
            "source_scope": receipt.get("source_scope"),
            "anchor_index": receipt.get("anchor_index"),
            "anchor_error_rad": receipt.get("anchor_error_rad"),
            "finished_at": receipt.get("finished_at", _utc_now()),
        }
        if receipt.get("status") != "passed":
            reasons = receipt.get("failure_reasons") or ["path recovery failed"]
            raise RuntimeError("; ".join(str(reason) for reason in reasons))
        return {
            "executed": True,
            "status": "passed",
            "recovery_id": receipt["recovery_id"],
            "source_plan_id": receipt.get("source_plan_id"),
            "source_scope": receipt.get("source_scope"),
            "reverse_waypoints": receipt.get("reverse_waypoints"),
            "final_start_error_rad": receipt.get("final_start_error_rad"),
            "receipt": str(path),
            "next_action": "capture a fresh cam_right_wrist RGB-D observation",
        }

    def status(self) -> dict:
        try:
            lease = self._armed()
            candidate = self._candidate()
            available = True
            reason = None
        except (PermissionError, ValueError) as exc:
            lease = self.arming_reader(self.root)
            candidate = None
            available = False
            reason = str(exc)
        return {
            "enabled": True,
            "available": available,
            "reason": reason,
            "one_call_per_arming": True,
            "arming": lease,
            "candidate": None
            if candidate is None
            else {
                "source_plan_id": candidate["plan"].get("plan_id"),
                "source_scope": candidate["source_scope"],
                "anchor_index": candidate["anchor_index"],
                "anchor_error_rad": candidate["anchor_error_rad"],
                "reverse_waypoints": candidate["anchor_index"],
            },
            "last_recovery": self.last_recovery,
        }
