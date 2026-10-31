"""Plan-bound Cartesian motion controller for the Piper RAX backend."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np

from services import arming
from services.json_io import write_private_json

from .trajectory import PiperJointTrajectoryExecutor, TrajectorySettings


HARD_MAX_TRANSLATION_M = 0.5
HARD_MAX_ROTATION_RAD = math.pi
HARD_MAX_WAYPOINTS = 128
HARD_MAX_EXECUTIONS_PER_ARMING = 20
HARD_MAX_PLAN_TTL_S = 600.0


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _rotation_distance_rpy(start_rpy, target_rpy) -> float:
    def matrix(rpy):
        roll, pitch, yaw = (float(value) for value in rpy)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.asarray(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float64,
        )

    relative = matrix(target_rpy) @ matrix(start_rpy).T
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


@dataclass(frozen=True)
class CartesianMotionSettings:
    enabled: bool
    allowed_arms: frozenset[str]
    max_translation_m: float
    max_rotation_rad: float
    max_waypoints: int
    max_executions_per_arming: int
    plan_ttl_s: float
    require_workspace_checks: bool
    trajectory: TrajectorySettings

    @classmethod
    def from_options(cls, value, *, backend_options, arms):
        value = {} if value is None else value
        if not isinstance(value, dict):
            raise ValueError("cartesian_motion must be an object")
        allowed_keys = {
            "enabled",
            "allowed_arms",
            "speed_percent",
            "max_translation_m",
            "max_rotation_rad",
            "max_waypoints",
            "max_executions_per_arming",
            "plan_ttl_s",
            "require_workspace_checks",
            "poll_interval_s",
            "waypoint_timeout_s",
            "settle_tolerance_rad",
            "max_start_error_rad",
        }
        unknown = set(value) - allowed_keys
        if unknown:
            raise ValueError(f"unknown cartesian_motion options: {sorted(unknown)}")
        enabled = value.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("cartesian_motion.enabled must be true or false")
        allowed = value.get("allowed_arms", [])
        if not isinstance(allowed, list) or set(allowed) - {"left", "right"}:
            raise ValueError("cartesian_motion.allowed_arms must contain left and/or right")
        if len(allowed) != len(set(allowed)):
            raise ValueError("cartesian_motion.allowed_arms contains duplicates")
        maximum_translation = float(value.get("max_translation_m", 0.25))
        maximum_rotation = float(value.get("max_rotation_rad", 0.75))
        maximum_waypoints = value.get("max_waypoints", 64)
        maximum_executions = value.get("max_executions_per_arming", 10)
        ttl = float(value.get("plan_ttl_s", 120.0))
        require_workspace_checks = value.get("require_workspace_checks", True)
        if type(require_workspace_checks) is not bool:
            raise ValueError("cartesian require_workspace_checks must be true or false")
        if not math.isfinite(maximum_translation) or not (
            0 < maximum_translation <= HARD_MAX_TRANSLATION_M
        ):
            raise ValueError(
                f"cartesian max_translation_m must be in (0, {HARD_MAX_TRANSLATION_M}]"
            )
        if not math.isfinite(maximum_rotation) or not (
            0 < maximum_rotation <= HARD_MAX_ROTATION_RAD
        ):
            raise ValueError(
                f"cartesian max_rotation_rad must be in (0, {HARD_MAX_ROTATION_RAD}]"
            )
        if type(maximum_waypoints) is not int or not 2 <= maximum_waypoints <= HARD_MAX_WAYPOINTS:
            raise ValueError(f"cartesian max_waypoints must be in 2..{HARD_MAX_WAYPOINTS}")
        if (
            type(maximum_executions) is not int
            or not 1 <= maximum_executions <= HARD_MAX_EXECUTIONS_PER_ARMING
        ):
            raise ValueError(
                "cartesian max_executions_per_arming must be in "
                f"1..{HARD_MAX_EXECUTIONS_PER_ARMING}"
            )
        if not math.isfinite(ttl) or not 1 <= ttl <= HARD_MAX_PLAN_TTL_S:
            raise ValueError(f"cartesian plan_ttl_s must be in 1..{HARD_MAX_PLAN_TTL_S}")

        trajectory = TrajectorySettings(
            speed_percent=value.get("speed_percent", 2),
            feedback_max_age_s=float(backend_options.get("feedback_max_age_s", 0.25)),
            poll_interval_s=float(value.get("poll_interval_s", 0.05)),
            waypoint_timeout_s=float(value.get("waypoint_timeout_s", 5.0)),
            settle_tolerance_rad=float(value.get("settle_tolerance_rad", 0.002)),
            max_start_error_rad=float(value.get("max_start_error_rad", 0.02)),
            joint_margin_rad=float(backend_options.get("joint_limit_margin_rad", 0.02)),
            max_joint_step_rad=float(backend_options["max_joint_step_rad"]),
        )
        if enabled:
            if not allowed:
                raise ValueError("enabled Cartesian motion requires allowed_arms")
            configured_arms = set(arms)
            if set(allowed) - configured_arms:
                raise ValueError("Cartesian allowed_arms must be configured Piper arms")
            if require_workspace_checks:
                bounds = backend_options.get("workspace_bounds") or {}
                missing = sorted(set(allowed) - set(bounds))
                if missing:
                    raise ValueError(
                        f"Cartesian motion requires workspace bounds for arms: {missing}"
                    )
                if backend_options.get("link_min_z_m") is None:
                    raise ValueError("Cartesian motion requires link_min_z_m")
        return cls(
            enabled=enabled,
            allowed_arms=frozenset(allowed),
            max_translation_m=maximum_translation,
            max_rotation_rad=maximum_rotation,
            max_waypoints=maximum_waypoints,
            max_executions_per_arming=maximum_executions,
            plan_ttl_s=ttl,
            require_workspace_checks=require_workspace_checks,
            trajectory=trajectory,
        )

    def public(self) -> dict:
        return {
            "enabled": self.enabled,
            "allowed_arms": sorted(self.allowed_arms),
            "max_translation_m": self.max_translation_m,
            "max_rotation_rad": self.max_rotation_rad,
            "max_waypoints": self.max_waypoints,
            "max_executions_per_arming": self.max_executions_per_arming,
            "plan_ttl_s": self.plan_ttl_s,
            "require_workspace_checks": self.require_workspace_checks,
            "speed_percent": self.trajectory.speed_percent,
            "max_joint_step_rad": self.trajectory.max_joint_step_rad,
        }


class CartesianMotionStopped(RuntimeError):
    def __init__(self, message, *, control_certain, robot_state=None):
        super().__init__(message)
        self.control_certain = control_certain
        self.robot_state = robot_state


class PiperCartesianMotionController:
    """Own short-lived plans and account for their execution and retreat."""

    def __init__(
        self,
        root: Path,
        reader,
        settings: CartesianMotionSettings,
        *,
        executor=None,
        clock=time.time,
        arming_reader=arming.read,
    ):
        if not settings.enabled:
            raise ValueError("cannot create a Cartesian controller while motion is disabled")
        self.root = Path(root)
        self.reader = reader
        self.settings = settings
        self.executor = executor or PiperJointTrajectoryExecutor(
            reader, settings.trajectory
        )
        self.clock = clock
        self.arming_reader = arming_reader
        self.plans = {}
        self.lease_since = None
        self.executions_used = 0
        self.last_motion = None

    def _armed(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is not True:
            raise PermissionError(
                f"Cartesian motion is not armed ({state.get('reason', 'unknown reason')})"
            )
        since = state.get("since")
        if not isinstance(since, (int, float)) or not math.isfinite(since):
            raise PermissionError("operator arming lease has no valid creation time")
        if self.lease_since != since:
            self.lease_since = since
            self.executions_used = 0
            self.plans.clear()
        return state

    def _require_same_lease(self, since: float) -> None:
        if self._armed().get("since") != since:
            raise PermissionError("operator arming lease changed during Cartesian motion")

    def _plan_path(self, plan: dict) -> tuple[list[float], list[list[float]]]:
        start = [float(value) for value in plan.get("start_joints_rad", ())]
        rows = plan.get("waypoints")
        if len(start) != 6 or not isinstance(rows, list) or not rows:
            raise ValueError("Cartesian plan has no valid start or waypoints")
        waypoints = []
        for row in rows:
            joints = [float(value) for value in row.get("joints_rad", ())]
            if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
                raise ValueError("Cartesian plan contains an invalid joint waypoint")
            waypoints.append(joints)
        return start, waypoints

    def _validate_geometry(self, plan: dict, arm: str) -> None:
        if arm not in self.settings.allowed_arms:
            raise PermissionError(f"Cartesian motion is not enabled for the {arm} arm")
        if plan.get("status") != "passed" or plan.get("planning_accepted") is not True:
            raise ValueError("only a passed, ready Cartesian plan can be staged")
        checks = plan.get("checks", {})
        if self.settings.require_workspace_checks and (
            checks.get("workspace_bounds") is not True
            or checks.get("distal_link_min_z") is not True
        ):
            raise ValueError("Cartesian execution requires workspace and link-height checks")
        waypoints = plan.get("waypoints", [])
        if not 1 <= len(waypoints) <= self.settings.max_waypoints:
            raise ValueError("Cartesian plan exceeds the configured waypoint budget")
        start_position = np.asarray(plan.get("start", {}).get("position_m"), dtype=np.float64)
        target_position = np.asarray(plan.get("target", {}).get("position_m"), dtype=np.float64)
        if start_position.shape != (3,) or target_position.shape != (3,):
            raise ValueError("Cartesian plan is missing start or target position")
        translation = float(np.linalg.norm(target_position - start_position))
        if translation > self.settings.max_translation_m + 1e-12:
            raise ValueError(
                f"Cartesian translation {translation:.4f} m exceeds "
                f"{self.settings.max_translation_m:.4f} m"
            )
        rotation = _rotation_distance_rpy(
            plan["start"]["rpy_rad"], plan["target"]["rpy_rad"]
        )
        if rotation > self.settings.max_rotation_rad + 1e-12:
            raise ValueError(
                f"Cartesian rotation {rotation:.4f} rad exceeds "
                f"{self.settings.max_rotation_rad:.4f} rad"
            )
        plan["execution_limits"] = {
            "translation_m": translation,
            "rotation_rad": rotation,
            **self.settings.public(),
        }

    def stage(self, plan: dict, *, arm: str) -> dict:
        lease = self._armed()
        self._validate_geometry(plan, arm)
        if self.executions_used >= self.settings.max_executions_per_arming:
            raise PermissionError("Cartesian execution budget is exhausted; disarm and inspect")
        if any(
            item.get("state")
            in {"planned", "partially_executed", "partially_retreated", "executing"}
            for item in self.plans.values()
        ):
            raise PermissionError("finish or retreat the current Cartesian plan first")
        for previous in self.plans.values():
            if previous.get("state") == "executed":
                previous["state"] = "superseded"
                previous["superseded_at"] = _utc_now()
                write_private_json(
                    Path(previous["receipt"]), previous, directory_mode=0o700
                )
        start, _ = self._plan_path(plan)
        plan_id = uuid.uuid4().hex[:12]
        now = float(self.clock())
        staged = {
            **plan,
            "plan_id": plan_id,
            "arm": arm,
            "state": "planned",
            "used": False,
            "progress_waypoints": 0,
            "execution_receipts": [],
            "retreat_receipts": [],
            "created_at": _utc_now(),
            "created_unix": now,
            "expires_unix": now + self.settings.plan_ttl_s,
            "authorization": {
                "lease_since": lease["since"],
                "lease_until": lease.get("until"),
                "lease_by": lease.get("by"),
            },
            "start_joints_rad": start,
        }
        canonical = json.dumps(staged, sort_keys=True, separators=(",", ":")).encode()
        staged["plan_sha256"] = hashlib.sha256(canonical).hexdigest()
        path = self.root / "piper-cartesian-plans" / f"{plan_id}.json"
        staged["receipt"] = str(path)
        write_private_json(path, staged, directory_mode=0o700)
        self.plans[plan_id] = staged
        return {
            "plan_id": plan_id,
            "execution_admissible": True,
            "expires_unix": staged["expires_unix"],
            "plan_receipt": str(path),
            "execution_limits": staged["execution_limits"],
            "waypoints_total": len(staged["waypoints"]),
        }

    def _plan(self, plan_id: str, *, states) -> tuple[dict, dict]:
        lease = self._armed()
        plan = self.plans.get(plan_id)
        if not isinstance(plan, dict):
            raise ValueError("unknown Cartesian plan_id")
        if plan.get("state") not in states:
            raise PermissionError(
                f"Cartesian plan state is {plan.get('state')}; expected {sorted(states)}"
            )
        if plan.get("authorization", {}).get("lease_since") != lease["since"]:
            raise PermissionError("Cartesian plan belongs to a different arming lease")
        if (
            float(self.clock()) > float(plan.get("expires_unix", -math.inf))
            and int(plan.get("progress_waypoints", 0)) == 0
        ):
            raise PermissionError("Cartesian plan expired; plan again from live state")
        return plan, lease

    @staticmethod
    def _segment_size(value, available: int, *, operation: str) -> int:
        if value is None:
            return available
        if type(value) is not int or value < 1:
            raise ValueError(f"{operation} waypoints must be a positive integer")
        if value > available:
            raise ValueError(
                f"{operation} requested {value} waypoints but only {available} are available"
            )
        return value

    def validate_execute(self, plan_id: str, waypoints=None) -> dict:
        plan, _ = self._plan(
            plan_id,
            states={"planned", "partially_executed", "partially_retreated"},
        )
        progress = int(plan.get("progress_waypoints", 0))
        remaining = len(plan["waypoints"]) - progress
        self._segment_size(waypoints, remaining, operation="execute")
        if self.executions_used >= self.settings.max_executions_per_arming:
            raise PermissionError("Cartesian execution budget is exhausted; disarm and inspect")
        return plan

    def discard(self, plan_id: str) -> dict:
        plan = self.plans.get(plan_id)
        if not isinstance(plan, dict):
            raise ValueError("unknown Cartesian plan_id")
        if plan.get("state") != "planned" or int(plan.get("progress_waypoints", 0)) != 0:
            raise PermissionError("only an unexecuted planned Cartesian path can be discarded")
        plan["state"] = "discarded"
        plan["discarded_at"] = _utc_now()
        write_private_json(Path(plan["receipt"]), plan, directory_mode=0o700)
        return {
            "status": "passed",
            "tool": "piper_discard_plan",
            "plan_id": plan_id,
            "discarded": True,
            "motion_command_attempted": False,
        }

    def validate_retreat(self, plan_id: str, waypoints=None) -> dict:
        plan, _ = self._plan(
            plan_id,
            states={"executed", "partially_executed", "partially_retreated"},
        )
        progress = int(plan.get("progress_waypoints", 0))
        self._segment_size(waypoints, progress, operation="retreat")
        return plan

    def _save_motion(self, prefix: str, receipt: dict) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = self.root / "piper-motion" / f"{prefix}_{stamp}_{uuid.uuid4().hex[:8]}.json"
        write_private_json(path, receipt, directory_mode=0o700)
        return path

    def execute(self, plan_id: str, *, waypoint_count=None) -> dict:
        plan, lease = self._plan(
            plan_id,
            states={"planned", "partially_executed", "partially_retreated"},
        )
        start, waypoints = self._plan_path(plan)
        progress = int(plan.get("progress_waypoints", 0))
        count = self._segment_size(
            waypoint_count, len(waypoints) - progress, operation="execute"
        )
        segment_start = start if progress == 0 else waypoints[progress - 1]
        segment = waypoints[progress : progress + count]
        previous_state = plan["state"]
        plan["state"] = "executing"
        plan["used"] = True
        write_private_json(Path(plan["receipt"]), plan, directory_mode=0o700)
        receipt = self.executor.execute(
            arm=plan["arm"],
            start=segment_start,
            waypoints=segment,
            authorization_guard=lambda: self._require_same_lease(lease["since"]),
        )
        receipt.update(
            {
                "plan_id": plan_id,
                "plan_sha256": plan["plan_sha256"],
                "authorization": plan["authorization"],
                "execution_limits": plan["execution_limits"],
                "segment": {
                    "direction": "forward",
                    "start_index": progress,
                    "end_index": progress + count,
                    "waypoint_count": count,
                },
            }
        )
        motion_path = self._save_motion("cartesian_execute", receipt)
        if receipt.get("motion_command_attempted") is True:
            self.executions_used += 1
        passed = receipt.get("status") == "passed"
        if passed:
            progress += count
            plan["progress_waypoints"] = progress
            plan["state"] = (
                "executed" if progress == len(waypoints) else "partially_executed"
            )
        else:
            plan["state"] = "failed"
            plan["state_before_failure"] = previous_state
        plan["motion_receipt"] = str(motion_path)
        plan["execution_receipts"].append(str(motion_path))
        write_private_json(Path(plan["receipt"]), plan, directory_mode=0o700)
        self.last_motion = {
            "tool": "piper_execute_plan",
            "plan_id": plan_id,
            "status": receipt.get("status"),
            "receipt": str(motion_path),
        }
        if not passed:
            reasons = "; ".join(receipt.get("failure_reasons", ["Cartesian motion failed"]))
            raise CartesianMotionStopped(
                f"Piper Cartesian motion stopped: {reasons}; receipt: {motion_path}",
                control_certain=receipt.get("control_certain") is True,
                robot_state={"joints_rad": receipt.get("final_joints_rad")},
            )
        return {
            "executed": True,
            "plan_id": plan_id,
            "arm": plan["arm"],
            "target": plan["target"],
            "final_joints_rad": receipt.get("final_joints_rad"),
            "hold": receipt.get("final_hold"),
            "segment_waypoints": count,
            "progress_waypoints": progress,
            "waypoints_total": len(waypoints),
            "waypoints_remaining": len(waypoints) - progress,
            "continue_available": progress < len(waypoints),
            "retreat_available": progress > 0,
            "receipt": str(motion_path),
            "budget": self._budget(),
        }

    def retreat(self, plan_id: str, *, waypoint_count=None) -> dict:
        plan, lease = self._plan(
            plan_id,
            states={"executed", "partially_executed", "partially_retreated"},
        )
        start, waypoints = self._plan_path(plan)
        progress = int(plan.get("progress_waypoints", 0))
        count = self._segment_size(waypoint_count, progress, operation="retreat")
        begin = progress - count
        full_path = [start, *waypoints]
        path = full_path[begin : progress + 1]
        receipt = self.executor.retreat(
            arm=plan["arm"],
            path=path,
            authorization_guard=lambda: self._require_same_lease(lease["since"]),
        )
        receipt.update(
            {
                "plan_id": plan_id,
                "plan_sha256": plan["plan_sha256"],
                "authorization": plan["authorization"],
                "segment": {
                    "direction": "reverse",
                    "start_index": progress,
                    "end_index": begin,
                    "waypoint_count": count,
                },
            }
        )
        motion_path = self._save_motion("cartesian_retreat", receipt)
        passed = receipt.get("status") == "passed"
        if passed:
            progress = begin
            plan["progress_waypoints"] = progress
            plan["state"] = "retreated" if progress == 0 else "partially_retreated"
        else:
            plan["state"] = "retreat_failed"
        plan["retreat_receipt"] = str(motion_path)
        plan["retreat_receipts"].append(str(motion_path))
        write_private_json(Path(plan["receipt"]), plan, directory_mode=0o700)
        self.last_motion = {
            "tool": "piper_retreat_plan",
            "plan_id": plan_id,
            "status": receipt.get("status"),
            "receipt": str(motion_path),
        }
        if not passed:
            reasons = "; ".join(receipt.get("failure_reasons", ["Cartesian retreat failed"]))
            raise CartesianMotionStopped(
                f"Piper Cartesian retreat stopped: {reasons}; receipt: {motion_path}",
                control_certain=receipt.get("control_certain") is True,
                robot_state={"joints_rad": receipt.get("final_joints_rad")},
            )
        return {
            "retreated": True,
            "plan_id": plan_id,
            "arm": plan["arm"],
            "final_joints_rad": receipt.get("final_joints_rad"),
            "hold": receipt.get("final_hold"),
            "segment_waypoints": count,
            "progress_waypoints": progress,
            "waypoints_total": len(waypoints),
            "waypoints_remaining": len(waypoints) - progress,
            "continue_available": progress < len(waypoints),
            "retreat_available": progress > 0,
            "receipt": str(motion_path),
            "budget": self._budget(),
        }

    def hold(self, arm: str) -> dict:
        if arm not in self.settings.allowed_arms:
            raise PermissionError(f"Cartesian motion is not enabled for the {arm} arm")
        receipt = self.executor.hold(arm=arm)
        motion_path = self._save_motion("cartesian_hold", receipt)
        self.last_motion = {
            "tool": "piper_hold",
            "arm": arm,
            "status": receipt.get("status"),
            "receipt": str(motion_path),
        }
        return {**receipt, "receipt": str(motion_path)}

    def _budget(self) -> dict:
        return {
            "executions_used": self.executions_used,
            "executions_remaining": max(
                0,
                self.settings.max_executions_per_arming - self.executions_used,
            ),
        }

    def status(self) -> dict:
        try:
            lease = self._armed()
        except PermissionError:
            lease = self.arming_reader(self.root)
        active = [
            {
                "plan_id": plan["plan_id"],
                "arm": plan["arm"],
                "state": plan["state"],
                "progress_waypoints": int(plan.get("progress_waypoints", 0)),
                "waypoints_total": len(plan.get("waypoints", ())),
                "target": plan["target"],
                "receipt": plan["receipt"],
            }
            for plan in self.plans.values()
        ]
        return {
            **self.settings.public(),
            "arming": lease,
            **self._budget(),
            "plans": active,
            "last_motion": self.last_motion,
        }
