"""Fresh RGB-D localization and one-shot dynamic ClickBell planning."""

from __future__ import annotations

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
from services.piper.safety import DEFAULT_MAX_JOINT_STEP_RAD

from tasks.click_bell.perception import localize_saved_frame

from .planning import plan_press_path


MAX_FRAME_AGE_S = 120.0
MAX_CAPTURE_MOVEMENT_RAD = 0.002
MAX_STATE_DRIFT_RAD = 0.01
MIN_VALID_DEPTH_FRACTION = 0.75
MAX_PLANE_RMSE_MM = 3.0
MAX_DEPTH_SPREAD_95_MM = 15.0
MAX_IK_WAYPOINTS = 128
DYNAMIC_APPROACH_WAYPOINTS = 40
DYNAMIC_CONTACT_WAYPOINTS = 4


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _template_parameters(template: dict) -> tuple[list[float], list[int]]:
    stages = template.get("stages")
    if not isinstance(stages, list) or len(stages) < 2:
        raise ValueError("dynamic ClickBell planning requires a staged template plan")
    contact_clearance = float(stages[-2]["approach_distance_m"])
    contact_distance = float(stages[-1]["approach_distance_m"])
    approach_count = min(
        MAX_IK_WAYPOINTS,
        DYNAMIC_APPROACH_WAYPOINTS,
        sum(len(stage.get("plan", {}).get("waypoints", ())) for stage in stages[:-1]),
    )
    contact_count = min(
        MAX_IK_WAYPOINTS,
        DYNAMIC_CONTACT_WAYPOINTS,
        len(stages[-1].get("plan", {}).get("waypoints", ())),
    )
    if contact_distance != 0.0 or not 0 < contact_clearance <= 0.005:
        raise ValueError("template plan must end with an admitted 5 mm contact stage")
    counts = [approach_count, contact_count]
    if any(count < 2 for count in counts):
        raise ValueError("template plan has an empty waypoint stage")
    return [contact_clearance, contact_distance], counts


def validate_localization(localization: dict, *, arm: str) -> None:
    expected_frame = f"piper_{arm}_base"
    surface = localization.get("surface", {})
    if localization.get("status") != "passed" or surface.get("status") != "passed":
        raise ValueError("dynamic ClickBell planning requires a passed localization")
    if localization.get("frame") != expected_frame:
        raise ValueError(
            f"dynamic {arm}-arm planning requires frame {expected_frame}, "
            f"found {localization.get('frame')}"
        )
    if float(surface.get("valid_depth_fraction", 0.0)) < MIN_VALID_DEPTH_FRACTION:
        raise ValueError("press-cap localization has insufficient valid depth")
    if float(surface.get("plane_rmse_mm", math.inf)) > MAX_PLANE_RMSE_MM:
        raise ValueError("press-cap surface plane RMSE is too large")
    if float(surface.get("depth_fit_spread_95_mm", math.inf)) > MAX_DEPTH_SPREAD_95_MM:
        raise ValueError("press-cap depth uncertainty is too large")


def build_dynamic_press_plan(
    template: dict,
    localization: dict,
    current_joints_rad,
    *,
    arm: str,
    backend_options: dict,
    workspace_check_enabled: bool = True,
) -> dict:
    """Build a fresh read-only path using commissioned geometry and live localization."""
    validate_localization(localization, arm=arm)
    distances, counts = _template_parameters(template)
    if type(workspace_check_enabled) is not bool:
        raise ValueError("workspace_check_enabled must be true or false")
    bounds = (backend_options.get("workspace_bounds") or {}).get(arm)
    if workspace_check_enabled and (
        not isinstance(bounds, dict) or set(bounds) != {"min", "max"}
    ):
        raise ValueError(f"dynamic ClickBell planning requires {arm} workspace bounds")
    link_min_z = backend_options.get("link_min_z_m")
    if link_min_z is None:
        raise ValueError("dynamic ClickBell planning requires link_min_z_m")
    if template.get("tool_geometry_verified") is not True:
        raise ValueError("dynamic ClickBell planning requires verified tool geometry")

    plan = plan_press_path(
        current_joints_rad,
        localization,
        tool_contact_offset_tcp_m=template["tool_contact_offset_tcp_m"],
        rpy_rad=template["target_rpy_rad"],
        approach_distances_m=distances,
        waypoint_counts=counts,
        joint_margin_rad=float(backend_options.get("joint_limit_margin_rad", 0.02)),
        max_joint_step_rad=float(
            backend_options.get("max_joint_step_rad", DEFAULT_MAX_JOINT_STEP_RAD)
        ),
        workspace_min=bounds["min"] if workspace_check_enabled else None,
        workspace_max=bounds["max"] if workspace_check_enabled else None,
        workspace_required=workspace_check_enabled,
        link_min_z_m=float(link_min_z),
        tool_geometry_verified=True,
    )
    plan["arm"] = arm
    plan["dynamic"] = True
    plan["template_frame_id"] = template.get("frame_id")
    plan["workspace_policy"] = (
        "configured_bounds" if workspace_check_enabled else "operator_disabled_kinematic_only"
    )
    plan["execution_admissible"] = bool(
        plan.get("status") == "passed" and plan.get("geometry_admissible") is True
    )
    plan["warnings"] = [
        warning
        for warning in plan.get("warnings", ())
        if "Agent execution is not admitted" not in warning
    ]
    return plan


class ClickBellDynamicPlanner:
    """Create and consume one fresh dynamic plan per operator arming lease."""

    def __init__(
        self,
        root: Path,
        frames_root: Path,
        reader,
        press_settings,
        backend_options: dict,
        *,
        clock=time.time,
        arming_reader=arming.read,
    ):
        self.root = Path(root)
        self.frames_root = Path(frames_root)
        self.reader = reader
        self.press_settings = press_settings
        self.backend_options = backend_options
        self.clock = clock
        self.arming_reader = arming_reader
        self.latest = None

    def _armed(self) -> dict:
        state = self.arming_reader(self.root)
        if state.get("armed") is not True:
            raise PermissionError("dynamic ClickBell planning requires an arming lease")
        since = state.get("since")
        if not isinstance(since, (int, float)) or not math.isfinite(since):
            raise PermissionError("operator arming lease has no valid creation time")
        return state

    def _frame(self, frame_id: str) -> tuple[Path, dict]:
        if not isinstance(frame_id, str) or not frame_id.isalnum():
            raise ValueError("invalid frame_id")
        directory = self.frames_root / frame_id
        try:
            metadata = json.loads((directory / "meta.json").read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"unknown frame_id {frame_id}") from exc
        age = float(self.clock()) - float(metadata.get("captured_at", -math.inf))
        if not 0 <= age <= MAX_FRAME_AGE_S:
            raise ValueError(
                f"ClickBell frame is stale ({age:.1f}s > {MAX_FRAME_AGE_S:.1f}s)"
            )
        movement = float(metadata.get("max_joint_movement_during_capture_rad", math.inf))
        if movement > MAX_CAPTURE_MOVEMENT_RAD:
            raise ValueError("an arm moved too far during ClickBell image capture")
        return directory, metadata

    def plan(self, frame_id: str, camera: str, *, radius: int = 3) -> dict:
        lease = self._armed()
        if (
            isinstance(self.latest, dict)
            and self.latest.get("authorization", {}).get("lease_since") == lease["since"]
        ):
            raise PermissionError(
                "dynamic ClickBell plan budget is exhausted; disarm and inspect"
            )
        directory, metadata = self._frame(frame_id)
        localization = localize_saved_frame(directory, camera, radius=radius)
        arm = self.press_settings.arm
        validate_localization(localization, arm=arm)
        snapshot = self.reader.snapshot()
        arm_state = snapshot.get("arms", {}).get(arm, {})
        current = np.asarray(arm_state.get("joints_rad"), dtype=np.float64)
        observed = np.asarray(
            metadata.get("state_after", {})
            .get("arms", {})
            .get(arm, {})
            .get("joints_rad"),
            dtype=np.float64,
        )
        if current.shape != (6,) or observed.shape != (6,):
            raise ValueError("dynamic ClickBell planning requires six measured arm joints")
        drift = float(np.max(np.abs(current - observed)))
        if drift > MAX_STATE_DRIFT_RAD:
            raise ValueError(
                f"arm moved after RGB-D capture ({drift:.6f} > {MAX_STATE_DRIFT_RAD:.6f} rad)"
            )

        plan = build_dynamic_press_plan(
            self.press_settings.plan,
            localization,
            current.tolist(),
            arm=arm,
            backend_options=self.backend_options,
            workspace_check_enabled=self.press_settings.dynamic_workspace_check,
        )
        plan_id = uuid.uuid4().hex[:12]
        created = float(self.clock())
        frame_expires = float(metadata["captured_at"]) + MAX_FRAME_AGE_S
        plan.update(
            {
                "plan_id": plan_id,
                "created_at": _utc_now(),
                "created_unix": created,
                "expires_unix": min(created + MAX_FRAME_AGE_S, frame_expires),
                "authorization": {
                    "lease_since": lease["since"],
                    "lease_until": lease.get("until"),
                    "lease_by": lease.get("by"),
                },
                "source_frame_directory": str(directory),
                "capture_to_plan_joint_drift_rad": drift,
                "used": False,
            }
        )
        canonical = json.dumps(plan, sort_keys=True, separators=(",", ":")).encode()
        plan["plan_sha256"] = hashlib.sha256(canonical).hexdigest()
        path = self.root / "click-bell-plans" / f"{plan_id}.json"
        write_private_json(path, plan, directory_mode=0o700)
        self.latest = {**plan, "receipt": str(path)}
        return self.latest

    def _validated_plan(self, plan_id: str) -> tuple[dict, dict]:
        lease = self._armed()
        plan = self.latest
        if not isinstance(plan, dict) or plan.get("plan_id") != plan_id:
            raise ValueError("plan_id is not the latest dynamic ClickBell plan")
        if plan.get("used") is True:
            raise PermissionError("dynamic ClickBell plan was already consumed")
        if plan.get("execution_admissible") is not True:
            raise PermissionError("dynamic ClickBell plan did not pass read-only planning")
        if plan.get("authorization", {}).get("lease_since") != lease["since"]:
            raise PermissionError("dynamic ClickBell plan belongs to a different arming lease")
        if float(self.clock()) > float(plan.get("expires_unix", -math.inf)):
            raise PermissionError("dynamic ClickBell plan expired; observe and plan again")
        current = np.asarray(
            self.reader.snapshot().get("arms", {}).get(self.press_settings.arm, {}).get("joints_rad"),
            dtype=np.float64,
        )
        start = np.asarray(plan.get("start_joints_rad"), dtype=np.float64)
        if current.shape != (6,) or start.shape != (6,):
            raise ValueError("dynamic ClickBell plan has no valid start state")
        drift = float(np.max(np.abs(current - start)))
        if drift > MAX_STATE_DRIFT_RAD:
            raise PermissionError(
                f"arm moved after planning ({drift:.6f} > {MAX_STATE_DRIFT_RAD:.6f} rad)"
            )
        return plan, lease

    def validate(self, plan_id: str) -> dict:
        plan, _ = self._validated_plan(plan_id)
        return plan

    def consume(self, plan_id: str) -> dict:
        plan, _ = self._validated_plan(plan_id)
        plan["used"] = True
        plan["consumed_at"] = _utc_now()
        write_private_json(Path(plan["receipt"]), plan, directory_mode=0o700)
        return plan

    def status(self) -> dict | None:
        if self.latest is None:
            return None
        return {
            key: self.latest.get(key)
            for key in (
                "plan_id",
                "status",
                "execution_admissible",
                "frame_id",
                "camera",
                "expires_unix",
                "used",
                "receipt",
            )
        }
