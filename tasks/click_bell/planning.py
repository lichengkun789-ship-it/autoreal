"""Read-only ClickBell contact-path planning on top of the Piper IK checker."""

from __future__ import annotations

from collections.abc import Iterable, Sequence

import numpy as np

from services.piper.ik import plan_pose, rotation_from_rpy


def plan_press_path(
    current_joints_rad: Iterable[float],
    localization: dict,
    *,
    tool_contact_offset_tcp_m: Iterable[float],
    rpy_rad: Iterable[float],
    approach_distances_m: Sequence[float] = (0.06, 0.02, 0.005, 0.0),
    waypoint_counts: Sequence[int] = (64, 24, 16, 8),
    joint_margin_rad: float = 0.02,
    max_joint_step_rad: float = 0.05,
    workspace_min: Iterable[float] | None = None,
    workspace_max: Iterable[float] | None = None,
    link_min_z_m: float | None = None,
    tool_geometry_verified: bool = False,
) -> dict:
    """Plan staged tool-contact targets without authorizing or sending motion."""
    surface = localization.get("surface")
    if not isinstance(surface, dict) or surface.get("status") != "passed":
        raise ValueError("a passed press-cap surface localization is required")
    center = np.asarray(surface.get("position_m"), dtype=np.float64)
    normal = np.asarray(surface.get("normal_base"), dtype=np.float64)
    offset = np.asarray(tuple(tool_contact_offset_tcp_m), dtype=np.float64)
    rpy = np.asarray(tuple(rpy_rad), dtype=np.float64)
    if center.shape != (3,) or normal.shape != (3,):
        raise ValueError("surface position and normal must be 3-D")
    if offset.shape != (3,) or rpy.shape != (3,):
        raise ValueError("tool contact offset and RPY must be 3-D")
    if not np.all(np.isfinite(np.concatenate((center, normal, offset, rpy)))):
        raise ValueError("press planning inputs must be finite")
    normal_length = float(np.linalg.norm(normal))
    if normal_length < 1e-8:
        raise ValueError("surface normal must be non-zero")
    normal /= normal_length
    distances = tuple(float(value) for value in approach_distances_m)
    counts = tuple(waypoint_counts)
    if not distances or len(distances) != len(counts):
        raise ValueError("approach distances and waypoint counts must have equal length")
    if any(not np.isfinite(value) or value < 0 for value in distances):
        raise ValueError("approach distances must be finite and non-negative")
    if any(before <= after for before, after in zip(distances, distances[1:])):
        raise ValueError("approach distances must be strictly descending")

    rotation = rotation_from_rpy(rpy)
    current = list(current_joints_rad)
    start_joints = list(current)
    stages = []
    for distance, waypoints in zip(distances, counts):
        contact = center + distance * normal
        tcp = contact - rotation @ offset
        plan = plan_pose(
            current,
            tcp,
            rpy_rad=rpy,
            waypoints=waypoints,
            joint_margin_rad=joint_margin_rad,
            max_joint_step_rad=max_joint_step_rad,
            workspace_min=workspace_min,
            workspace_max=workspace_max,
            link_min_z_m=link_min_z_m,
        )
        stages.append(
            {
                "approach_distance_m": distance,
                "contact_target_m": np.round(contact, 6).tolist(),
                "tcp_target_m": np.round(tcp, 6).tolist(),
                "plan": plan,
            }
        )
        if plan["status"] != "passed":
            break
        current = plan["target_joints_rad"]

    passed = len(stages) == len(distances) and all(
        stage["plan"]["status"] == "passed" for stage in stages
    )
    workspace_configured = workspace_min is not None and workspace_max is not None
    clearance_configured = link_min_z_m is not None
    warnings = []
    if not tool_geometry_verified:
        warnings.append("tool contact offset is provisional and must be physically verified")
    if not workspace_configured:
        warnings.append("workspace bounds are not configured")
    if not clearance_configured:
        warnings.append("distal-link minimum height is not configured")
    warnings.append(
        "guarded contact stop and retreat exist only in the operator runner; "
        "Agent execution is not admitted"
    )
    geometry_admissible = bool(
        passed
        and tool_geometry_verified
        and workspace_configured
        and clearance_configured
    )
    return {
        "schema_version": 1,
        "kind": "click_bell_readonly_press_path",
        "status": "passed" if passed else "failed",
        "dry_run": True,
        "motion_authorized": False,
        "geometry_admissible": geometry_admissible,
        "execution_admissible": False,
        "operator_guarded_executor_available": True,
        "frame_id": localization.get("frame_id"),
        "camera": localization.get("camera"),
        "frame": localization.get("frame"),
        "surface": surface,
        "start_joints_rad": start_joints,
        "tool_contact_offset_tcp_m": offset.tolist(),
        "tool_geometry_verified": bool(tool_geometry_verified),
        "target_rpy_rad": rpy.tolist(),
        "stages_requested": len(distances),
        "stages_planned": len(stages),
        "stages": stages,
        "warnings": warnings,
    }
