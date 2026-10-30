"""Derive review-only Piper workspace candidates from an audited joint path."""

from __future__ import annotations

from collections.abc import Iterable
import math

import numpy as np

from .kinematics import joint_frames


def derive_workspace_candidate(
    plan: dict,
    *,
    arm: str,
    tcp_margin_m: Iterable[float] = (0.01, 0.01, 0.01),
    link_margin_m: float = 0.01,
) -> dict:
    """Return a narrow path envelope; never claim physical obstacle clearance."""
    if arm not in ("left", "right"):
        raise ValueError("arm must be left or right")
    if plan.get("kind") != "click_bell_readonly_press_path" or plan.get("status") != "passed":
        raise ValueError("a passed ClickBell press-path report is required")
    if plan.get("arm") not in (None, arm):
        raise ValueError("press plan arm does not match the requested arm")
    margin = np.asarray(tuple(tcp_margin_m), dtype=np.float64)
    if margin.shape != (3,) or not np.all(np.isfinite(margin)) or np.any(margin <= 0):
        raise ValueError("tcp_margin_m must contain three positive finite values")
    if not math.isfinite(link_margin_m) or link_margin_m <= 0:
        raise ValueError("link_margin_m must be positive and finite")

    rows = [plan.get("start_joints_rad")]
    for stage in plan.get("stages", ()):
        rows.extend(
            waypoint.get("joints_rad")
            for waypoint in stage.get("plan", {}).get("waypoints", ())
        )
    joints = [np.asarray(row, dtype=np.float64) for row in rows]
    if not joints or any(row.shape != (6,) or not np.all(np.isfinite(row)) for row in joints):
        raise ValueError("press plan contains invalid joint waypoints")

    tcp_points = []
    distal_z = []
    for row in joints:
        frames = joint_frames(row)
        tcp_points.append(frames[-1][:3, 3])
        distal_z.extend(float(frame[2, 3]) for frame in frames[2:])
    tcp = np.asarray(tcp_points)
    tcp_min = np.min(tcp, axis=0)
    tcp_max = np.max(tcp, axis=0)
    observed_link_min = min(distal_z)
    return {
        "schema_version": 1,
        "kind": "piper_workspace_candidate",
        "status": "candidate",
        "arm": arm,
        "source_frame_id": plan.get("frame_id"),
        "waypoint_states": len(joints),
        "observed_path_envelope": {
            "tcp_min_m": np.round(tcp_min, 6).tolist(),
            "tcp_max_m": np.round(tcp_max, 6).tolist(),
            "distal_link_min_z_m": round(observed_link_min, 6),
        },
        "margins": {
            "tcp_m": margin.tolist(),
            "distal_link_m": float(link_margin_m),
        },
        "candidate": {
            "workspace_bounds": {
                arm: {
                    "min": np.round(tcp_min - margin, 6).tolist(),
                    "max": np.round(tcp_max + margin, 6).tolist(),
                }
            },
            "link_min_z_m": round(observed_link_min - link_margin_m, 6),
        },
        "activation_allowed": False,
        "operator_review_required": [
            "confirm the complete TCP corridor is clear in the selected arm base frame",
            "confirm every distal link stays above the physical table and fixtures",
            "rerun the read-only path audit with the candidate backend configuration",
        ],
    }
