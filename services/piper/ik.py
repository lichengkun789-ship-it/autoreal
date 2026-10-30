"""Deterministic read-only IK and Cartesian path checks for Piper arms."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from .kinematics import end_transform, joint_frames, rotation_to_rpy
from .safety import JOINT_LIMITS_RAD


JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
LOWER = np.asarray([item[0] for item in JOINT_LIMITS_RAD], dtype=np.float64)
UPPER = np.asarray([item[1] for item in JOINT_LIMITS_RAD], dtype=np.float64)


def rotation_from_rpy(rpy_rad: Iterable[float]) -> np.ndarray:
    roll, pitch, yaw = np.asarray(tuple(rpy_rad), dtype=np.float64)
    if not np.all(np.isfinite([roll, pitch, yaw])):
        raise ValueError("RPY values must be finite")
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


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    cosine = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    skew = np.asarray(
        [
            matrix[2, 1] - matrix[1, 2],
            matrix[0, 2] - matrix[2, 0],
            matrix[1, 0] - matrix[0, 1],
        ],
        dtype=np.float64,
    )
    if angle < 1e-8:
        return 0.5 * skew
    if math.pi - angle < 1e-5:
        values, vectors = np.linalg.eigh((matrix + np.eye(3)) / 2.0)
        axis = vectors[:, int(np.argmax(values))]
        if float(axis @ skew) < 0:
            axis = -axis
        return axis * angle
    return angle / (2.0 * math.sin(angle)) * skew


def _rotation_matrix(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def pose_error(target: np.ndarray, measured: np.ndarray) -> np.ndarray:
    translation = target[:3, 3] - measured[:3, 3]
    rotation = _rotation_vector(target[:3, :3] @ measured[:3, :3].T)
    return np.concatenate([translation, rotation])


def numerical_jacobian(joints_rad: Iterable[float], *, epsilon: float = 1e-5) -> np.ndarray:
    joints = np.asarray(tuple(joints_rad), dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise ValueError("Piper Jacobian requires six finite joint angles")
    jacobian = np.zeros((6, 6), dtype=np.float64)
    for index in range(6):
        plus, minus = joints.copy(), joints.copy()
        plus[index] += epsilon
        minus[index] -= epsilon
        T_plus, T_minus = end_transform(plus), end_transform(minus)
        jacobian[:3, index] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2.0 * epsilon)
        jacobian[3:, index] = _rotation_vector(
            T_plus[:3, :3] @ T_minus[:3, :3].T
        ) / (2.0 * epsilon)
    return jacobian


def solve(
    initial_joints_rad: Iterable[float],
    target_transform: np.ndarray,
    *,
    iterations: int = 200,
    damping: float = 0.04,
    max_step_rad: float = 0.08,
    position_tolerance_m: float = 5e-4,
    rotation_tolerance_rad: float = math.radians(0.5),
) -> tuple[np.ndarray, bool, dict]:
    joints = np.asarray(tuple(initial_joints_rad), dtype=np.float64)
    target = np.asarray(target_transform, dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise ValueError("Piper IK requires six finite initial joint angles")
    if target.shape != (4, 4) or not np.all(np.isfinite(target)):
        raise ValueError("Piper IK target must be a finite 4x4 transform")
    if iterations <= 0 or damping <= 0 or max_step_rad <= 0:
        raise ValueError("IK iteration, damping, and step settings must be positive")

    orientation_weight = 0.35
    last_error = None
    for count in range(1, iterations + 1):
        measured = end_transform(joints)
        error = pose_error(target, measured)
        last_error = error
        if (
            np.linalg.norm(error[:3]) <= position_tolerance_m
            and np.linalg.norm(error[3:]) <= rotation_tolerance_rad
        ):
            return joints, True, _solver_report(error, count)
        jacobian = numerical_jacobian(joints)
        weighted_jacobian = jacobian.copy()
        weighted_error = error.copy()
        weighted_jacobian[3:] *= orientation_weight
        weighted_error[3:] *= orientation_weight
        system = weighted_jacobian @ weighted_jacobian.T + damping**2 * np.eye(6)
        delta = weighted_jacobian.T @ np.linalg.solve(system, weighted_error)
        scale = float(np.max(np.abs(delta))) / max_step_rad
        joints += delta / scale if scale > 1.0 else delta
        if np.any(joints < LOWER - 0.5) or np.any(joints > UPPER + 0.5):
            break
    assert last_error is not None
    final_error = pose_error(target, end_transform(joints))
    return joints, False, _solver_report(final_error, count)


def _solver_report(error: np.ndarray, iterations: int) -> dict:
    return {
        "iterations": int(iterations),
        "position_error_mm": round(float(np.linalg.norm(error[:3])) * 1000.0, 3),
        "rotation_error_deg": round(float(np.degrees(np.linalg.norm(error[3:]))), 3),
    }


def _joint_path_reasons(previous: np.ndarray, target: np.ndarray, margin_rad: float) -> list[str]:
    reasons = []
    safe_lower, safe_upper = LOWER + margin_rad, UPPER - margin_rad
    for index, (before, after) in enumerate(zip(previous, target)):
        if after < LOWER[index] or after > UPPER[index]:
            reasons.append(f"{JOINT_NAMES[index]} outside hard limit")
        elif before < safe_lower[index] and after < before - 1e-8:
            reasons.append(f"{JOINT_NAMES[index]} moves deeper below safe margin")
        elif before > safe_upper[index] and after > before + 1e-8:
            reasons.append(f"{JOINT_NAMES[index]} moves deeper above safe margin")
        elif safe_lower[index] <= before <= safe_upper[index] and not (
            safe_lower[index] <= after <= safe_upper[index]
        ):
            reasons.append(f"{JOINT_NAMES[index]} enters joint-limit margin")
    return reasons


def _workspace_reasons(
    joints: np.ndarray,
    workspace_min: np.ndarray | None,
    workspace_max: np.ndarray | None,
    link_min_z_m: float | None,
) -> list[str]:
    reasons = []
    frames = joint_frames(joints)
    tcp = frames[-1][:3, 3]
    if workspace_min is not None and (
        np.any(tcp < workspace_min) or np.any(tcp > workspace_max)
    ):
        reasons.append("TCP leaves configured workspace bounds")
    if link_min_z_m is not None:
        minimum = min(float(frame[2, 3]) for frame in frames[2:])
        if minimum < link_min_z_m:
            reasons.append(
                f"distal link falls below configured z minimum ({minimum:.4f} m)"
            )
    return reasons


def plan_pose(
    current_joints_rad: Iterable[float],
    position_m: Iterable[float],
    *,
    rpy_rad: Iterable[float] | None = None,
    relative: bool = False,
    waypoints: int = 12,
    joint_margin_rad: float = 0.02,
    max_joint_step_rad: float = 0.05,
    workspace_min: Iterable[float] | None = None,
    workspace_max: Iterable[float] | None = None,
    link_min_z_m: float | None = None,
) -> dict:
    current = np.asarray(tuple(current_joints_rad), dtype=np.float64)
    position = np.asarray(tuple(position_m), dtype=np.float64)
    if current.shape != (6,) or not np.all(np.isfinite(current)):
        raise ValueError("Piper pose planning requires six finite current joints")
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("Piper pose planning requires a finite 3-D position")
    if type(relative) is not bool:
        raise ValueError("relative must be a boolean")
    if type(waypoints) is not int or not 2 <= waypoints <= 64:
        raise ValueError("waypoints must be an integer from 2 to 64")
    if joint_margin_rad < 0:
        raise ValueError("joint margin must be non-negative")
    if not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0:
        raise ValueError("maximum joint step must be positive and finite")

    minimum = None if workspace_min is None else np.asarray(tuple(workspace_min), dtype=np.float64)
    maximum = None if workspace_max is None else np.asarray(tuple(workspace_max), dtype=np.float64)
    if (minimum is None) != (maximum is None):
        raise ValueError("workspace minimum and maximum must be configured together")
    if minimum is not None:
        if minimum.shape != (3,) or maximum.shape != (3,) or np.any(minimum >= maximum):
            raise ValueError("workspace bounds must be ordered finite 3-D vectors")
        if not np.all(np.isfinite(minimum)) or not np.all(np.isfinite(maximum)):
            raise ValueError("workspace bounds must be finite")
    if link_min_z_m is not None and not math.isfinite(float(link_min_z_m)):
        raise ValueError("link minimum z must be finite")

    start = end_transform(current)
    goal = np.eye(4, dtype=np.float64)
    goal[:3, 3] = start[:3, 3] + position if relative else position
    goal[:3, :3] = start[:3, :3] if rpy_rad is None else rotation_from_rpy(rpy_rad)
    rotation_delta = _rotation_vector(goal[:3, :3] @ start[:3, :3].T)

    planned = []
    failures = []
    previous = current.copy()
    for index in range(1, waypoints + 1):
        fraction = index / waypoints
        target = np.eye(4, dtype=np.float64)
        target[:3, 3] = (1.0 - fraction) * start[:3, 3] + fraction * goal[:3, 3]
        target[:3, :3] = _rotation_matrix(rotation_delta * fraction) @ start[:3, :3]
        solution, converged, solver = solve(previous, target)
        reasons = []
        if not converged:
            reasons.append("IK did not converge")
        else:
            reasons.extend(_joint_path_reasons(previous, solution, joint_margin_rad))
            maximum_delta = float(np.max(np.abs(solution - previous)))
            if maximum_delta > max_joint_step_rad:
                reasons.append(
                    "joint waypoint step exceeds configured maximum "
                    f"({maximum_delta:.6f} > {max_joint_step_rad:.6f} rad)"
                )
            reasons.extend(
                _workspace_reasons(solution, minimum, maximum, link_min_z_m)
            )
        singular_values = np.linalg.svd(
            numerical_jacobian(solution), compute_uv=False
        )
        minimum_singular_value = float(singular_values[-1])
        condition_number = (
            float(singular_values[0] / minimum_singular_value)
            if minimum_singular_value > 0
            else math.inf
        )
        row = {
            "index": index,
            "fraction": round(fraction, 6),
            "position_m": np.round(target[:3, 3], 6).tolist(),
            "joints_rad": np.round(solution, 7).tolist(),
            "minimum_hard_limit_margin_rad": round(
                float(np.min(np.minimum(solution - LOWER, UPPER - solution))), 6
            ),
            "max_joint_delta_rad": round(float(np.max(np.abs(solution - previous))), 6),
            "max_joint_step_limit_rad": max_joint_step_rad,
            "jacobian_min_singular_value": round(minimum_singular_value, 8),
            "jacobian_condition_number": round(condition_number, 3),
            "solver": solver,
            "accepted": not reasons,
            "reasons": reasons,
        }
        planned.append(row)
        if reasons:
            failures.append({"waypoint": index, "reasons": reasons})
            break
        previous = solution

    passed = len(planned) == waypoints and not failures
    warnings = []
    if minimum is None:
        warnings.append(
            "workspace bounds are not configured; this plan cannot authorize motion"
        )
    if link_min_z_m is None:
        warnings.append(
            "distal-link minimum height is not configured; link clearance is unchecked"
        )
    return {
        "schema_version": 1,
        "kind": "piper_readonly_pose_plan",
        "status": "passed" if passed else "failed",
        "dry_run": True,
        "motion_authorized": False,
        "target": {
            "position_m": np.round(goal[:3, 3], 6).tolist(),
            "rpy_rad": np.round(rotation_to_rpy(goal[:3, :3]), 6).tolist(),
            "relative_input": relative,
        },
        "checks": {
            "joint_limits": True,
            "joint_waypoint_step": True,
            "straight_cartesian_path": True,
            "workspace_bounds": minimum is not None,
            "distal_link_min_z": link_min_z_m is not None,
        },
        "warnings": warnings,
        "waypoints_requested": waypoints,
        "waypoints_planned": len(planned),
        "waypoints": planned,
        "failures": failures,
        "target_joints_rad": planned[-1]["joints_rad"] if passed else None,
    }
