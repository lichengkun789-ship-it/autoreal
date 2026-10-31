"""Piper forward kinematics in the robot base frame.

The modified-DH constants match the offset model used by ``piper_sdk==0.6.2``.
Translations are metres and rotations are radians throughout this module.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


_A = np.asarray([0.0, 0.0, 0.28503, -0.02198, 0.0, 0.0])
_ALPHA = np.asarray([0.0, -math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2])
_THETA_OFFSET = np.radians([0.0, -172.22, -102.78, 0.0, 0.0, 0.0])
_D = np.asarray([0.123, 0.0, 0.0, 0.25075, 0.0, 0.091])


def _link_transform(alpha: float, a: float, theta: float, d: float) -> np.ndarray:
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.asarray(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -sa * d],
            [st * sa, ct * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def joint_frames(joints_rad: Iterable[float]) -> tuple[np.ndarray, ...]:
    """Return base-frame transforms after joints 1 through 6."""
    joints = np.asarray(tuple(joints_rad), dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise ValueError("Piper FK requires six finite joint angles")
    transform = np.eye(4, dtype=np.float64)
    frames = []
    for alpha, a, theta, offset, d in zip(_ALPHA, _A, joints, _THETA_OFFSET, _D):
        transform = transform @ _link_transform(alpha, a, theta + offset, d)
        frames.append(transform.copy())
    return tuple(frames)


def end_transform(joints_rad: Iterable[float]) -> np.ndarray:
    """Pose of the Piper SDK end-pose frame in the arm base frame."""
    return joint_frames(joints_rad)[-1]


def rotation_to_rpy(rotation: np.ndarray) -> np.ndarray:
    """XYZ roll, pitch and yaw matching the Piper SDK FK convention."""
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation must be a 3x3 matrix")
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(abs(pitch) - math.pi / 2) < 1e-4:
        yaw = 0.0
        roll = (
            math.atan2(rotation[0, 1], rotation[1, 1])
            if pitch > 0
            else -math.atan2(rotation[0, 1], rotation[1, 1])
        )
    else:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.asarray([roll, pitch, yaw], dtype=np.float64)


def end_pose(joints_rad: Iterable[float]) -> tuple[np.ndarray, np.ndarray]:
    transform = end_transform(joints_rad)
    return transform[:3, 3].copy(), rotation_to_rpy(transform[:3, :3])


def pose_report(joints_rad: Iterable[float]) -> dict:
    position, rpy = end_pose(joints_rad)
    return {
        "frame": "piper_sdk_end_pose",
        "position_m": np.round(position, 6).tolist(),
        "rpy_rad": np.round(rpy, 6).tolist(),
    }


def tool_pose_report(
    joints_rad: Iterable[float], tool_offset_tcp_m: Iterable[float]
) -> dict:
    """Pose of a configured tool point with the SDK end-frame orientation."""
    transform = end_transform(joints_rad)
    offset = np.asarray(tuple(tool_offset_tcp_m), dtype=np.float64)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)):
        raise ValueError("tool offset must be a finite 3-D vector")
    position = transform[:3, 3] + transform[:3, :3] @ offset
    return {
        "frame": "piper_configured_tool_point",
        "position_m": np.round(position, 6).tolist(),
        "rpy_rad": np.round(rotation_to_rpy(transform[:3, :3]), 6).tolist(),
        "offset_tcp_m": np.round(offset, 6).tolist(),
    }
