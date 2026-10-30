"""Camera extrinsics and RGB-D point transforms for Piper observations."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Iterable

import numpy as np

from services.perception.geometry import deproject_pixel, transform_point

from .camera import CameraSpec


CALIBRATION_ENVIRONMENT = "PIPER_CALIBRATION_DIR"
MOUNTS = frozenset({"left_base", "right_base", "left_tcp", "right_tcp"})


def _calibration_path(value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    root = os.environ.get(CALIBRATION_ENVIRONMENT)
    if not root:
        raise ValueError(
            f"relative Piper extrinsics path {path} requires {CALIBRATION_ENVIRONMENT}"
        )
    return Path(root).expanduser() / path


def validate_extrinsics_matrix(value, *, context: str = "matrix") -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if (
        matrix.shape != (4, 4)
        or not np.all(np.isfinite(matrix))
        or not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0])
    ):
        raise ValueError(f"{context}: matrix must be a finite 4x4 homogeneous transform")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-4
    ):
        raise ValueError(f"{context}: matrix rotation must be orthonormal")
    return matrix


def load_extrinsics(value: str, *, expected_mount: str | None = None) -> np.ndarray:
    path = _calibration_path(value)
    document = json.loads(path.read_text())
    if document.get("convention") != "T_mount_camera":
        raise ValueError(f"{path}: convention must be T_mount_camera")
    declared_mount = document.get("mount")
    if expected_mount is not None and declared_mount not in {None, expected_mount}:
        raise ValueError(
            f"{path}: declared mount {declared_mount!r} does not match {expected_mount!r}"
        )
    return validate_extrinsics_matrix(document.get("matrix"), context=str(path))


@dataclass(frozen=True)
class CameraModel:
    name: str
    mount: str
    T_mount_camera: np.ndarray

    @property
    def base_frame(self) -> str:
        side = self.mount.split("_", 1)[0]
        return f"piper_{side}_base"

    @classmethod
    def from_spec(cls, spec: CameraSpec) -> "CameraModel | None":
        if spec.mount is None and spec.extrinsics is None:
            return None
        if spec.mount not in MOUNTS or not spec.extrinsics:
            raise ValueError(
                f"camera {spec.name}: mount and extrinsics must be configured together"
            )
        return cls(
            spec.name,
            spec.mount,
            load_extrinsics(spec.extrinsics, expected_mount=spec.mount),
        )

    def base_from_camera(self, arm_end_transforms: dict[str, np.ndarray]) -> np.ndarray:
        if self.mount.endswith("_base"):
            return self.T_mount_camera.copy()
        side = self.mount.removesuffix("_tcp")
        try:
            T_base_tcp = arm_end_transforms[side]
        except KeyError as exc:
            raise ValueError(f"camera {self.name}: missing {side} arm end pose") from exc
        return np.asarray(T_base_tcp, dtype=np.float64) @ self.T_mount_camera


def camera_models(specs: Iterable[CameraSpec]) -> dict[str, CameraModel]:
    models = {}
    for spec in specs:
        model = CameraModel.from_spec(spec)
        if model is not None:
            models[spec.name] = model
    return models
