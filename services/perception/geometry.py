"""Camera geometry shared by robot backends and task perception."""

from __future__ import annotations

from typing import Iterable

import numpy as np


def _require_realsense():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError(
            "non-zero RealSense distortion requires pyrealsense2"
        ) from exc
    return rs


def deproject_pixel(u: float, v: float, depth_m: float, intrinsics: dict) -> np.ndarray:
    """Convert an aligned color pixel and metric depth to a camera-frame point."""
    values = [u, v, depth_m]
    if not all(np.isfinite(float(item)) for item in values) or depth_m <= 0:
        raise ValueError("pixel and depth must be finite, with positive depth")
    required = {"width", "height", "fx", "fy", "ppx", "ppy"}
    if not required.issubset(intrinsics):
        raise ValueError(f"camera intrinsics are missing {sorted(required - set(intrinsics))}")
    intrinsic_values = {
        name: float(intrinsics[name]) for name in ("fx", "fy", "ppx", "ppy")
    }
    if not all(np.isfinite(value) for value in intrinsic_values.values()):
        raise ValueError("camera intrinsics must be finite")
    if intrinsic_values["fx"] <= 0 or intrinsic_values["fy"] <= 0:
        raise ValueError("camera focal lengths must be positive")
    coeffs = [float(item) for item in intrinsics.get("coeffs", [])]
    model = str(intrinsics.get("model", "")).rsplit(".", 1)[-1].lower()
    if not coeffs or not any(abs(item) > 1e-12 for item in coeffs) or model in {"none", "0"}:
        return np.asarray(
            [
                (float(u) - intrinsic_values["ppx"]) / intrinsic_values["fx"] * depth_m,
                (float(v) - intrinsic_values["ppy"]) / intrinsic_values["fy"] * depth_m,
                depth_m,
            ],
            dtype=np.float64,
        )

    rs = _require_realsense()
    distortion = getattr(rs.distortion, model, None)
    if distortion is None:
        raise ValueError(f"unsupported RealSense distortion model {intrinsics.get('model')!r}")
    native = rs.intrinsics()
    for name in ("width", "height"):
        setattr(native, name, int(intrinsics[name]))
    for name in ("fx", "fy", "ppx", "ppy"):
        setattr(native, name, float(intrinsics[name]))
    native.model = distortion
    native.coeffs = coeffs
    return np.asarray(
        rs.rs2_deproject_pixel_to_point(native, [float(u), float(v)], float(depth_m)),
        dtype=np.float64,
    )


def transform_point(transform: np.ndarray, point: Iterable[float]) -> np.ndarray:
    matrix = np.asarray(transform, dtype=np.float64)
    vector = np.asarray(tuple(point), dtype=np.float64)
    if matrix.shape != (4, 4) or vector.shape != (3,):
        raise ValueError("point transform requires a 4x4 transform and a 3-D point")
    return (matrix @ np.append(vector, 1.0))[:3]
