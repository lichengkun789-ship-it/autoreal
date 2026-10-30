"""Read-only AprilTag evidence for validating Piper camera extrinsics."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


TAG_DICTIONARY = "DICT_APRILTAG_36h11"
TAG_ID = 0
TAG_SIZE_M = 0.1


def _require_cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "AprilTag validation requires opencv-contrib-python-headless; "
            "install requirements-robot.txt"
        ) from exc
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, TAG_DICTIONARY):
        raise RuntimeError("installed OpenCV does not provide the AprilTag ArUco dictionary")
    return cv2


def _camera_matrix(intrinsics: dict) -> tuple[np.ndarray, np.ndarray]:
    values = [float(intrinsics[name]) for name in ("fx", "fy", "ppx", "ppy")]
    if not all(math.isfinite(value) for value in values) or values[0] <= 0 or values[1] <= 0:
        raise ValueError("camera intrinsics must contain finite positive focal lengths")
    matrix = np.asarray(
        [[values[0], 0.0, values[2]], [0.0, values[1], values[3]], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    coefficients = np.asarray(intrinsics.get("coeffs", [0.0] * 5), dtype=np.float64)
    if coefficients.shape != (5,) or not np.all(np.isfinite(coefficients)):
        raise ValueError("camera distortion coefficients must contain five finite values")
    return matrix, coefficients


def _object_points(tag_size_m: float) -> np.ndarray:
    half = tag_size_m / 2.0
    return np.asarray(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )


def detect_tag(image_rgb: np.ndarray, intrinsics: dict, *, tag_size_m: float = TAG_SIZE_M):
    """Return tag pose evidence and an annotated RGB image."""
    if tag_size_m <= 0:
        raise ValueError("tag_size_m must be positive")
    image = np.asarray(image_rgb, dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("AprilTag detection requires an RGB image")

    cv2 = _require_cv2()
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, TAG_DICTIONARY))
    parameters = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "CORNER_REFINE_APRILTAG"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector_profile = "apriltag_refine"
    corners_list, ids, _ = cv2.aruco.ArucoDetector(
        dictionary, parameters
    ).detectMarkers(gray)
    detected_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    if TAG_ID not in detected_ids:
        # Dense canonical-cell sampling is more stable for the high camera's
        # oblique views. Keep the dictionary and accepted ID unchanged.
        parameters = cv2.aruco.DetectorParameters()
        parameters.perspectiveRemovePixelPerCell = 8
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector_profile = "perspective_dense_subpix"
        corners_list, ids, _ = cv2.aruco.ArucoDetector(
            dictionary, parameters
        ).detectMarkers(gray)
        detected_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    if TAG_ID not in detected_ids:
        raise ValueError(f"AprilTag id {TAG_ID} not detected; detected ids: {detected_ids}")

    index = detected_ids.index(TAG_ID)
    corners = np.asarray(corners_list[index], dtype=np.float64).reshape(4, 2)
    camera_matrix, distortion = _camera_matrix(intrinsics)
    solved, rotation_vector, translation_vector = cv2.solvePnP(
        _object_points(tag_size_m),
        corners,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not solved:
        raise RuntimeError("OpenCV could not solve the AprilTag pose")

    projected, _ = cv2.projectPoints(
        _object_points(tag_size_m),
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion,
    )
    projected = projected.reshape(4, 2)
    reprojection_error = float(
        np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=1)))
    )
    rotation, _ = cv2.Rodrigues(rotation_vector)
    T_camera_tag = np.eye(4, dtype=np.float64)
    T_camera_tag[:3, :3] = rotation
    T_camera_tag[:3, 3] = np.asarray(translation_vector).reshape(3)

    annotated = image.copy()
    cv2.aruco.drawDetectedMarkers(annotated, [corners.astype(np.float32).reshape(1, 4, 2)])
    cv2.drawFrameAxes(
        annotated,
        camera_matrix,
        distortion,
        rotation_vector,
        translation_vector,
        tag_size_m * 0.5,
        2,
    )
    return {
        "dictionary": TAG_DICTIONARY,
        "tag_id": TAG_ID,
        "tag_size_m": tag_size_m,
        "detector_profile": detector_profile,
        "corners_px": np.round(corners, 3).tolist(),
        "reprojection_error_px": round(reprojection_error, 6),
        "T_camera_tag": np.round(T_camera_tag, 9).tolist(),
    }, annotated


def localize_saved_tag(
    frame_directory: str | Path,
    camera: str,
    *,
    overlay_path: str | Path | None = None,
    tag_size_m: float = TAG_SIZE_M,
) -> dict:
    directory = Path(frame_directory)
    metadata = json.loads((directory / "meta.json").read_text())
    camera_metadata = metadata.get("cameras", {}).get(camera)
    if camera_metadata is None:
        raise ValueError(f"frame has no camera {camera!r}")
    if "T_base_camera" not in camera_metadata or "base_frame" not in camera_metadata:
        raise ValueError(f"camera {camera!r} has no base-frame extrinsics")

    image = np.asarray(Image.open(directory / camera_metadata["rgb"]).convert("RGB"))
    detection, annotated = detect_tag(
        image,
        camera_metadata["intrinsics"],
        tag_size_m=tag_size_m,
    )
    T_base_camera = np.asarray(camera_metadata["T_base_camera"], dtype=np.float64)
    T_camera_tag = np.asarray(detection["T_camera_tag"], dtype=np.float64)
    T_base_tag = T_base_camera @ T_camera_tag
    result = {
        "schema_version": 1,
        "kind": "piper_apriltag_extrinsics_sample",
        "status": "passed",
        "frame_id": metadata.get("frame_id"),
        "camera": camera,
        "base_frame": camera_metadata["base_frame"],
        "mount": camera_metadata.get("mount"),
        **detection,
        "position_m": np.round(T_base_tag[:3, 3], 6).tolist(),
        "T_base_tag": np.round(T_base_tag, 9).tolist(),
    }
    if overlay_path is not None:
        path = Path(overlay_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(annotated).save(path)
        result["overlay"] = str(path.resolve())
    return result


def _mean_rotation(rotations: np.ndarray) -> np.ndarray:
    left, _, right = np.linalg.svd(rotations.sum(axis=0))
    mean = left @ right
    if np.linalg.det(mean) < 0:
        left[:, -1] *= -1
        mean = left @ right
    return mean


def _rotation_error_deg(reference: np.ndarray, rotation: np.ndarray) -> float:
    value = (np.trace(reference.T @ rotation) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(value, -1.0, 1.0))))


def evaluate_samples(
    samples: list[dict],
    *,
    min_samples: int = 5,
    translation_rmse_limit_mm: float = 5.0,
    translation_max_limit_mm: float = 10.0,
) -> dict:
    if min_samples < 2:
        raise ValueError("min_samples must be at least 2")
    passed_samples = [sample for sample in samples if sample.get("status") == "passed"]
    if not passed_samples:
        raise ValueError("no passed AprilTag samples were provided")
    cameras = {sample.get("camera") for sample in passed_samples}
    frames = {sample.get("base_frame") for sample in passed_samples}
    if len(cameras) != 1 or len(frames) != 1:
        raise ValueError("all samples must use one camera and one base frame")

    transforms = np.asarray([sample["T_base_tag"] for sample in passed_samples], dtype=np.float64)
    if transforms.shape != (len(passed_samples), 4, 4):
        raise ValueError("every sample must contain one 4x4 T_base_tag")
    positions = transforms[:, :3, 3]
    centroid = positions.mean(axis=0)
    translation_errors = np.linalg.norm(positions - centroid, axis=1) * 1000.0
    translation_rmse = float(np.sqrt(np.mean(translation_errors**2)))
    translation_max = float(translation_errors.max())
    mean_rotation = _mean_rotation(transforms[:, :3, :3])
    rotation_errors = np.asarray(
        [_rotation_error_deg(mean_rotation, rotation) for rotation in transforms[:, :3, :3]]
    )
    enough = len(passed_samples) >= min_samples
    within_limits = (
        translation_rmse <= translation_rmse_limit_mm
        and translation_max <= translation_max_limit_mm
    )
    return {
        "schema_version": 1,
        "kind": "piper_apriltag_extrinsics_evaluation",
        "status": "passed" if enough and within_limits else "failed",
        "camera": next(iter(cameras)),
        "base_frame": next(iter(frames)),
        "sample_count": len(passed_samples),
        "minimum_sample_count": min_samples,
        "centroid_position_m": np.round(centroid, 6).tolist(),
        "translation_rmse_mm": round(translation_rmse, 3),
        "translation_max_mm": round(translation_max, 3),
        "translation_errors_mm": np.round(translation_errors, 3).tolist(),
        "rotation_rmse_deg": round(float(np.sqrt(np.mean(rotation_errors**2))), 4),
        "rotation_max_deg": round(float(rotation_errors.max()), 4),
        "limits": {
            "translation_rmse_mm": translation_rmse_limit_mm,
            "translation_max_mm": translation_max_limit_mm,
        },
        "failure_reasons": [
            reason
            for condition, reason in (
                (not enough, f"need at least {min_samples} samples"),
                (
                    translation_rmse > translation_rmse_limit_mm,
                    "translation RMSE exceeds the commissioning limit",
                ),
                (
                    translation_max > translation_max_limit_mm,
                    "maximum translation error exceeds the commissioning limit",
                ),
            )
            if condition
        ],
        "motion_authorized": False,
    }
