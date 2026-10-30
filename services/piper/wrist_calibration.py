"""Read-only eye-in-hand calibration for Piper wrist cameras."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from .backend import PiperBackend
from .calibration import validate_extrinsics_matrix
from .kinematics import end_transform
from .tag_validation import TAG_SIZE_M, _require_cv2, localize_saved_tag


DEFAULT_MIN_SAMPLES = 15
DEFAULT_HOLDOUT_COUNT = 5
DEFAULT_MIN_TRANSLATION_SPAN_MM = 50.0
DEFAULT_MIN_ROTATION_SPAN_DEG = 20.0


def _as_transform(value, *, context: str) -> np.ndarray:
    return validate_extrinsics_matrix(value, context=context).astype(np.float64, copy=True)


def _rotation_error_deg(reference: np.ndarray, rotation: np.ndarray) -> float:
    value = (np.trace(reference.T @ rotation) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(value, -1.0, 1.0))))


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    left, _, right = np.linalg.svd(np.asarray(rotations).sum(axis=0))
    mean = left @ right
    if np.linalg.det(mean) < 0:
        left[:, -1] *= -1
        mean = left @ right
    return mean


def _mean_transform(transforms: list[np.ndarray]) -> np.ndarray:
    mean = np.eye(4, dtype=np.float64)
    mean[:3, :3] = _mean_rotation([transform[:3, :3] for transform in transforms])
    mean[:3, 3] = np.mean([transform[:3, 3] for transform in transforms], axis=0)
    return mean


def _project_rotation(rotation: np.ndarray, *, context: str) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(rotation)):
        raise ValueError(f"{context} returned non-finite rotation values")
    left, _, right = np.linalg.svd(rotation)
    projected = left @ right
    if np.linalg.det(projected) < 0:
        left[:, -1] *= -1
        projected = left @ right
    if np.linalg.det(projected) <= 0:
        raise ValueError(f"{context} did not return a proper rotation")
    return projected


def _transform_metrics(
    base_to_tag: list[np.ndarray],
    *,
    reference: np.ndarray | None = None,
) -> tuple[dict, np.ndarray]:
    if not base_to_tag:
        raise ValueError("at least one tag transform is required")
    reference = _mean_transform(base_to_tag) if reference is None else reference
    translation_errors = np.asarray(
        [np.linalg.norm(item[:3, 3] - reference[:3, 3]) * 1000.0 for item in base_to_tag]
    )
    rotation_errors = np.asarray(
        [_rotation_error_deg(reference[:3, :3], item[:3, :3]) for item in base_to_tag]
    )
    return {
        "sample_count": len(base_to_tag),
        "translation_rmse_mm": round(float(np.sqrt(np.mean(translation_errors**2))), 3),
        "translation_max_mm": round(float(translation_errors.max()), 3),
        "translation_errors_mm": np.round(translation_errors, 3).tolist(),
        "rotation_rmse_deg": round(float(np.sqrt(np.mean(rotation_errors**2))), 4),
        "rotation_max_deg": round(float(rotation_errors.max()), 4),
        "rotation_errors_deg": np.round(rotation_errors, 4).tolist(),
    }, reference


def _pose_diversity(samples: list[dict]) -> dict:
    transforms = [
        _as_transform(sample["T_base_tcp"], context="sample T_base_tcp")
        for sample in samples
    ]
    positions = np.asarray([transform[:3, 3] for transform in transforms])
    span_xyz = np.ptp(positions, axis=0) * 1000.0
    rotation_span = 0.0
    for index, first in enumerate(transforms):
        for second in transforms[index + 1 :]:
            rotation_span = max(
                rotation_span,
                _rotation_error_deg(first[:3, :3], second[:3, :3]),
            )
    return {
        "translation_span_xyz_mm": np.round(span_xyz, 3).tolist(),
        "translation_span_norm_mm": round(float(np.linalg.norm(span_xyz)), 3),
        "maximum_pairwise_rotation_deg": round(rotation_span, 4),
    }


def _camera_options(path: Path, camera: str) -> tuple[dict, Path, str]:
    resolved = path.expanduser().resolve()
    options = json.loads(resolved.read_text())
    if not isinstance(options, dict):
        raise ValueError("backend options must be a JSON object")
    motion = options.get("agent_motion") or {}
    if isinstance(motion, dict) and motion.get("enabled") is True:
        raise PermissionError("calibration capture refuses agent_motion.enabled=true")
    selected = [item for item in options.get("cameras", []) if item.get("name") == camera]
    if len(selected) != 1:
        raise ValueError(f"backend options must contain exactly one camera named {camera!r}")
    mount = selected[0].get("mount")
    if not isinstance(mount, str) or not mount.endswith("_tcp"):
        raise ValueError("wrist calibration requires a camera mounted on left_tcp or right_tcp")
    if not selected[0].get("extrinsics"):
        raise ValueError("capture requires the current candidate extrinsics for frame metadata")
    filtered = deepcopy(options)
    filtered["cameras"] = selected
    filtered["agent_motion"] = {"enabled": False}
    return filtered, resolved, mount


def calibration_sample_from_frame(frame_directory: Path, camera: str, sample: dict) -> dict:
    """Attach synchronized FK evidence needed by the hand-eye solver."""
    directory = Path(frame_directory).expanduser().resolve()
    metadata = json.loads((directory / "meta.json").read_text())
    camera_metadata = metadata.get("cameras", {}).get(camera)
    if camera_metadata is None:
        raise ValueError(f"frame has no camera {camera!r}")
    mount = camera_metadata.get("mount")
    if not isinstance(mount, str) or not mount.endswith("_tcp"):
        raise ValueError(f"camera {camera!r} is not a wrist-mounted camera")
    side = mount.removesuffix("_tcp")
    state = metadata.get("state_after", {}).get("arms", {}).get(side)
    if state is None:
        raise ValueError(f"frame has no synchronized {side} arm state")
    joints = np.asarray(state.get("joints_rad"), dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise ValueError(f"frame has invalid {side} joint feedback")
    result = dict(sample)
    result.update(
        calibration_role="eye_in_hand_sample",
        arm=side,
        mount=mount,
        joints_rad=np.round(joints, 9).tolist(),
        T_base_tcp=np.round(end_transform(joints), 9).tolist(),
        frame_directory=str(directory),
    )
    return result


def capture_wrist_sample(
    *,
    sample_root: Path,
    backend_options: Path,
    camera: str,
    tag_size_m: float = TAG_SIZE_M,
    max_capture_movement_rad: float = 0.002,
) -> dict:
    """Capture one stationary wrist-calibration sample without exposing motion tools."""
    report = {
        "schema_version": 1,
        "kind": "piper_wrist_hand_eye_capture",
        "status": "failed",
        "started_at": datetime.now(timezone.utc).isoformat(),
        "camera": camera,
        "motion_commands_sent": False,
        "motion_authorized": False,
    }
    backend = None
    try:
        options, options_path, mount = _camera_options(backend_options, camera)
        report.update(backend_options=str(options_path), mount=mount)
        config = SimpleNamespace(
            root=Path(sample_root),
            backend_options=options,
            observation_profile="rgbd",
        )
        backend = PiperBackend(config)
        if backend.mutating:
            raise RuntimeError(f"capture exposed mutating tools: {sorted(backend.mutating)}")
        observation, _ = backend.call("piper_observe", {"cameras": [camera]})
        movement = observation.get("max_joint_movement_during_capture_rad")
        frame_directory = Path(sample_root) / "agent-workspace" / observation["directory"]
        report.update(
            frame_directory=str(frame_directory.resolve()),
            robot_ready=observation.get("ready"),
            readiness_reasons=observation.get("readiness_reasons", []),
            max_joint_movement_during_capture_rad=movement,
        )
        if observation.get("ready") is not True:
            reasons = "; ".join(observation.get("readiness_reasons", []))
            raise RuntimeError("Piper feedback was not ready during capture: " + reasons)
        if movement is None or movement > max_capture_movement_rad:
            raise RuntimeError(
                f"arm moved {movement} rad during capture; limit is "
                f"{max_capture_movement_rad} rad"
            )
        detected = localize_saved_tag(
            frame_directory,
            camera,
            overlay_path=Path(sample_root) / "tag_overlay.jpg",
            tag_size_m=tag_size_m,
        )
        report.update(
            status="passed",
            sample=calibration_sample_from_frame(frame_directory, camera, detected),
        )
    except Exception as exc:
        report["error"] = {"type": type(exc).__name__, "message": str(exc)}
    finally:
        if backend is not None:
            backend.close()
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
    return report


def load_wrist_sample(path: Path) -> dict:
    """Load a new receipt or upgrade a validation receipt using its saved metadata."""
    receipt = Path(path).expanduser().resolve()
    document = json.loads(receipt.read_text())
    if document.get("status") != "passed":
        raise ValueError(f"capture receipt did not pass: {receipt}")
    sample = dict(document.get("sample") or {})
    if sample.get("status") != "passed":
        raise ValueError(f"AprilTag sample did not pass: {receipt}")
    if "T_base_tcp" not in sample:
        frame_directory = document.get("frame_directory") or sample.get("frame_directory")
        if not frame_directory:
            raise ValueError(f"receipt has no frame directory for FK recovery: {receipt}")
        sample = calibration_sample_from_frame(Path(frame_directory), sample["camera"], sample)
    sample["source_receipt"] = str(receipt)
    return sample


def _holdout_indices(sample_count: int, holdout_count: int) -> list[int]:
    if holdout_count < 1 or sample_count - holdout_count < 3:
        raise ValueError("holdout split requires at least three training samples and one holdout")
    return sorted(
        {
            int(round(value))
            for value in np.linspace(0, sample_count - 1, num=holdout_count)
        }
    )


def solve_wrist_calibration(
    samples: list[dict],
    *,
    min_samples: int = DEFAULT_MIN_SAMPLES,
    holdout_count: int = DEFAULT_HOLDOUT_COUNT,
    translation_rmse_limit_mm: float = 5.0,
    translation_max_limit_mm: float = 10.0,
    min_translation_span_mm: float = DEFAULT_MIN_TRANSLATION_SPAN_MM,
    min_rotation_span_deg: float = DEFAULT_MIN_ROTATION_SPAN_DEG,
) -> dict:
    """Solve T_tcp_camera on training poses and evaluate untouched holdout poses."""
    if min_samples < 4:
        raise ValueError("min_samples must be at least four")
    if min_samples - holdout_count < 3:
        raise ValueError("minimum sample count must leave at least three training samples")
    if len(samples) < 4:
        raise ValueError("at least four samples are required to run the solver")
    cameras = {sample.get("camera") for sample in samples}
    mounts = {sample.get("mount") for sample in samples}
    frames = {sample.get("base_frame") for sample in samples}
    if len(cameras) != 1 or len(mounts) != 1 or len(frames) != 1:
        raise ValueError("all samples must use one camera, mount, and base frame")
    mount = next(iter(mounts))
    if not isinstance(mount, str) or not mount.endswith("_tcp"):
        raise ValueError("all samples must come from one wrist-mounted camera")
    frame_ids = [sample.get("frame_id") for sample in samples]
    if None in frame_ids or len(set(frame_ids)) != len(frame_ids):
        raise ValueError("every calibration sample must have a unique frame_id")

    holdout_indices = _holdout_indices(len(samples), holdout_count)
    holdout_set = set(holdout_indices)
    training_indices = [index for index in range(len(samples)) if index not in holdout_set]
    training = [samples[index] for index in training_indices]
    holdout = [samples[index] for index in holdout_indices]
    diversity = _pose_diversity(training)

    cv2 = _require_cv2()
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    training_base_tcp = [
        _as_transform(sample["T_base_tcp"], context="training T_base_tcp")
        for sample in training
    ]
    training_camera_tag = [
        _as_transform(sample["T_camera_tag"], context="training T_camera_tag")
        for sample in training
    ]
    method_results = {}
    for name, method in methods.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                [item[:3, :3] for item in training_base_tcp],
                [item[:3, 3] for item in training_base_tcp],
                [item[:3, :3] for item in training_camera_tag],
                [item[:3, 3] for item in training_camera_tag],
                method=method,
            )
            transform = np.eye(4, dtype=np.float64)
            transform[:3, :3] = _project_rotation(rotation, context=name)
            transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
            if not np.all(np.isfinite(transform)) or np.linalg.norm(transform[:3, 3]) > 1.0:
                raise ValueError(f"{name} returned an implausible transform")
            base_tag = [
                base_tcp @ transform @ camera_tag
                for base_tcp, camera_tag in zip(training_base_tcp, training_camera_tag)
            ]
            metrics, reference = _transform_metrics(base_tag)
            method_results[name] = {
                "status": "passed",
                "matrix": np.round(transform, 9).tolist(),
                "training_metrics": metrics,
                "T_base_tag_reference": np.round(reference, 9).tolist(),
                "selection_score": round(
                    metrics["translation_rmse_mm"] + metrics["rotation_rmse_deg"], 6
                ),
            }
        except (ValueError, np.linalg.LinAlgError, cv2.error) as exc:
            method_results[name] = {
                "status": "failed",
                "error": {"type": type(exc).__name__, "message": str(exc)},
            }

    successful = [
        (name, result)
        for name, result in method_results.items()
        if result.get("status") == "passed"
    ]
    if not successful:
        raise RuntimeError("all OpenCV hand-eye methods failed")
    selected_method, selected = min(
        successful,
        key=lambda item: (
            item[1]["selection_score"],
            item[1]["training_metrics"]["translation_rmse_mm"],
        ),
    )
    transform = np.asarray(selected["matrix"], dtype=np.float64)
    reference = np.asarray(selected["T_base_tag_reference"], dtype=np.float64)
    holdout_base_tag = [
        _as_transform(sample["T_base_tcp"], context="holdout T_base_tcp")
        @ transform
        @ _as_transform(sample["T_camera_tag"], context="holdout T_camera_tag")
        for sample in holdout
    ]
    holdout_metrics, _ = _transform_metrics(holdout_base_tag, reference=reference)

    failure_reasons = []
    if len(samples) < min_samples:
        failure_reasons.append(f"need at least {min_samples} samples")
    if diversity["translation_span_norm_mm"] < min_translation_span_mm:
        failure_reasons.append("training poses do not have enough translation diversity")
    if diversity["maximum_pairwise_rotation_deg"] < min_rotation_span_deg:
        failure_reasons.append("training poses do not have enough rotation diversity")
    if holdout_metrics["translation_rmse_mm"] > translation_rmse_limit_mm:
        failure_reasons.append("holdout translation RMSE exceeds the commissioning limit")
    if holdout_metrics["translation_max_mm"] > translation_max_limit_mm:
        failure_reasons.append("holdout maximum translation error exceeds the commissioning limit")

    return {
        "schema_version": 1,
        "kind": "piper_wrist_hand_eye_calibration",
        "status": "passed" if not failure_reasons else "failed",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "convention": "T_mount_camera",
        "camera": next(iter(cameras)),
        "mount": mount,
        "base_frame": next(iter(frames)),
        "matrix": np.round(transform, 9).tolist(),
        "sample_count": len(samples),
        "minimum_sample_count": min_samples,
        "training_indices": training_indices,
        "holdout_indices": holdout_indices,
        "selected_method": selected_method,
        "training_pose_diversity": diversity,
        "training_metrics": selected["training_metrics"],
        "holdout_metrics": holdout_metrics,
        "limits": {
            "translation_rmse_mm": translation_rmse_limit_mm,
            "translation_max_mm": translation_max_limit_mm,
            "minimum_training_translation_span_mm": min_translation_span_mm,
            "minimum_training_rotation_span_deg": min_rotation_span_deg,
        },
        "failure_reasons": failure_reasons,
        "all_methods": method_results,
        "source_receipts": [sample.get("source_receipt") for sample in samples],
        "restrictions": {
            "motion_authorized": False,
            "automatic_activation": False,
            "requires_static_target_validation": True,
        },
    }
