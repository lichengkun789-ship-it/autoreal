"""Read-only eye-to-hand calibration for fixed Piper workspace cameras."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from .backend import PiperBackend
from .kinematics import end_transform
from .tag_validation import TAG_SIZE_M, _require_cv2, localize_saved_tag
from .wrist_calibration import (
    DEFAULT_HOLDOUT_COUNT,
    DEFAULT_MIN_ROTATION_SPAN_DEG,
    DEFAULT_MIN_TRANSLATION_SPAN_MM,
    _as_transform,
    _holdout_indices,
    _pose_diversity,
    _project_rotation,
    _transform_metrics,
)


DEFAULT_MIN_SAMPLES = 20


def _camera_options(path: Path, camera: str) -> tuple[dict, Path, str, str]:
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
    if not isinstance(mount, str) or not mount.endswith("_base"):
        raise ValueError("fixed-camera calibration requires a left_base or right_base mount")
    arm = mount.removesuffix("_base")
    if arm not in {"left", "right"}:
        raise ValueError("fixed-camera mount must identify the calibration arm base")
    if not selected[0].get("extrinsics"):
        raise ValueError("capture requires current candidate extrinsics for frame metadata")
    filtered = deepcopy(options)
    filtered["cameras"] = selected
    filtered["agent_motion"] = {"enabled": False}
    return filtered, resolved, mount, arm


def calibration_sample_from_frame(
    frame_directory: Path,
    camera: str,
    sample: dict,
    *,
    arm: str,
) -> dict:
    """Attach synchronized arm FK to one fixed-camera tag observation."""
    if arm not in {"left", "right"}:
        raise ValueError("calibration arm must be left or right")
    directory = Path(frame_directory).expanduser().resolve()
    metadata = json.loads((directory / "meta.json").read_text())
    camera_metadata = metadata.get("cameras", {}).get(camera)
    if camera_metadata is None:
        raise ValueError(f"frame has no camera {camera!r}")
    mount = camera_metadata.get("mount")
    if mount != f"{arm}_base":
        raise ValueError(f"camera {camera!r} mount does not match {arm} calibration arm")
    state = metadata.get("state_after", {}).get("arms", {}).get(arm)
    if state is None:
        raise ValueError(f"frame has no synchronized {arm} arm state")
    joints = np.asarray(state.get("joints_rad"), dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise ValueError(f"frame has invalid {arm} joint feedback")
    result = dict(sample)
    result.update(
        calibration_role="eye_to_hand_sample",
        arm=arm,
        mount=mount,
        joints_rad=np.round(joints, 9).tolist(),
        T_base_tcp=np.round(end_transform(joints), 9).tolist(),
        frame_directory=str(directory),
    )
    return result


def capture_fixed_camera_sample(
    *,
    sample_root: Path,
    backend_options: Path,
    camera: str,
    tag_size_m: float = TAG_SIZE_M,
    max_capture_movement_rad: float = 0.002,
) -> dict:
    """Capture one stationary eye-to-hand sample without exposing motion tools."""
    report = {
        "schema_version": 1,
        "kind": "piper_fixed_camera_eye_to_hand_capture",
        "status": "failed",
        "started_at": datetime.now(timezone.utc).isoformat(),
        "camera": camera,
        "motion_commands_sent": False,
        "motion_authorized": False,
    }
    backend = None
    try:
        options, options_path, mount, arm = _camera_options(backend_options, camera)
        report.update(backend_options=str(options_path), mount=mount, arm=arm)
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
            sample=calibration_sample_from_frame(
                frame_directory,
                camera,
                detected,
                arm=arm,
            ),
        )
    except Exception as exc:
        report["error"] = {"type": type(exc).__name__, "message": str(exc)}
    finally:
        if backend is not None:
            backend.close()
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
    return report


def load_fixed_camera_sample(path: Path) -> dict:
    receipt = Path(path).expanduser().resolve()
    document = json.loads(receipt.read_text())
    if document.get("status") != "passed":
        raise ValueError(f"capture receipt did not pass: {receipt}")
    sample = dict(document.get("sample") or {})
    if sample.get("status") != "passed":
        raise ValueError(f"AprilTag sample did not pass: {receipt}")
    if "T_base_tcp" not in sample:
        frame_directory = document.get("frame_directory") or sample.get("frame_directory")
        arm = document.get("arm") or sample.get("arm")
        if not frame_directory or arm not in {"left", "right"}:
            raise ValueError(f"receipt cannot recover synchronized arm FK: {receipt}")
        sample = calibration_sample_from_frame(
            Path(frame_directory), sample["camera"], sample, arm=arm
        )
    sample["source_receipt"] = str(receipt)
    return sample


def solve_fixed_camera_calibration(
    samples: list[dict],
    *,
    min_samples: int = DEFAULT_MIN_SAMPLES,
    holdout_count: int = DEFAULT_HOLDOUT_COUNT,
    translation_rmse_limit_mm: float = 5.0,
    translation_max_limit_mm: float = 10.0,
    min_translation_span_mm: float = DEFAULT_MIN_TRANSLATION_SPAN_MM,
    min_rotation_span_deg: float = DEFAULT_MIN_ROTATION_SPAN_DEG,
) -> dict:
    """Solve T_base_camera and validate fixed T_tcp_tag on untouched poses."""
    if min_samples < 4 or min_samples - holdout_count < 3:
        raise ValueError("minimum sample count must leave three training samples and one holdout")
    if len(samples) < 4:
        raise ValueError("at least four samples are required to run the solver")
    cameras = {sample.get("camera") for sample in samples}
    mounts = {sample.get("mount") for sample in samples}
    frames = {sample.get("base_frame") for sample in samples}
    arms = {sample.get("arm") for sample in samples}
    if any(len(values) != 1 for values in (cameras, mounts, frames, arms)):
        raise ValueError("all samples must use one camera, mount, base frame, and arm")
    mount = next(iter(mounts))
    arm = next(iter(arms))
    if mount != f"{arm}_base" or arm not in {"left", "right"}:
        raise ValueError("fixed camera mount and calibration arm do not match")
    frame_ids = [sample.get("frame_id") for sample in samples]
    if None in frame_ids or len(set(frame_ids)) != len(frame_ids):
        raise ValueError("every calibration sample must have a unique frame_id")

    holdout_indices = _holdout_indices(len(samples), holdout_count)
    holdout_set = set(holdout_indices)
    training_indices = [index for index in range(len(samples)) if index not in holdout_set]
    training = [samples[index] for index in training_indices]
    holdout = [samples[index] for index in holdout_indices]
    diversity = _pose_diversity(training)
    training_base_tcp = [
        _as_transform(sample["T_base_tcp"], context="training T_base_tcp")
        for sample in training
    ]
    training_camera_tag = [
        _as_transform(sample["T_camera_tag"], context="training T_camera_tag")
        for sample in training
    ]
    training_tcp_base = [np.linalg.inv(transform) for transform in training_base_tcp]

    cv2 = _require_cv2()
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    method_results = {}
    for name, method in methods.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                [item[:3, :3] for item in training_tcp_base],
                [item[:3, 3] for item in training_tcp_base],
                [item[:3, :3] for item in training_camera_tag],
                [item[:3, 3] for item in training_camera_tag],
                method=method,
            )
            transform = np.eye(4, dtype=np.float64)
            transform[:3, :3] = _project_rotation(rotation, context=name)
            transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
            if not np.all(np.isfinite(transform)) or np.linalg.norm(transform[:3, 3]) > 3.0:
                raise ValueError(f"{name} returned an implausible transform")
            tcp_tag = [
                tcp_base @ transform @ camera_tag
                for tcp_base, camera_tag in zip(training_tcp_base, training_camera_tag)
            ]
            metrics, reference = _transform_metrics(tcp_tag)
            method_results[name] = {
                "status": "passed",
                "matrix": np.round(transform, 9).tolist(),
                "training_metrics": metrics,
                "T_tcp_tag_reference": np.round(reference, 9).tolist(),
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
    reference = np.asarray(selected["T_tcp_tag_reference"], dtype=np.float64)
    holdout_tcp_tag = [
        np.linalg.inv(_as_transform(sample["T_base_tcp"], context="holdout T_base_tcp"))
        @ transform
        @ _as_transform(sample["T_camera_tag"], context="holdout T_camera_tag")
        for sample in holdout
    ]
    holdout_metrics, _ = _transform_metrics(holdout_tcp_tag, reference=reference)

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
        "kind": "piper_fixed_camera_eye_to_hand_calibration",
        "status": "passed" if not failure_reasons else "failed",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "convention": "T_mount_camera",
        "camera": next(iter(cameras)),
        "mount": mount,
        "base_frame": next(iter(frames)),
        "arm": arm,
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
