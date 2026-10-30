"""Convert standalone Piper calibration results into AutoReal extrinsics evidence."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from .calibration import MOUNTS, validate_extrinsics_matrix


def _source_side(document: dict) -> str | None:
    value = document.get("arm_name")
    if value in {"left", "left_arm"}:
        return "left"
    if value in {"right", "right_arm"}:
        return "right"
    return None


def _quality_metrics(document: dict) -> dict:
    metrics = document.get("best_metrics") or {}
    return {
        key: value
        for key, value in metrics.items()
        if type(value) in {int, float}
        and any(token in key for token in ("mean", "rmse", "max"))
    }


def import_calibration_result(source: Path, *, mount: str) -> dict:
    """Build a standalone, read-only calibration candidate from a legacy result."""
    source = Path(source).expanduser().resolve()
    if mount not in MOUNTS:
        raise ValueError(f"mount must be one of {sorted(MOUNTS)}")
    raw = source.read_bytes()
    document = json.loads(raw)
    side = mount.split("_", 1)[0]
    source_side = _source_side(document)
    if source_side is not None and source_side != side:
        raise ValueError(
            f"source arm {document.get('arm_name')!r} does not match mount {mount!r}"
        )

    mode = document.get("calibration_mode")
    if mount.endswith("_tcp"):
        if mode != "eye_in_hand":
            raise ValueError("a TCP-mounted camera requires an eye_in_hand calibration result")
        source_transform = "supplemental_eye_in_hand.T_gripper_camera"
        matrix_value = (document.get("supplemental_eye_in_hand") or {}).get(
            "T_gripper_camera"
        )
    else:
        if mode == "eye_in_hand":
            raise ValueError("an eye_in_hand calibration cannot be imported as a fixed base camera")
        source_transform = "T_base_camera"
        matrix_value = document.get("T_base_camera")
    if matrix_value is None:
        raise ValueError(f"source is missing {source_transform}")
    matrix = validate_extrinsics_matrix(
        matrix_value, context=f"{source}:{source_transform}"
    )

    return {
        "schema_version": 1,
        "kind": "piper_camera_extrinsics_candidate",
        "status": "read_only_validation_required",
        "convention": "T_mount_camera",
        "mount": mount,
        "matrix": matrix.tolist(),
        "source": {
            "path": str(source),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "run_name": document.get("run_name"),
            "calibration_mode": mode or "eye_to_hand",
            "arm_name": document.get("arm_name"),
            "camera_name": document.get("camera_name"),
            "sample_count": document.get("sample_count"),
            "selected_method": document.get("selected_method"),
            "transform": source_transform,
        },
        "quality_metrics": _quality_metrics(document),
        "restrictions": {
            "motion_authorized": False,
            "requires_autoreal_validation": True,
        },
    }
