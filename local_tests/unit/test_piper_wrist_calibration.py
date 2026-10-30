"""Piper wrist calibration solves only from synchronized read-only evidence."""

import json
import math

import numpy as np
import pytest

from services.piper.kinematics import end_transform
from services.piper.wrist_calibration import (
    calibration_sample_from_frame,
    solve_wrist_calibration,
)


cv2 = pytest.importorskip("cv2")


def _rotation(rx, ry, rz):
    sx, cx = math.sin(rx), math.cos(rx)
    sy, cy = math.sin(ry), math.cos(ry)
    sz, cz = math.sin(rz), math.cos(rz)
    x = np.asarray([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    y = np.asarray([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    z = np.asarray([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return z @ y @ x


def _transform(translation, rpy):
    result = np.eye(4)
    result[:3, :3] = _rotation(*rpy)
    result[:3, 3] = translation
    return result


def _inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -transform[:3, :3].T @ transform[:3, 3]
    return result


def _synthetic_samples(count=20):
    expected = _transform([-0.09, 0.004, 0.025], [0.02, -0.34, -1.53])
    base_tag = _transform([0.42, -0.20, 0.03], [0.01, -0.02, 1.57])
    samples = []
    for index in range(count):
        phase = index / max(1, count - 1)
        base_tcp = _transform(
            [
                0.18 + 0.18 * phase,
                -0.22 + 0.13 * math.sin(index * 0.7),
                0.22 + 0.09 * math.cos(index * 0.5),
            ],
            [
                -0.55 + 1.05 * phase,
                -0.40 + 0.65 * math.sin(index * 0.41),
                -1.0 + 1.8 * math.cos(index * 0.37),
            ],
        )
        camera_tag = _inverse(expected) @ _inverse(base_tcp) @ base_tag
        samples.append(
            {
                "status": "passed",
                "frame_id": f"frame-{index:02d}",
                "camera": "cam_left_wrist",
                "mount": "left_tcp",
                "base_frame": "piper_left_base",
                "T_base_tcp": base_tcp.tolist(),
                "T_camera_tag": camera_tag.tolist(),
                "source_receipt": f"/evidence/{index:02d}.json",
            }
        )
    return expected, samples


def test_solver_recovers_known_transform_on_held_out_poses():
    expected, samples = _synthetic_samples()

    result = solve_wrist_calibration(samples)

    assert result["status"] == "passed"
    assert result["sample_count"] == 20
    assert len(result["holdout_indices"]) == 5
    assert result["holdout_metrics"]["translation_rmse_mm"] < 0.01
    assert np.asarray(result["matrix"]) == pytest.approx(expected, abs=1e-6)
    assert result["restrictions"]["motion_authorized"] is False
    assert result["restrictions"]["automatic_activation"] is False


def test_solver_rejects_duplicate_frame_evidence():
    _, samples = _synthetic_samples()
    samples[-1]["frame_id"] = samples[0]["frame_id"]

    with pytest.raises(ValueError, match="unique frame_id"):
        solve_wrist_calibration(samples)


def test_frame_sample_uses_synchronized_joint_fk(tmp_path):
    joints = [0.1, 0.2, -0.3, 0.4, -0.2, 0.5]
    metadata = {
        "frame_id": "frame-1",
        "state_after": {"arms": {"left": {"joints_rad": joints}}},
        "cameras": {"cam_left_wrist": {"mount": "left_tcp"}},
    }
    (tmp_path / "meta.json").write_text(json.dumps(metadata))
    detected = {
        "status": "passed",
        "camera": "cam_left_wrist",
        "frame_id": "frame-1",
        "base_frame": "piper_left_base",
        "T_camera_tag": np.eye(4).tolist(),
    }

    sample = calibration_sample_from_frame(tmp_path, "cam_left_wrist", detected)

    assert np.asarray(sample["T_base_tcp"]) == pytest.approx(end_transform(joints), abs=1e-9)
    assert sample["arm"] == "left"
    assert sample["calibration_role"] == "eye_in_hand_sample"
