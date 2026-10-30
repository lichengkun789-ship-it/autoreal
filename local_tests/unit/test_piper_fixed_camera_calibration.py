"""Fixed Piper cameras solve eye-to-hand extrinsics from held-out evidence."""

import json
import math

import numpy as np
import pytest

from services.piper.fixed_camera_calibration import (
    calibration_sample_from_frame,
    solve_fixed_camera_calibration,
)
from services.piper.kinematics import end_transform


pytest.importorskip("cv2")


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


def _synthetic_samples(count=24):
    expected = _transform([-0.03, -0.25, 0.64], [3.0, -0.35, -1.72])
    tcp_tag = _transform([0.0, 0.0, 0.08], [0.0, 0.0, 0.0])
    samples = []
    for index in range(count):
        phase = index / max(1, count - 1)
        base_tcp = _transform(
            [
                0.18 + 0.20 * phase,
                -0.24 + 0.15 * math.sin(index * 0.61),
                0.20 + 0.10 * math.cos(index * 0.47),
            ],
            [
                -0.65 + 1.15 * phase,
                -0.45 + 0.72 * math.sin(index * 0.39),
                -1.1 + 1.9 * math.cos(index * 0.33),
            ],
        )
        camera_tag = _inverse(expected) @ base_tcp @ tcp_tag
        samples.append(
            {
                "status": "passed",
                "frame_id": f"frame-{index:02d}",
                "camera": "cam_high",
                "mount": "left_base",
                "arm": "left",
                "base_frame": "piper_left_base",
                "T_base_tcp": base_tcp.tolist(),
                "T_camera_tag": camera_tag.tolist(),
                "source_receipt": f"/evidence/{index:02d}.json",
            }
        )
    return expected, samples


def test_solver_recovers_fixed_camera_transform_on_held_out_poses():
    expected, samples = _synthetic_samples()

    result = solve_fixed_camera_calibration(samples)

    assert result["status"] == "passed"
    assert result["sample_count"] == 24
    assert len(result["holdout_indices"]) == 5
    assert result["holdout_metrics"]["translation_rmse_mm"] < 0.01
    assert np.asarray(result["matrix"]) == pytest.approx(expected, abs=1e-6)
    assert result["restrictions"]["motion_authorized"] is False
    assert result["restrictions"]["automatic_activation"] is False


def test_solver_rejects_mismatched_fixed_camera_mount():
    _, samples = _synthetic_samples()
    samples[-1]["mount"] = "right_base"

    with pytest.raises(ValueError, match="one camera, mount, base frame, and arm"):
        solve_fixed_camera_calibration(samples)


def test_frame_sample_uses_selected_arm_fk(tmp_path):
    joints = [0.1, 0.2, -0.3, 0.4, -0.2, 0.5]
    metadata = {
        "frame_id": "frame-1",
        "state_after": {"arms": {"left": {"joints_rad": joints}}},
        "cameras": {"cam_high": {"mount": "left_base"}},
    }
    (tmp_path / "meta.json").write_text(json.dumps(metadata))
    detected = {
        "status": "passed",
        "camera": "cam_high",
        "frame_id": "frame-1",
        "base_frame": "piper_left_base",
        "T_camera_tag": np.eye(4).tolist(),
    }

    sample = calibration_sample_from_frame(tmp_path, "cam_high", detected, arm="left")

    assert np.asarray(sample["T_base_tcp"]) == pytest.approx(end_transform(joints), abs=1e-9)
    assert sample["arm"] == "left"
    assert sample["calibration_role"] == "eye_to_hand_sample"
