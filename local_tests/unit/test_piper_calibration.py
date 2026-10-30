"""Piper calibration tests cover fixed and eye-in-hand transform conventions."""

import json

import numpy as np
import pytest

from services.piper.calibration import CameraModel, deproject_pixel, load_extrinsics, transform_point
from services.piper.camera import CameraSpec


def write_extrinsics(path, matrix, convention="T_mount_camera"):
    path.write_text(json.dumps({"convention": convention, "matrix": matrix}))
    return path


def test_fixed_and_wrist_camera_models_use_direct_mount_to_camera_transform(tmp_path):
    matrix = np.eye(4)
    matrix[:3, 3] = [0.1, 0.2, 0.3]
    path = write_extrinsics(tmp_path / "camera.json", matrix.tolist())

    fixed = CameraModel.from_spec(CameraSpec("high", "1", "right_base", str(path)))
    assert fixed.base_from_camera({}) == pytest.approx(matrix)
    assert fixed.base_frame == "piper_right_base"

    wrist = CameraModel.from_spec(CameraSpec("wrist", "2", "right_tcp", str(path)))
    T_base_tcp = np.eye(4)
    T_base_tcp[0, 3] = 0.4
    assert wrist.base_from_camera({"right": T_base_tcp}) == pytest.approx(T_base_tcp @ matrix)
    assert wrist.base_frame == "piper_right_base"


def test_extrinsics_require_declared_convention_and_rigid_rotation(tmp_path):
    wrong = write_extrinsics(tmp_path / "wrong.json", np.eye(4).tolist(), "T_camera_mount")
    with pytest.raises(ValueError, match="convention"):
        load_extrinsics(str(wrong))
    matrix = np.eye(4)
    matrix[0, 0] = 2.0
    invalid = write_extrinsics(tmp_path / "invalid.json", matrix.tolist())
    with pytest.raises(ValueError, match="orthonormal"):
        load_extrinsics(str(invalid))


def test_declared_mount_must_match_camera_configuration(tmp_path):
    path = tmp_path / "camera.json"
    path.write_text(
        json.dumps(
            {
                "convention": "T_mount_camera",
                "mount": "left_tcp",
                "matrix": np.eye(4).tolist(),
            }
        )
    )
    with pytest.raises(ValueError, match="declared mount"):
        CameraModel.from_spec(CameraSpec("wrist", "2", "right_tcp", str(path)))


def test_zero_distortion_pixel_deprojection_and_base_transform():
    intrinsics = {
        "width": 640,
        "height": 480,
        "fx": 400.0,
        "fy": 400.0,
        "ppx": 320.0,
        "ppy": 240.0,
        "model": "distortion.none",
        "coeffs": [0.0] * 5,
    }
    point = deproject_pixel(360, 220, 0.5, intrinsics)
    assert point == pytest.approx([0.05, -0.025, 0.5])
    transform = np.eye(4)
    transform[:3, 3] = [0.1, 0.0, 0.2]
    assert transform_point(transform, point) == pytest.approx([0.15, -0.025, 0.7])


def test_pixel_deprojection_rejects_invalid_focal_length():
    intrinsics = {
        "width": 640,
        "height": 480,
        "fx": 0.0,
        "fy": 400.0,
        "ppx": 320.0,
        "ppy": 240.0,
    }
    with pytest.raises(ValueError, match="focal lengths"):
        deproject_pixel(320, 240, 0.5, intrinsics)
