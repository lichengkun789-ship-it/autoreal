"""AprilTag extrinsics validation stays read-only and hardware-free."""

import json

import numpy as np
from PIL import Image
import pytest

from services.piper.tag_validation import evaluate_samples, localize_saved_tag


cv2 = pytest.importorskip("cv2")


def _sample(transform, index=0):
    return {
        "status": "passed",
        "camera": "cam_right_wrist",
        "base_frame": "piper_right_base",
        "frame_id": str(index),
        "T_base_tag": transform.tolist(),
    }


def test_localizes_generated_tag_in_saved_frame(tmp_path):
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        0,
        200,
    )
    image = np.full((480, 640), 255, dtype=np.uint8)
    image[140:340, 220:420] = marker
    Image.fromarray(image).convert("RGB").save(tmp_path / "camera.png")
    metadata = {
        "frame_id": "frame1",
        "cameras": {
            "cam_right_wrist": {
                "rgb": "camera.png",
                "base_frame": "piper_right_base",
                "mount": "right_tcp",
                "T_base_camera": np.eye(4).tolist(),
                "intrinsics": {
                    "fx": 600.0,
                    "fy": 600.0,
                    "ppx": 320.0,
                    "ppy": 240.0,
                    "coeffs": [0.0] * 5,
                },
            }
        },
    }
    (tmp_path / "meta.json").write_text(json.dumps(metadata))

    result = localize_saved_tag(
        tmp_path,
        "cam_right_wrist",
        overlay_path=tmp_path / "overlay.jpg",
    )

    assert result["status"] == "passed"
    assert result["tag_id"] == 0
    assert result["position_m"] == pytest.approx([0.0, 0.0, 0.3], abs=0.003)
    assert (tmp_path / "overlay.jpg").is_file()


def test_evaluation_requires_enough_consistent_samples():
    samples = []
    for index, offset_mm in enumerate((-2.0, -1.0, 0.0, 1.0, 2.0)):
        transform = np.eye(4)
        transform[:3, 3] = [0.4 + offset_mm / 1000.0, 0.1, 0.05]
        samples.append(_sample(transform, index))

    passed = evaluate_samples(samples)
    assert passed["status"] == "passed"
    assert passed["translation_max_mm"] <= 2.0

    transform = np.eye(4)
    transform[:3, 3] = [0.45, 0.1, 0.05]
    failed = evaluate_samples([*samples[:-1], _sample(transform, 9)])
    assert failed["status"] == "failed"
    assert failed["motion_authorized"] is False
