import json

import numpy as np
from PIL import Image, ImageDraw
import pytest
from types import SimpleNamespace

from services.task_tools import compose_task_tools
from tasks.click_bell.evaluator import evaluate
from tasks.click_bell.perception import detect_press_cap, localize_saved_frame


def bell_image():
    image = Image.new("RGB", (160, 120), (220, 220, 215))
    draw = ImageDraw.Draw(image)
    draw.ellipse((48, 34, 112, 102), fill=(30, 150, 205))
    draw.ellipse((68, 28, 92, 44), fill=(220, 185, 65))
    draw.ellipse((43, 92, 117, 110), fill=(210, 170, 55))
    return image


def test_detects_top_cap_instead_of_yellow_base_ring():
    result = detect_press_cap(bell_image())
    json.dumps(result)
    assert result["confidence"] == "high"
    assert result["press_pixel"] == pytest.approx([80, 36], abs=1)
    assert result["press_cap"]["centroid_px"][1] < result["bell"]["centroid_px"][1]


def test_rejects_blue_object_without_yellow_press_cap():
    image = Image.new("RGB", (80, 80), "white")
    ImageDraw.Draw(image).ellipse((20, 20, 60, 65), fill=(30, 150, 205))
    with pytest.raises(ValueError, match="no yellow press cap"):
        detect_press_cap(image)


def test_localizes_press_cap_from_saved_rgbd_evidence(tmp_path):
    bell_image().save(tmp_path / "right.jpg")
    Image.fromarray(np.full((120, 160), 500, dtype=np.uint16)).save(tmp_path / "right_depth.png")
    (tmp_path / "meta.json").write_text(
        json.dumps(
            {
                "frame_id": "frame1",
                "cameras": {
                    "cam_right_wrist": {
                        "rgb": "right.jpg",
                        "depth": "right_depth.png",
                        "base_frame": "piper_right_base",
                        "T_base_camera": np.eye(4).tolist(),
                        "intrinsics": {
                            "width": 160,
                            "height": 120,
                            "fx": 100.0,
                            "fy": 100.0,
                            "ppx": 80.0,
                            "ppy": 60.0,
                            "model": "distortion.none",
                            "coeffs": [0.0] * 5,
                        },
                    }
                },
            }
        )
    )

    result = localize_saved_frame(tmp_path, "cam_right_wrist")

    assert result["status"] == "passed"
    assert result["frame"] == "piper_right_base"
    assert result["depth_m"] == 0.5
    assert result["valid_fraction"] == 1.0
    assert result["position_m"] == pytest.approx([0.0, -0.12, 0.5], abs=0.01)
    assert result["surface"]["status"] == "passed"
    assert result["surface"]["method"] == "eroded_yellow_component_robust_plane"
    assert result["surface"]["valid_depth_samples"] >= 12
    assert result["surface"]["plane_rmse_mm"] == pytest.approx(0.0, abs=1e-6)
    assert result["surface"]["normal_base"] == pytest.approx([0.0, 0.0, -1.0])
    assert "excludes_camera_extrinsic_error" in result["surface"]["uncertainty_scope"]


def test_surface_fit_rejects_depth_outliers(tmp_path):
    bell_image().save(tmp_path / "right.jpg")
    depth = np.full((120, 160), 500, dtype=np.uint16)
    depth[34:39, 78:83] = 850
    Image.fromarray(depth).save(tmp_path / "right_depth.png")
    (tmp_path / "meta.json").write_text(
        json.dumps(
            {
                "frame_id": "frame1",
                "cameras": {
                    "cam_right_wrist": {
                        "rgb": "right.jpg",
                        "depth": "right_depth.png",
                        "base_frame": "piper_right_base",
                        "T_base_camera": np.eye(4).tolist(),
                        "intrinsics": {
                            "width": 160,
                            "height": 120,
                            "fx": 100.0,
                            "fy": 100.0,
                            "ppx": 80.0,
                            "ppy": 60.0,
                            "model": "distortion.none",
                            "coeffs": [0.0] * 5,
                        },
                    }
                },
            }
        )
    )

    result = localize_saved_frame(tmp_path, "cam_right_wrist")

    assert result["surface"]["inlier_depth_samples"] < result["surface"]["valid_depth_samples"]
    assert result["surface"]["position_m"][2] == pytest.approx(0.5, abs=0.005)


def test_evaluator_requires_visual_success_evidence():
    assert evaluate({})["status"] == "unverified"
    assert evaluate({"visual_success": True})["status"] == "passed"
    assert evaluate({"visual_success": False})["status"] == "failed"


def test_task_tool_registry_does_not_attach_piper_tools_to_other_robots():
    class Backend:
        closed = False

        def close(self):
            self.closed = True

    backend = Backend()
    config = SimpleNamespace(task="click_bell", robot="mock", observation_profile="rgbd")
    assert compose_task_tools(config, backend) is backend
    assert backend.closed is False
