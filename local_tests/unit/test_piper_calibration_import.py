"""Legacy calibration imports remain explicit, standalone, and read-only."""

import json

import pytest

from services.piper.calibration_import import import_calibration_result


IDENTITY = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def write_result(path, **updates):
    value = {
        "run_name": "run",
        "arm_name": "left_arm",
        "camera_name": "camera",
        "sample_count": 12,
        "selected_method": "HORAUD",
        "T_base_camera": IDENTITY,
        "best_metrics": {"reprojection_error_rmse_px": 0.5, "large_array": []},
    }
    value.update(updates)
    path.write_text(json.dumps(value))
    return path


def test_imports_fixed_base_calibration_with_provenance(tmp_path):
    source = write_result(tmp_path / "result.json")
    value = import_calibration_result(source, mount="left_base")
    assert value["convention"] == "T_mount_camera"
    assert value["matrix"] == IDENTITY
    assert value["status"] == "read_only_validation_required"
    assert value["restrictions"]["motion_authorized"] is False
    assert value["source"]["transform"] == "T_base_camera"
    assert len(value["source"]["sha256"]) == 64
    assert value["quality_metrics"] == {"reprojection_error_rmse_px": 0.5}


def test_imports_eye_in_hand_transform_for_matching_tcp(tmp_path):
    source = write_result(
        tmp_path / "result.json",
        calibration_mode="eye_in_hand",
        supplemental_eye_in_hand={"T_gripper_camera": IDENTITY},
    )
    value = import_calibration_result(source, mount="left_tcp")
    assert value["matrix"] == IDENTITY
    assert value["source"]["transform"] == "supplemental_eye_in_hand.T_gripper_camera"


def test_import_rejects_wrong_arm_or_calibration_mode(tmp_path):
    source = write_result(tmp_path / "fixed.json")
    with pytest.raises(ValueError, match="does not match"):
        import_calibration_result(source, mount="right_base")
    with pytest.raises(ValueError, match="eye_in_hand"):
        import_calibration_result(source, mount="left_tcp")

    wrist = write_result(
        tmp_path / "wrist.json",
        calibration_mode="eye_in_hand",
        supplemental_eye_in_hand={"T_gripper_camera": IDENTITY},
    )
    with pytest.raises(ValueError, match="fixed base"):
        import_calibration_result(wrist, mount="left_base")
