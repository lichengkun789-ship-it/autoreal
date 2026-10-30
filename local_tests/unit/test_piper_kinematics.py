"""Piper geometry tests use reference values from piper_sdk 0.6.2."""

import math

import numpy as np
import pytest

from services.piper.kinematics import end_pose, joint_frames


def test_zero_pose_matches_piper_sdk_offset_fk():
    position, rpy = end_pose([0.0] * 6)
    assert position == pytest.approx([0.0561275122, 0.0, 0.2132662681], abs=1e-9)
    assert rpy == pytest.approx([0.0, math.radians(85.0), 0.0], abs=1e-9)


def test_nonzero_pose_matches_piper_sdk_offset_fk():
    position, rpy = end_pose([0.1, 0.2, -0.3, 0.4, -0.2, 0.5])
    assert position == pytest.approx([0.0581038412, -0.0012457775, 0.3181643684], abs=1e-9)
    assert rpy == pytest.approx(np.radians([64.0160437, 34.2255706, 75.6384049]), abs=1e-8)


def test_fk_rejects_invalid_joint_vectors():
    with pytest.raises(ValueError, match="six finite"):
        joint_frames([0.0] * 5)
    with pytest.raises(ValueError, match="six finite"):
        joint_frames([0.0, 0.0, 0.0, 0.0, 0.0, float("nan")])
