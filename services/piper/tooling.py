"""Commissioned Piper tool geometry owned by AutoReal."""

from __future__ import annotations


# Piper's URDF places both finger prismatic-joint origins 135.8 mm along the
# gripper-base tool axis. The project operator accepted that model as matching
# the installed hardware on 2026-10-30.
GRIPPER_CENTER_CONTACT_OFFSET_TCP_M = (0.0, 0.0, 0.1358)
GRIPPER_CENTER_CONTACT_GEOMETRY = {
    "frame": "piper_sdk_end_pose",
    "point": "closed_gripper_center",
    "offset_m": list(GRIPPER_CENTER_CONTACT_OFFSET_TCP_M),
    "source": "Piper URDF joint7/joint8 origin xyz",
    "operator_accepted": True,
    "accepted_on": "2026-10-30",
}
