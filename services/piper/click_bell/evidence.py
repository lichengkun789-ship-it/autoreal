"""Shared evidence loading for commissioned Piper ClickBell motion."""

import json
from pathlib import Path


MOTION_CONFIG_KEYS = (
    "arms",
    "joint_limit_margin_rad",
    "max_joint_step_rad",
    "workspace_bounds",
    "link_min_z_m",
)


def read_json(path: Path, label: str) -> tuple[dict, bytes]:
    try:
        raw = path.read_bytes()
        value = json.loads(raw)
    except (OSError, ValueError) as exc:
        raise ValueError(f"cannot read {label} {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return value, raw
