"""Operator-owned motion authorization shared by every real robot backend."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time


def arm_file(session_root: str | Path) -> Path:
    return Path(session_root) / "operator" / "armed.json"


def read(session_root: str | Path, now: float | None = None) -> dict:
    """Return arming state; missing, invalid, or expired state is disarmed."""
    path = arm_file(session_root)
    try:
        value = json.loads(path.read_text())
    except (OSError, ValueError):
        return {"armed": False, "reason": "not armed by the operator"}

    current_time = time.time() if now is None else now
    until = value.get("until")
    if value.get("armed") is not True:
        return {"armed": False, "reason": value.get("reason") or "disarmed by the operator"}
    if until is not None and current_time >= until:
        return {"armed": False, "reason": "operator arming expired"}
    return {
        "armed": True,
        "until": until,
        "by": value.get("by"),
        "since": value.get("since"),
    }


def write(
    session_root: str | Path,
    armed: bool,
    *,
    minutes: float | None = None,
    by: str | None = None,
    reason: str | None = None,
) -> dict:
    path = arm_file(session_root)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    now = time.time()
    value = {
        "armed": bool(armed),
        "since": now,
        "by": by or os.environ.get("USER"),
        "until": now + 60 * minutes if armed and minutes else None,
        "reason": reason,
    }
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value))
    temporary.chmod(0o600)
    os.replace(temporary, path)
    return value
