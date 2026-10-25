#!/usr/bin/env python3
"""List attached RealSense cameras without opening streams or connecting to Piper CAN.

    .venv/bin/python scripts/list_piper_cameras.py
"""

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.piper.camera import list_realsense_devices  # noqa: E402


def main() -> int:
    try:
        devices = list_realsense_devices()
        value = {"status": "ok", "device_count": len(devices), "devices": devices}
    except Exception as exc:
        value = {
            "status": "failed",
            "device_count": 0,
            "devices": [],
            "error": {"type": type(exc).__name__, "message": str(exc)},
        }
    print(json.dumps(value, indent=2))
    return 0 if value["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
