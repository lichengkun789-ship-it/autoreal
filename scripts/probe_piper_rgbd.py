#!/usr/bin/env python3
"""Capture one read-only calibrated RGB-D observation from a Piper backend."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.backend import PiperBackend  # noqa: E402


def _default_artifact_root() -> Path:
    configured = os.environ.get("RAX_ARTIFACT_ROOT")
    return Path(configured).expanduser() if configured else PROJECT / "runtime"


def _motion_enabled(options: dict) -> bool:
    value = options.get("agent_motion")
    return isinstance(value, dict) and value.get("enabled") is True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend-options", type=Path, required=True)
    parser.add_argument("--artifact-root", type=Path, default=_default_artifact_root())
    args = parser.parse_args()

    options_path = args.backend_options.expanduser().resolve()
    options = json.loads(options_path.read_text())
    if not isinstance(options, dict):
        parser.error("backend options must be a JSON object")
    if _motion_enabled(options):
        parser.error("the RGB-D probe refuses configurations with agent motion enabled")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    root = args.artifact_root.expanduser().resolve() / "probes" / f"piper_rgbd_{stamp}"
    config = SimpleNamespace(
        root=root,
        backend_options=options,
        observation_profile="rgbd",
    )
    started_at = datetime.now(timezone.utc).isoformat()
    backend = None
    report = {
        "schema_version": 1,
        "kind": "piper_rgbd_readonly_probe",
        "status": "failed",
        "started_at": started_at,
        "backend_options": str(options_path),
        "agent_motion_enabled": False,
        "motion_commands_sent": False,
    }
    try:
        backend = PiperBackend(config)
        tool_names = [tool["name"] for tool in backend.tools()]
        if backend.mutating:
            raise RuntimeError(f"read-only probe exposed mutating tools: {sorted(backend.mutating)}")
        observation, previews = backend.call("piper_observe", {})
        cameras = observation.get("cameras", {})
        movement = observation.get("max_joint_movement_during_capture_rad")
        report.update(
            status=(
                "passed"
                if observation.get("ready")
                and cameras
                and all(item.get("depth_available") for item in cameras.values())
                and "piper_pixel_to_position" in tool_names
                and movement is not None
                and movement <= 0.002
                else "failed"
            ),
            finished_at=datetime.now(timezone.utc).isoformat(),
            frame_id=observation.get("frame_id"),
            frame_directory=str(root / "agent-workspace" / observation["directory"]),
            robot_ready=observation.get("ready"),
            readiness_reasons=observation.get("readiness_reasons", []),
            cameras=cameras,
            images_in_order=observation.get("images_in_order", []),
            preview_count=len(previews),
            max_joint_movement_during_capture_rad=movement,
            tool_surface={"tools": tool_names, "mutating_tools": []},
        )
    except Exception as exc:
        report.update(
            finished_at=datetime.now(timezone.utc).isoformat(),
            error={"type": type(exc).__name__, "message": str(exc)},
        )
    finally:
        if backend is not None:
            backend.close()

    receipt = root / "piper-rgbd-readonly-probe.json"
    write_private_json(receipt, report, directory_mode=0o700)
    report["receipt"] = str(receipt)
    print(json.dumps(report, indent=2))
    print(f"Receipt: {receipt}", file=sys.stderr)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
