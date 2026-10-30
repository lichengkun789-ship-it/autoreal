#!/usr/bin/env python3
"""Capture or evaluate read-only AprilTag evidence for Piper camera extrinsics."""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
from types import SimpleNamespace
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.backend import PiperBackend  # noqa: E402
from services.piper.tag_validation import evaluate_samples, localize_saved_tag  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _load_options(path: Path, camera: str) -> tuple[dict, Path]:
    resolved = path.expanduser().resolve()
    options = json.loads(resolved.read_text())
    if not isinstance(options, dict):
        raise ValueError("backend options must be a JSON object")
    motion = options.get("agent_motion") or {}
    if isinstance(motion, dict) and motion.get("enabled") is True:
        raise PermissionError("AprilTag capture refuses agent_motion.enabled=true")
    selected = [item for item in options.get("cameras", []) if item.get("name") == camera]
    if len(selected) != 1:
        raise ValueError(f"backend options must contain exactly one camera named {camera!r}")
    if not selected[0].get("mount") or not selected[0].get("extrinsics"):
        raise ValueError(f"camera {camera!r} requires mount and extrinsics")
    filtered = deepcopy(options)
    filtered["cameras"] = selected
    filtered["agent_motion"] = {"enabled": False}
    return filtered, resolved


def capture(args) -> tuple[dict, Path]:
    root = artifact_root() / "calibration_validation" / f"apriltag_{_stamp()}"
    receipt = root / "tag_sample.json"
    report = {
        "schema_version": 1,
        "kind": "piper_apriltag_extrinsics_capture",
        "status": "failed",
        "started_at": datetime.now(timezone.utc).isoformat(),
        "camera": args.camera,
        "motion_commands_sent": False,
        "motion_authorized": False,
    }
    backend = None
    try:
        options, options_path = _load_options(args.backend_options, args.camera)
        report["backend_options"] = str(options_path)
        config = SimpleNamespace(root=root, backend_options=options, observation_profile="rgbd")
        backend = PiperBackend(config)
        if backend.mutating:
            raise RuntimeError(f"capture exposed mutating tools: {sorted(backend.mutating)}")
        observation, _ = backend.call("piper_observe", {"cameras": [args.camera]})
        movement = observation.get("max_joint_movement_during_capture_rad")
        frame_directory = root / "agent-workspace" / observation["directory"]
        report.update(
            frame_directory=str(frame_directory),
            robot_ready=observation.get("ready"),
            readiness_reasons=observation.get("readiness_reasons", []),
            max_joint_movement_during_capture_rad=movement,
        )
        if observation.get("ready") is not True:
            reasons = "; ".join(observation.get("readiness_reasons", []))
            raise RuntimeError("Piper feedback was not ready during capture: " + reasons)
        if movement is None or movement > args.max_capture_movement_rad:
            raise RuntimeError(
                f"arm moved {movement} rad during capture; limit is "
                f"{args.max_capture_movement_rad} rad"
            )
        sample = localize_saved_tag(
            frame_directory,
            args.camera,
            overlay_path=root / "tag_overlay.jpg",
            tag_size_m=args.tag_size_mm / 1000.0,
        )
        report.update(
            status="passed",
            sample=sample,
        )
    except Exception as exc:
        report["error"] = {"type": type(exc).__name__, "message": str(exc)}
    finally:
        if backend is not None:
            backend.close()
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        write_private_json(receipt, report, directory_mode=0o700)
    return report, receipt


def evaluate(args) -> tuple[dict, Path]:
    samples = []
    for path in args.receipts:
        document = json.loads(path.expanduser().resolve().read_text())
        sample = document.get("sample", document)
        samples.append(sample)
    report = evaluate_samples(
        samples,
        min_samples=args.min_samples,
        translation_rmse_limit_mm=args.rmse_limit_mm,
        translation_max_limit_mm=args.max_limit_mm,
    )
    output = args.output or (
        artifact_root() / "calibration_validation" / f"apriltag_evaluation_{_stamp()}.json"
    )
    output = output.expanduser().resolve()
    write_private_json(output, report, directory_mode=0o700)
    return report, output


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture", help="Capture one stationary sample")
    capture_parser.add_argument("--backend-options", type=Path, required=True)
    capture_parser.add_argument("--camera", default="cam_right_wrist")
    capture_parser.add_argument("--tag-size-mm", type=float, default=100.0)
    capture_parser.add_argument("--max-capture-movement-rad", type=float, default=0.002)
    evaluate_parser = commands.add_parser("evaluate", help="Evaluate multiple capture receipts")
    evaluate_parser.add_argument("receipts", type=Path, nargs="+")
    evaluate_parser.add_argument("--output", type=Path)
    evaluate_parser.add_argument("--min-samples", type=int, default=5)
    evaluate_parser.add_argument("--rmse-limit-mm", type=float, default=5.0)
    evaluate_parser.add_argument("--max-limit-mm", type=float, default=10.0)
    return root


def main() -> int:
    args = parser().parse_args()
    report, receipt = capture(args) if args.command == "capture" else evaluate(args)
    print(json.dumps(report, indent=2))
    print(f"Receipt: {receipt}", file=sys.stderr)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
