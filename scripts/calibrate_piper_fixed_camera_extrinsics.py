#!/usr/bin/env python3
"""Manage read-only Piper fixed-camera eye-to-hand calibration runs."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.fixed_camera_calibration import (  # noqa: E402
    DEFAULT_HOLDOUT_COUNT,
    DEFAULT_MIN_ROTATION_SPAN_DEG,
    DEFAULT_MIN_SAMPLES,
    DEFAULT_MIN_TRANSLATION_SPAN_MM,
    capture_fixed_camera_sample,
    load_fixed_camera_sample,
    solve_fixed_camera_calibration,
)
from services.storage_root import artifact_root, require_artifact_path  # noqa: E402


def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _manifest_path(value: Path) -> Path:
    path = value.expanduser().resolve()
    if path.is_dir():
        path = path / "manifest.json"
    require_artifact_path(path, "Calibration run")
    if not path.is_file():
        raise FileNotFoundError(f"calibration manifest not found: {path}")
    return path


def _load_manifest(value: Path) -> tuple[dict, Path]:
    path = _manifest_path(value)
    manifest = json.loads(path.read_text())
    if manifest.get("kind") != "piper_fixed_camera_eye_to_hand_run":
        raise ValueError(f"not a Piper fixed-camera calibration manifest: {path}")
    return manifest, path


def initialize(args) -> tuple[dict, Path]:
    if args.tag_size_mm <= 0:
        raise ValueError("tag size must be positive")
    if args.max_capture_movement_rad < 0:
        raise ValueError("maximum capture movement must be non-negative")
    if args.min_samples < 4 or args.min_samples - args.holdout_count < 3:
        raise ValueError(
            "minimum sample count must leave at least three training samples and one holdout"
        )
    backend_options = args.backend_options.expanduser().resolve()
    options = json.loads(backend_options.read_text())
    cameras = [item for item in options.get("cameras", []) if item.get("name") == args.camera]
    if len(cameras) != 1:
        raise ValueError(f"backend options must contain exactly one camera named {args.camera!r}")
    mount = cameras[0].get("mount")
    if not isinstance(mount, str) or not mount.endswith("_base"):
        raise ValueError("calibration camera must be mounted on left_base or right_base")
    arm = mount.removesuffix("_base")
    if arm not in {"left", "right"}:
        raise ValueError("calibration camera mount does not identify a Piper arm")
    motion = options.get("agent_motion") or {}
    if isinstance(motion, dict) and motion.get("enabled") is True:
        raise PermissionError("calibration init refuses agent_motion.enabled=true")

    run = args.output or (
        artifact_root()
        / "calibration"
        / "piper"
        / "eye_to_hand"
        / f"{_stamp()}_{args.camera}"
    )
    run = require_artifact_path(run.expanduser().resolve(), "Calibration run")
    manifest_path = run / "manifest.json"
    if manifest_path.exists():
        raise FileExistsError(f"calibration run already exists: {run}")
    manifest = {
        "schema_version": 1,
        "kind": "piper_fixed_camera_eye_to_hand_run",
        "status": "collecting",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "run_directory": str(run),
        "backend_options": str(backend_options),
        "camera": args.camera,
        "mount": mount,
        "arm": arm,
        "tag_dictionary": "DICT_APRILTAG_36h11",
        "tag_id": 0,
        "tag_size_mm": args.tag_size_mm,
        "max_capture_movement_rad": args.max_capture_movement_rad,
        "minimum_samples": args.min_samples,
        "holdout_count": args.holdout_count,
        "capture_attempts": [],
        "samples": [],
        "motion_commands_sent": False,
        "motion_authorized": False,
        "physical_setup": "tag rigidly attached to calibration arm TCP; fixed camera unmoved",
    }
    write_private_json(manifest_path, manifest, directory_mode=0o700)
    return manifest, manifest_path


def capture(args) -> tuple[dict, Path]:
    manifest, manifest_path = _load_manifest(args.run)
    run = manifest_path.parent
    attempt = len(manifest.get("capture_attempts", [])) + 1
    sample_root = run / "samples" / f"{attempt:04d}"
    receipt = sample_root / "receipt.json"
    report = capture_fixed_camera_sample(
        sample_root=sample_root,
        backend_options=Path(manifest["backend_options"]),
        camera=manifest["camera"],
        tag_size_m=float(manifest["tag_size_mm"]) / 1000.0,
        max_capture_movement_rad=float(manifest["max_capture_movement_rad"]),
    )
    report["run_manifest"] = str(manifest_path)
    report["attempt"] = attempt
    write_private_json(receipt, report, directory_mode=0o700)
    manifest.setdefault("capture_attempts", []).append(
        {"attempt": attempt, "receipt": str(receipt), "status": report["status"]}
    )
    if report["status"] == "passed":
        manifest.setdefault("samples", []).append(str(receipt))
    manifest["sample_count"] = len(manifest.get("samples", []))
    manifest["updated_at"] = datetime.now(timezone.utc).isoformat()
    write_private_json(manifest_path, manifest, directory_mode=0o700)
    return report, receipt


def solve(args) -> tuple[dict, Path]:
    manifest, manifest_path = _load_manifest(args.run)
    samples = [load_fixed_camera_sample(Path(path)) for path in manifest.get("samples", [])]
    report = solve_fixed_camera_calibration(
        samples,
        min_samples=int(manifest["minimum_samples"]),
        holdout_count=int(manifest["holdout_count"]),
        translation_rmse_limit_mm=args.rmse_limit_mm,
        translation_max_limit_mm=args.max_limit_mm,
        min_translation_span_mm=args.min_translation_span_mm,
        min_rotation_span_deg=args.min_rotation_span_deg,
    )
    report["run_manifest"] = str(manifest_path)
    output = manifest_path.parent / "calibration_candidate.json"
    write_private_json(output, report, directory_mode=0o700)
    manifest["latest_candidate"] = str(output)
    manifest["latest_candidate_status"] = report["status"]
    manifest["status"] = "candidate_ready" if report["status"] == "passed" else "collecting"
    manifest["updated_at"] = datetime.now(timezone.utc).isoformat()
    write_private_json(manifest_path, manifest, directory_mode=0o700)
    return report, output


def status(args) -> tuple[dict, Path]:
    manifest, path = _load_manifest(args.run)
    report = {
        "run_directory": str(path.parent),
        "status": manifest.get("status"),
        "camera": manifest.get("camera"),
        "mount": manifest.get("mount"),
        "arm": manifest.get("arm"),
        "sample_count": len(manifest.get("samples", [])),
        "minimum_samples": manifest.get("minimum_samples"),
        "holdout_count": manifest.get("holdout_count"),
        "latest_candidate": manifest.get("latest_candidate"),
        "latest_candidate_status": manifest.get("latest_candidate_status"),
        "motion_commands_sent": False,
        "motion_authorized": False,
    }
    return report, path


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)

    init_parser = commands.add_parser("init", help="Create a calibration run")
    init_parser.add_argument("--backend-options", type=Path, required=True)
    init_parser.add_argument("--camera", default="cam_high")
    init_parser.add_argument("--output", type=Path)
    init_parser.add_argument("--tag-size-mm", type=float, default=100.0)
    init_parser.add_argument("--max-capture-movement-rad", type=float, default=0.002)
    init_parser.add_argument("--min-samples", type=int, default=DEFAULT_MIN_SAMPLES)
    init_parser.add_argument("--holdout-count", type=int, default=DEFAULT_HOLDOUT_COUNT)

    capture_parser = commands.add_parser("capture", help="Capture one stationary sample")
    capture_parser.add_argument("--run", type=Path, required=True)

    solve_parser = commands.add_parser("solve", help="Solve and holdout-test the run")
    solve_parser.add_argument("--run", type=Path, required=True)
    solve_parser.add_argument("--rmse-limit-mm", type=float, default=5.0)
    solve_parser.add_argument("--max-limit-mm", type=float, default=10.0)
    solve_parser.add_argument(
        "--min-translation-span-mm",
        type=float,
        default=DEFAULT_MIN_TRANSLATION_SPAN_MM,
    )
    solve_parser.add_argument(
        "--min-rotation-span-deg",
        type=float,
        default=DEFAULT_MIN_ROTATION_SPAN_DEG,
    )

    status_parser = commands.add_parser("status", help="Show calibration progress")
    status_parser.add_argument("--run", type=Path, required=True)
    return root


def main() -> int:
    args = parser().parse_args()
    try:
        if args.command == "init":
            report, receipt = initialize(args)
        elif args.command == "capture":
            report, receipt = capture(args)
        elif args.command == "solve":
            report, receipt = solve(args)
        else:
            report, receipt = status(args)
    except Exception as exc:
        report = {
            "status": "failed",
            "error": {"type": type(exc).__name__, "message": str(exc)},
            "motion_commands_sent": False,
            "motion_authorized": False,
        }
        receipt = None
    print(json.dumps(report, indent=2))
    if receipt is not None:
        print(f"Receipt: {receipt}", file=sys.stderr)
    return 0 if report.get("status") in {"passed", "collecting", "candidate_ready"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
