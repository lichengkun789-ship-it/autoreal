#!/usr/bin/env python3
"""Capture a passive Piper pose and optionally cross-check independent readings.

First capture the reported pose without a confirmation file. Then read the same joint
angles from an independent source, fill examples/piper_pose_confirmation.json, and run
again with --confirmation. This command never sends a CAN query or motion command.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.piper.pose_check import build_pose_check_receipt  # noqa: E402
from services.piper.probe import run_readonly_probe  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def _print_pose(receipt: dict) -> None:
    pose = receipt.get("measured_pose")
    if not pose:
        return
    print("\nReported pose (degrees):")
    print("arm    CAN          J1        J2        J3        J4        J5        J6    gripper")
    for side in ("left", "right"):
        arm = pose["arms"][side]
        joints = "  ".join(f"{value:8.3f}" for value in arm["joints_deg"])
        print(
            f"{side:5}  {receipt['can_names'][side]:8}  {joints}  "
            f"{arm['gripper_width_m'] * 1000:7.1f} mm"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend-options",
        type=Path,
        default=os.environ.get("RAX_BACKEND_OPTIONS") or None,
        help="Piper backend JSON (default: $RAX_BACKEND_OPTIONS)",
    )
    verification = parser.add_mutually_exclusive_group()
    verification.add_argument("--confirmation", type=Path, help="Independent operator readings JSON")
    verification.add_argument(
        "--skip-independent-confirmation",
        metavar="REASON",
        help="Record why no independent angle source is available; this does not mark the check passed",
    )
    parser.add_argument("--tolerance-deg", type=float, default=2.0)
    parser.add_argument("--stability-tolerance-deg", type=float, default=0.2)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.backend_options is None:
        parser.error("pass --backend-options or set RAX_BACKEND_OPTIONS")

    try:
        options = json.loads(args.backend_options.read_text())
        confirmation = json.loads(args.confirmation.read_text()) if args.confirmation else None
        probe = run_readonly_probe(options, samples=args.samples)
        can_names = {
            side: options["arms"][side]["can_name"] for side in ("left", "right")
        }
        receipt = build_pose_check_receipt(
            probe,
            can_names=can_names,
            confirmation=confirmation,
            skip_reason=args.skip_independent_confirmation,
            tolerance_deg=args.tolerance_deg,
            stability_tolerance_deg=args.stability_tolerance_deg,
        )
    except Exception as exc:
        receipt = {
            "schema_version": 1,
            "kind": "piper_pose_cross_check",
            "status": "failed",
            "can_queries_sent": False,
            "motion_commands_sent": False,
            "failure_reasons": [f"{type(exc).__name__}: {exc}"],
            "created_at": datetime.now(timezone.utc).isoformat(),
        }

    output = args.output
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = artifact_root() / "bringup" / "pose_checks" / f"piper_pose_{stamp}.json"
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(receipt, indent=2) + "\n")
    output.chmod(0o600)
    _print_pose(receipt)
    print(f"\nStatus: {receipt['status']}")
    print(f"Receipt: {output}")
    return 1 if receipt["status"] == "failed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
