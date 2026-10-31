#!/usr/bin/env python3
"""Audit ClickBell localization and IK at saved center/boundary RGB-D samples."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.click_bell.dynamic import build_dynamic_press_plan  # noqa: E402
from tasks.click_bell.perception import localize_saved_frame  # noqa: E402


def _sample(value: str) -> tuple[str, Path]:
    label, separator, raw_path = value.partition("=")
    if not separator or not label.strip() or not raw_path.strip():
        raise argparse.ArgumentTypeError("sample must be LABEL=/path/to/frame")
    return label.strip(), Path(raw_path).expanduser().resolve()


def _load(path: Path) -> dict:
    value = json.loads(path.expanduser().read_text())
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def audit_sample(
    label,
    directory,
    *,
    camera,
    arm,
    template,
    backend_options,
    workspace_check_enabled,
):
    try:
        metadata = _load(directory / "meta.json")
        localization = localize_saved_frame(directory, camera)
        joints = metadata["state_after"]["arms"][arm]["joints_rad"]
        plan = build_dynamic_press_plan(
            template,
            localization,
            joints,
            arm=arm,
            backend_options=backend_options,
            workspace_check_enabled=workspace_check_enabled,
        )
        return {
            "label": label,
            "frame_directory": str(directory),
            "status": "passed" if plan.get("execution_admissible") else "failed",
            "frame_id": localization.get("frame_id"),
            "camera": localization.get("camera"),
            "surface_position_m": localization.get("surface", {}).get("position_m"),
            "surface_quality": {
                key: localization.get("surface", {}).get(key)
                for key in (
                    "valid_depth_fraction",
                    "plane_rmse_mm",
                    "depth_fit_spread_95_mm",
                )
            },
            "execution_admissible": plan.get("execution_admissible"),
            "workspace_policy": plan.get("workspace_policy"),
            "stages_planned": plan.get("stages_planned"),
            "stage_summaries": [
                {
                    "approach_distance_m": stage.get("approach_distance_m"),
                    "tcp_target_m": stage.get("tcp_target_m"),
                    "status": stage.get("plan", {}).get("status"),
                    "checks": stage.get("plan", {}).get("checks"),
                    "failures": stage.get("plan", {}).get("failures", []),
                }
                for stage in plan.get("stages", [])
            ],
            "contact_tcp_target_m": (
                plan.get("stages", [{}])[-1].get("tcp_target_m")
                if plan.get("stages")
                else None
            ),
            "warnings": plan.get("warnings", []),
        }
    except Exception as exc:
        return {
            "label": label,
            "frame_directory": str(directory),
            "status": "failed",
            "execution_admissible": False,
            "error": {"type": type(exc).__name__, "message": str(exc)},
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sample",
        action="append",
        type=_sample,
        required=True,
        metavar="LABEL=FRAME_DIR",
        help="repeat for center and each boundary sample",
    )
    parser.add_argument("--camera", default="cam_right_wrist")
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--backend-options", type=Path, required=True)
    parser.add_argument(
        "--disable-workspace-check",
        action="store_true",
        help="skip configured ClickBell Cartesian workspace bounds for this read-only audit",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    template = _load(args.template)
    backend_options = _load(args.backend_options)
    samples = [
        audit_sample(
            label,
            directory,
            camera=args.camera,
            arm=args.arm,
            template=template,
            backend_options=backend_options,
            workspace_check_enabled=not args.disable_workspace_check,
        )
        for label, directory in args.sample
    ]
    passed = sum(item["status"] == "passed" for item in samples)
    report = {
        "schema_version": 1,
        "kind": "click_bell_dynamic_region_readonly_audit",
        "status": "passed" if passed == len(samples) else "failed",
        "dry_run": True,
        "motion_authorized": False,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "arm": args.arm,
        "camera": args.camera,
        "workspace_check_enabled": not args.disable_workspace_check,
        "samples_passed": passed,
        "samples_total": len(samples),
        "samples": samples,
    }
    if args.output:
        output = args.output.expanduser().resolve()
        write_private_json(output, report, directory_mode=0o700)
        report["receipt"] = str(output)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
