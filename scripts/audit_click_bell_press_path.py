#!/usr/bin/env python3
"""Create a read-only staged ClickBell press-path audit from a saved RGB-D frame."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.tooling import GRIPPER_CENTER_CONTACT_OFFSET_TCP_M  # noqa: E402
from tasks.click_bell.perception import localize_saved_frame  # noqa: E402
from tasks.click_bell.planning import plan_press_path  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame_directory", type=Path)
    parser.add_argument("--camera", default="cam_right_wrist")
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--rpy-deg", nargs=3, type=float, required=True)
    parser.add_argument(
        "--tool-contact-offset-tcp-m",
        nargs=3,
        type=float,
        default=GRIPPER_CENTER_CONTACT_OFFSET_TCP_M,
        help="defaults to the operator-accepted Piper URDF gripper-centre offset",
    )
    parser.add_argument("--backend-options", type=Path)
    parser.add_argument(
        "--unverified-tool-geometry",
        action="store_true",
        help="override the commissioned default when testing another tool",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    try:
        directory = args.frame_directory.expanduser().resolve()
        metadata = json.loads((directory / "meta.json").read_text())
        localization = localize_saved_frame(directory, args.camera)
        expected_frame = f"piper_{args.arm}_base"
        if localization["frame"] != expected_frame:
            raise ValueError(
                f"camera localization is in {localization['frame']}, not {expected_frame}"
            )
        options = (
            json.loads(args.backend_options.expanduser().read_text())
            if args.backend_options
            else {}
        )
        bounds = (options.get("workspace_bounds") or {}).get(args.arm)
        report = plan_press_path(
            metadata["state_after"]["arms"][args.arm]["joints_rad"],
            localization,
            tool_contact_offset_tcp_m=args.tool_contact_offset_tcp_m,
            rpy_rad=[math.radians(value) for value in args.rpy_deg],
            joint_margin_rad=float(options.get("joint_limit_margin_rad", 0.02)),
            max_joint_step_rad=float(options.get("max_joint_step_rad", 0.05)),
            workspace_min=None if bounds is None else bounds["min"],
            workspace_max=None if bounds is None else bounds["max"],
            link_min_z_m=options.get("link_min_z_m"),
            tool_geometry_verified=not args.unverified_tool_geometry,
        )
        report.update(
            {
                "arm": args.arm,
                "source_frame_directory": str(directory),
                "backend_options": (
                    None
                    if args.backend_options is None
                    else str(args.backend_options.expanduser().resolve())
                ),
            }
        )
    except Exception as exc:
        report = {
            "schema_version": 1,
            "kind": "click_bell_readonly_press_path",
            "status": "failed",
            "dry_run": True,
            "motion_authorized": False,
            "geometry_admissible": False,
            "execution_admissible": False,
            "error": {"type": type(exc).__name__, "message": str(exc)},
        }

    if args.output:
        write_private_json(args.output, report, directory_mode=0o700)
        report["receipt"] = str(args.output.expanduser().resolve())
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
