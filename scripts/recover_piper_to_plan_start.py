#!/usr/bin/env python3
"""Return one Piper arm to an admitted plan start under operator supervision."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import uuid


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.contact_motion import ContactStopSettings, PiperGuardedPressExecutor  # noqa: E402
from services.piper.minimum_motion import validate_probe_receipt, validate_software_hold_confirmation  # noqa: E402
from services.piper.sdk_reader import PiperSdkReader  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend-options", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--probe-receipt", type=Path, required=True)
    parser.add_argument("--confirmation", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    reader = None
    try:
        options = json.loads(args.backend_options.expanduser().read_text())
        plan = json.loads(args.plan.expanduser().read_text())
        probe = validate_probe_receipt(args.probe_receipt.expanduser(), options)
        confirmation = validate_software_hold_confirmation(
            json.loads(args.confirmation.expanduser().read_text())
        )
        phrase = f"RETURN {args.arm} PLAN_START"
        if args.execute and input(f"Type exactly '{phrase}' to continue: ").strip() != phrase:
            raise PermissionError("interactive operator confirmation did not match")
        reader = PiperSdkReader(options["arms"])
        settings = ContactStopSettings(
            effort_delta_nm=1.0,
            speed_percent=2,
            waypoint_timeout_s=5.0,
            settle_tolerance_rad=0.002,
            feedback_max_age_s=float(options.get("feedback_max_age_s", 0.25)),
            joint_margin_rad=float(options.get("joint_limit_margin_rad", 0.02)),
            max_joint_step_rad=float(options.get("max_joint_step_rad", 0.05)),
        )
        receipt = PiperGuardedPressExecutor(reader, settings).return_to_plan_start(
            plan,
            arm=args.arm,
            operator_lease_id=uuid.uuid4().hex,
            execute=args.execute,
            authorization_guard=lambda: None,
        )
        receipt.update(
            {
                "backend_options_path": str(args.backend_options.expanduser().resolve()),
                "press_plan_path": str(args.plan.expanduser().resolve()),
                "probe_receipt": probe,
                "operator_confirmation": confirmation,
                "motion_settings": settings.__dict__,
            }
        )
    except BaseException as exc:
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_plan_start_recovery",
            "status": "failed",
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "motion_command_attempted": False,
            "failure_reasons": [f"{type(exc).__name__}: {exc}"],
        }
    finally:
        if reader is not None:
            reader.close()
    output = args.output.expanduser().resolve()
    write_private_json(output, receipt)
    print(json.dumps(receipt, indent=2))
    print(f"Receipt: {output}", file=sys.stderr)
    return 0 if receipt["status"] in {"planned", "passed"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
