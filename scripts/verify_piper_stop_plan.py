#!/usr/bin/env python3
"""Record the operator stop/hold/recovery protocol without commanding the robot.

Run once without --confirmation to produce the required fields. After physically
checking the emergency-stop path on the exact supported robot, fill a private copy of
examples/piper_stop_confirmation.json and rerun with --confirmation.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.json_io import write_private_json  # noqa: E402
from services.piper.stop_check import build_stop_plan_receipt  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", type=Path, help="Completed operator confirmation JSON")
    parser.add_argument("--output", type=Path, help="Receipt path")
    args = parser.parse_args()

    try:
        confirmation = json.loads(args.confirmation.read_text()) if args.confirmation else None
        receipt = build_stop_plan_receipt(confirmation)
    except Exception as exc:
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_stop_plan",
            "status": "failed",
            "created_at": datetime.now(timezone.utc).isoformat(),
            "can_frames_sent": False,
            "motion_commands_sent": False,
            "general_motion_authorized": False,
            "failure_reasons": [f"{type(exc).__name__}: {exc}"],
        }

    output = args.output
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = artifact_root() / "bringup" / "stop_plans" / f"piper_stop_plan_{stamp}.json"
    output = output.resolve()
    write_private_json(output, receipt)
    print(json.dumps(receipt, indent=2))
    print(f"Receipt: {output}", file=sys.stderr)
    return 1 if receipt["status"] == "failed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
