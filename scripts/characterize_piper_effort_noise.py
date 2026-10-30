#!/usr/bin/env python3
"""Collect read-only stationary Piper joint-effort noise evidence."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.effort_characterization import PiperEffortNoiseRunner  # noqa: E402
from services.piper.sdk_reader import PiperSdkReader  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend-options", type=Path, default=os.environ.get("RAX_BACKEND_OPTIONS")
    )
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--interval-s", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.backend_options is None:
        parser.error("pass --backend-options or set RAX_BACKEND_OPTIONS")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or (
        artifact_root() / "bringup" / "effort" / f"piper_effort_noise_{stamp}.json"
    )
    reader = None
    try:
        options = json.loads(args.backend_options.expanduser().read_text())
        reader = PiperSdkReader(options["arms"])
        receipt = PiperEffortNoiseRunner(reader).run(
            arm=args.arm,
            samples=args.samples,
            interval_s=args.interval_s,
            feedback_max_age_s=float(options.get("feedback_max_age_s", 0.25)),
        )
        receipt["backend_options_path"] = str(args.backend_options.expanduser().resolve())
    except BaseException as exc:
        receipt = {
            "schema_version": 1,
            "kind": "piper_stationary_effort_noise",
            "status": "failed",
            "motion_commands_sent": False,
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "failure_reasons": [f"{type(exc).__name__}: {exc}"],
        }
    finally:
        if reader is not None:
            reader.close()

    output = output.expanduser().resolve()
    write_private_json(output, receipt)
    print(json.dumps(receipt, indent=2))
    print(f"Receipt: {output}", file=sys.stderr)
    return 0 if receipt["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
