#!/usr/bin/env python3
"""Probe dual Piper feedback without sending any motion command.

    python scripts/probe_piper_readonly.py --backend-options /path/to/piper.json

The command exits 0 only when every sample has fresh, error-free feedback from both
arms. It never configures CAN, enables motors, clears faults, or commands motion.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.piper.probe import run_readonly_probe  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--backend-options",
        type=Path,
        default=os.environ.get("RAX_BACKEND_OPTIONS") or None,
        help="Piper backend JSON (default: $RAX_BACKEND_OPTIONS)",
    )
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--startup-seconds", type=float, default=1.0)
    parser.add_argument("--interval-seconds", type=float, default=0.2)
    parser.add_argument("--output", type=Path, help="Receipt path (default: artifact root/probes/...json)")
    args = parser.parse_args()
    if args.backend_options is None:
        parser.error("pass --backend-options or set RAX_BACKEND_OPTIONS")
    try:
        options = json.loads(args.backend_options.read_text())
        receipt = run_readonly_probe(
            options,
            samples=args.samples,
            startup_seconds=args.startup_seconds,
            interval_seconds=args.interval_seconds,
        )
    except Exception as exc:
        receipt = {
            "schema_version": 1,
            "kind": "piper_readonly_probe",
            "status": "failed",
            "can_queries_sent": False,
            "motion_commands_sent": False,
            "failure_reasons": [f"{type(exc).__name__}: {exc}"],
            "finished_at": datetime.now(timezone.utc).isoformat(),
        }

    output = args.output
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = artifact_root() / "probes" / f"piper_readonly_{stamp}.json"
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(receipt, indent=2) + "\n")
    output.chmod(0o600)
    print(json.dumps(receipt, indent=2))
    print(f"Receipt: {output}", file=sys.stderr)
    return 0 if receipt["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
