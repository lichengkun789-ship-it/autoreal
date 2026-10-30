#!/usr/bin/env python3
"""Plan or run one operator-only Piper gripper close-and-restore cycle."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.gripper_characterization import (  # noqa: E402
    GripperCycleSettings,
    PiperGripperCycleRunner,
)
from services.piper.minimum_motion import (  # noqa: E402
    validate_probe_receipt,
    validate_software_hold_confirmation,
)
from services.piper.sdk_reader import PiperSdkReader  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def _interrupt(_signum, _frame):
    raise KeyboardInterrupt()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend-options", type=Path, default=os.environ.get("RAX_BACKEND_OPTIONS")
    )
    parser.add_argument("--probe-receipt", type=Path, required=True)
    parser.add_argument("--confirmation", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--step-m", type=float, default=0.01)
    parser.add_argument("--effort", type=int, default=500)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.backend_options is None:
        parser.error("pass --backend-options or set RAX_BACKEND_OPTIONS")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or (
        artifact_root() / "bringup" / "gripper" / f"piper_gripper_cycle_{stamp}.json"
    )
    reader = None
    previous_handlers = {}
    try:
        options = json.loads(args.backend_options.expanduser().read_text())
        probe = validate_probe_receipt(args.probe_receipt.expanduser(), options)
        confirmation = validate_software_hold_confirmation(
            json.loads(args.confirmation.expanduser().read_text())
        )
        if args.execute:
            phrase = f"CYCLE {args.arm} GRIPPER"
            print("WARNING: the selected gripper will close fully and reopen.")
            print("Keep fingers, cables, the AprilTag, and objects outside the jaws.")
            if input(f"Type exactly '{phrase}' to continue: ").strip() != phrase:
                raise PermissionError("interactive operator confirmation did not match")
        for name in ("SIGTERM", "SIGHUP"):
            if hasattr(signal, name):
                signum = getattr(signal, name)
                previous_handlers[signum] = signal.signal(signum, _interrupt)
        reader = PiperSdkReader(options["arms"])
        settings = GripperCycleSettings(step_m=args.step_m, effort=args.effort)
        receipt = PiperGripperCycleRunner(reader, settings).run(
            arm=args.arm, execute=args.execute
        )
        receipt.update(
            {
                "backend_options_path": str(args.backend_options.expanduser().resolve()),
                "probe_receipt": probe,
                "operator_confirmation": confirmation,
            }
        )
    except BaseException as exc:
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_gripper_cycle",
            "status": "failed",
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "arm_motion_commanded": False,
            "gripper_command_attempted": False,
            "failure_reasons": [f"{type(exc).__name__}: {exc}"],
        }
    finally:
        if reader is not None:
            try:
                reader.close()
            except BaseException as exc:
                receipt["status"] = "failed"
                receipt.setdefault("failure_reasons", []).append(
                    f"reader close failed: {type(exc).__name__}: {exc}"
                )
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)

    output = output.expanduser().resolve()
    write_private_json(output, receipt)
    print(json.dumps(receipt, indent=2))
    print(f"Receipt: {output}", file=sys.stderr)
    return 0 if receipt["status"] in {"planned", "passed"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
