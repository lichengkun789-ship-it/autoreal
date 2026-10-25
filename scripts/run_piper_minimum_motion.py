#!/usr/bin/env python3
"""Plan or execute one operator-only Piper minimum-motion characterization."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.json_io import write_private_json  # noqa: E402
from services.piper.minimum_motion import (  # noqa: E402
    PiperMinimumMotionRunner,
    validate_probe_receipt,
    validate_software_hold_confirmation,
)
from services.piper.sdk_reader import PiperSdkReader  # noqa: E402
from services.piper.probe import inspect_can_interface  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def _git_state() -> dict:
    project = Path(__file__).resolve().parents[1]
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=project,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=project,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        return {"git_commit": commit, "git_dirty": dirty}
    except (OSError, subprocess.SubprocessError) as exc:
        return {"unavailable": f"{type(exc).__name__}: {exc}"}


def _tx_delta(before: dict, after: dict) -> int | None:
    first = before.get("statistics", {}).get("tx_packets")
    last = after.get("statistics", {}).get("tx_packets")
    return last - first if isinstance(first, int) and isinstance(last, int) else None


def _interrupt_as_keyboard_interrupt(_signum, _frame):
    raise KeyboardInterrupt()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend-options",
        type=Path,
        default=os.environ.get("RAX_BACKEND_OPTIONS"),
    )
    parser.add_argument("--probe-receipt", type=Path, required=True)
    parser.add_argument("--confirmation", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--joint", type=int, required=True)
    parser.add_argument("--delta-rad", type=float, required=True)
    parser.add_argument("--speed-percent", type=int, default=3)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="send the bounded command; default is plan-only",
    )
    parser.add_argument(
        "--allow-enable",
        action="store_true",
        help="allow operator runner to enable the selected arm",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.backend_options is None:
        parser.error("pass --backend-options or set RAX_BACKEND_OPTIONS")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or (
        artifact_root()
        / "bringup"
        / "minimum_motion"
        / f"piper_minimum_motion_{stamp}.json"
    )
    reader = None
    can_before = None
    options = None
    previous_handlers = {}
    try:
        options = json.loads(args.backend_options.read_text())
        confirmation = validate_software_hold_confirmation(
            json.loads(args.confirmation.read_text())
        )
        probe = validate_probe_receipt(args.probe_receipt, options)
        phrase = f"MOVE {args.arm} J{args.joint} {args.delta_rad:+.6f}"
        if args.execute:
            print("WARNING: software hold only; no physical emergency stop is available.")
            print(
                "Ctrl+C requests the same last target, but process, CAN, or power "
                "failure can defeat it."
            )
            if input(f"Type exactly '{phrase}' to continue: ").strip() != phrase:
                raise PermissionError("interactive operator confirmation did not match")
        can_before = {
            side: inspect_can_interface(spec["can_name"])
            for side, spec in options["arms"].items()
        }
        for name in ("SIGTERM", "SIGHUP"):
            if hasattr(signal, name):
                signum = getattr(signal, name)
                previous_handlers[signum] = signal.signal(
                    signum, _interrupt_as_keyboard_interrupt
                )
        reader = PiperSdkReader(options["arms"])
        receipt = PiperMinimumMotionRunner(reader).run(
            arm=args.arm,
            joint=args.joint,
            delta_rad=args.delta_rad,
            speed_percent=args.speed_percent,
            operator_lease_id=uuid.uuid4().hex,
            execute=args.execute,
            allow_enable=args.allow_enable,
        )
        receipt["operator_confirmation"] = confirmation
        receipt["probe_receipt"] = probe
        receipt["backend_options_path"] = str(args.backend_options.resolve())
        receipt["software"] = _git_state()
    except BaseException as exc:
        receipt = {
            "schema_version": 1,
            "kind": "piper_minimum_motion_characterization",
            "status": "failed",
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "motion_command_attempted": False,
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

    if can_before is not None and options is not None:
        can_after = {
            side: inspect_can_interface(spec["can_name"])
            for side, spec in options["arms"].items()
        }
        receipt["can_interfaces"] = {
            side: {
                "name": options["arms"][side]["can_name"],
                "before": can_before[side],
                "after": can_after[side],
                "tx_packet_delta": _tx_delta(can_before[side], can_after[side]),
            }
            for side in ("left", "right")
        }
        if receipt.get("status") in {"planned", "passed"}:
            expected = {"left": 0, "right": 0}
            if receipt.get("motion_command_attempted"):
                # MotionCtrl_2 sends one frame and JointCtrl sends three; the
                # bounded command and last-target hold each use that four-frame pair.
                expected[args.arm] = 8 + int(receipt.get("enable_command_sent") is True)
            mismatches = [
                f"{side} tx_packet_delta={receipt['can_interfaces'][side]['tx_packet_delta']} "
                f"expected={expected[side]}"
                for side in ("left", "right")
                if receipt["can_interfaces"][side]["tx_packet_delta"] != expected[side]
            ]
            if mismatches:
                receipt["status"] = "failed"
                receipt.setdefault("failure_reasons", []).append(
                    "unexpected CAN transmission count: " + "; ".join(mismatches)
                )

    output = output.resolve()
    write_private_json(output, receipt)
    print(json.dumps(receipt, indent=2))
    print(f"Receipt: {output}", file=sys.stderr)
    return 0 if receipt["status"] in {"planned", "passed"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
