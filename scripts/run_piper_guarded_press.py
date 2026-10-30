#!/usr/bin/env python3
"""Plan or execute one operator-only guarded Piper press and reverse retreat."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import uuid


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.contact_motion import (  # noqa: E402
    ContactStopSettings,
    PiperGuardedPressExecutor,
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
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--probe-receipt", type=Path, required=True)
    parser.add_argument("--confirmation", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument(
        "--effort-delta-nm",
        type=float,
        required=True,
        help="characterized motor-effort increase that terminates contact motion",
    )
    parser.add_argument("--speed-percent", type=int, default=3)
    parser.add_argument("--waypoint-timeout-s", type=float, default=5.0)
    parser.add_argument("--settle-tolerance-rad", type=float, default=0.002)
    parser.add_argument("--prior-stage-receipt", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.backend_options is None:
        parser.error("pass --backend-options or set RAX_BACKEND_OPTIONS")
    if not 1.0 <= args.waypoint_timeout_s <= 10.0:
        parser.error("--waypoint-timeout-s must be in 1.0..10.0")
    if not 0.0005 <= args.settle_tolerance_rad <= 0.003:
        parser.error("--settle-tolerance-rad must be in 0.0005..0.003")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or (
        artifact_root()
        / "bringup"
        / "guarded_press"
        / f"piper_guarded_press_{stamp}.json"
    )
    reader = None
    previous_handlers = {}
    try:
        options = json.loads(args.backend_options.expanduser().read_text())
        plan = json.loads(args.plan.expanduser().read_text())
        prior_path = None if args.prior_stage_receipt is None else args.prior_stage_receipt.expanduser().resolve()
        prior_stage = None if prior_path is None else json.loads(prior_path.read_text())
        probe = validate_probe_receipt(args.probe_receipt.expanduser(), options)
        confirmation = validate_software_hold_confirmation(
            json.loads(args.confirmation.expanduser().read_text())
        )
        phrase = f"PRESS {args.arm} {plan.get('frame_id', 'unknown')}"
        if args.execute:
            print("WARNING: software contact stop and hold are not safety-rated.")
            print(
                "Ctrl+C requests measured-position hold; CAN, process, or power "
                "failure can defeat it."
            )
            if input(f"Type exactly '{phrase}' to continue: ").strip() != phrase:
                raise PermissionError("interactive operator confirmation did not match")
        for name in ("SIGTERM", "SIGHUP"):
            if hasattr(signal, name):
                signum = getattr(signal, name)
                previous_handlers[signum] = signal.signal(signum, _interrupt)
        reader = PiperSdkReader(options["arms"])
        settings = ContactStopSettings(
            effort_delta_nm=args.effort_delta_nm,
            speed_percent=args.speed_percent,
            feedback_max_age_s=float(options.get("feedback_max_age_s", 0.25)),
            joint_margin_rad=float(options.get("joint_limit_margin_rad", 0.02)),
            max_joint_step_rad=float(options.get("max_joint_step_rad", 0.05)),
            waypoint_timeout_s=args.waypoint_timeout_s,
            settle_tolerance_rad=args.settle_tolerance_rad,
        )
        receipt = PiperGuardedPressExecutor(reader, settings).run(
            plan,
            arm=args.arm,
            operator_lease_id=uuid.uuid4().hex,
            execute=args.execute,
            authorization_guard=lambda: None,
            prior_stage_receipt=prior_stage,
        )
        receipt.update(
            {
                "backend_options_path": str(args.backend_options.expanduser().resolve()),
                "press_plan_path": str(args.plan.expanduser().resolve()),
                "probe_receipt": probe,
                "operator_confirmation": confirmation,
                "contact_settings": settings.__dict__,
                "prior_stage_evidence": None
                if prior_path is None
                else {
                    "path": str(prior_path),
                    "sha256": hashlib.sha256(prior_path.read_bytes()).hexdigest(),
                },
            }
        )
    except BaseException as exc:
        receipt = {
            "schema_version": 1,
            "kind": "piper_operator_guarded_press",
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

    output = output.expanduser().resolve()
    write_private_json(output, receipt)
    print(json.dumps(receipt, indent=2))
    print(f"Receipt: {output}", file=sys.stderr)
    return 0 if receipt["status"] in {"planned", "passed", "contact_unverified"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
