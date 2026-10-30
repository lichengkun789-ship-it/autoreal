#!/usr/bin/env python3
"""Create, but do not activate, a path-scoped Piper workspace candidate."""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.json_io import write_private_json  # noqa: E402
from services.piper.workspace_commissioning import derive_workspace_candidate  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--backend-options", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--tcp-margin-m", nargs=3, type=float, default=(0.01, 0.01, 0.01))
    parser.add_argument("--link-margin-m", type=float, default=0.01)
    parser.add_argument("--candidate-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    try:
        plan_bytes = args.plan.expanduser().read_bytes()
        plan = json.loads(plan_bytes)
        options = json.loads(args.backend_options.expanduser().read_text())
        report = derive_workspace_candidate(
            plan,
            arm=args.arm,
            tcp_margin_m=args.tcp_margin_m,
            link_margin_m=args.link_margin_m,
        )
        candidate = deepcopy(options)
        existing = candidate.get("workspace_bounds") or {}
        existing.update(report["candidate"]["workspace_bounds"])
        candidate["workspace_bounds"] = existing
        candidate["link_min_z_m"] = report["candidate"]["link_min_z_m"]
        candidate.setdefault("agent_motion", {})["enabled"] = False
        write_private_json(args.candidate_config.expanduser().resolve(), candidate)
        report.update(
            {
                "source_plan": str(args.plan.expanduser().resolve()),
                "source_plan_sha256": hashlib.sha256(plan_bytes).hexdigest(),
                "source_backend_options": str(args.backend_options.expanduser().resolve()),
                "candidate_config": str(args.candidate_config.expanduser().resolve()),
            }
        )
    except Exception as exc:
        report = {
            "schema_version": 1,
            "kind": "piper_workspace_candidate",
            "status": "failed",
            "activation_allowed": False,
            "error": {"type": type(exc).__name__, "message": str(exc)},
        }
    write_private_json(args.output.expanduser().resolve(), report)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "candidate" else 1


if __name__ == "__main__":
    raise SystemExit(main())
