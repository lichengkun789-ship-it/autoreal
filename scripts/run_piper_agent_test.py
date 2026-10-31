#!/usr/bin/env python3
"""Run one operator-present, model-decided Piper Agent commissioning move.

The session must use ``tasks/piper_agent_joint_step/TASK.md`` and an operator-owned
backend configuration that admits only right J1 positive, one 0.005 rad command. The
script validates the model runtime before arming, runs the normal RAX launcher, and
always disarms the session when the launcher exits.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from harness.operator import launcher_command, model_preflight  # noqa: E402
from services import arming  # noqa: E402
from services.controller.config import Configuration  # noqa: E402
from services.json_io import write_private_json  # noqa: E402
from services.piper.agent_motion import AgentMotionSettings  # noqa: E402
from tasks.piper_agent_joint_step.evaluator import evaluate  # noqa: E402


TASK_ID = "piper_agent_joint_step"
TASK_FILE = PROJECT / "tasks" / TASK_ID / "TASK.md"
EXPECTED_ALLOWED = frozenset({("right", 1, "positive")})
EXPECTED_DELTA_RAD = 0.005


def validate_configuration(path: Path) -> Configuration:
    config = Configuration.load(path)
    if config.robot != "piper" or config.task != TASK_ID:
        raise ValueError(f"commissioning config must use robot=piper and task={TASK_ID}")
    if config.task_description.strip() != TASK_FILE.read_text().strip():
        raise ValueError(f"task description must come from {TASK_FILE}")
    options = config.backend_options
    settings = AgentMotionSettings.from_options(
        options.get("agent_motion"), arms=options.get("arms") or {}
    )
    if not settings.enabled:
        raise ValueError("Piper Agent motion is not enabled")
    if settings.allowed != EXPECTED_ALLOWED:
        raise ValueError("commissioning config must allow only right J1 positive")
    if settings.max_delta_rad != EXPECTED_DELTA_RAD:
        raise ValueError("commissioning max_delta_rad must be 0.005")
    if settings.max_commands_per_arming != 1:
        raise ValueError("commissioning config must allow one command per arming lease")
    if settings.max_cumulative_delta_rad != EXPECTED_DELTA_RAD:
        raise ValueError("commissioning cumulative delta budget must be 0.005 rad")
    if (config.root / "agent-lifecycle.json").exists():
        raise ValueError("commissioning requires a fresh session that has never been launched")
    return config


def _motion_evidence(root: Path) -> tuple[list[dict], list[dict], list[str]]:
    requests = []
    audit = root / "mcp.jsonl"
    if audit.is_file():
        for line in audit.read_text().splitlines():
            event = json.loads(line)
            if event.get("event") == "mcp_request" and event.get("tool") == "piper_move_joint":
                requests.append(event)

    receipt_paths = sorted((root / "piper-motion").glob("agent_joint_step_*.json"))
    receipts = [json.loads(path.read_text()) for path in receipt_paths]
    return requests, receipts, [str(path) for path in receipt_paths]


def run_trial(
    config: Configuration,
    config_path: Path,
    *,
    key_file: Path | None,
    arming_minutes: float,
    run_launcher=subprocess.run,
) -> dict:
    started_at = datetime.now(timezone.utc).isoformat()
    returncode = None
    launcher_error = None
    interrupted = False
    arming.write(config.root, True, minutes=arming_minutes)
    try:
        completed = run_launcher(launcher_command(config, config_path, key_file), cwd=PROJECT)
        returncode = completed.returncode
    except KeyboardInterrupt:
        interrupted = True
        launcher_error = "KeyboardInterrupt"
    except Exception as exc:
        launcher_error = f"{type(exc).__name__}: {exc}"
    finally:
        arming.write(config.root, False, reason="Agent commissioning run finished")

    final_arming = arming.read(config.root)
    requests, receipts, receipt_paths = _motion_evidence(config.root)
    evidence = {
        "launcher_returncode": returncode,
        "motion_requests": requests,
        "motion_receipts": receipts,
        "final_arming": final_arming,
    }
    evaluation = evaluate(evidence)
    report = {
        "schema_version": 1,
        "kind": "piper_agent_commissioning",
        "status": evaluation["status"],
        "started_at": started_at,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "session_root": str(config.root),
        "operator_input": str(config_path),
        "launcher_returncode": returncode,
        "launcher_error": launcher_error,
        "interrupted": interrupted,
        "final_arming": final_arming,
        "motion_requests": [
            {"time": item.get("time"), "call_id": item.get("call_id"), "arguments": item.get("arguments")}
            for item in requests
        ],
        "motion_receipts": receipt_paths,
        "evaluation": evaluation,
    }
    report_path = config.root / "piper-agent-commissioning.json"
    write_private_json(report_path, report, directory_mode=0o700)
    report["report"] = str(report_path)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="fresh operator-input.json")
    parser.add_argument("--key-file", type=Path, help="private key file for the selected Agent harness")
    parser.add_argument("--arming-minutes", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true", help="arm and run the full RAX Agent")
    args = parser.parse_args()
    if not 0 < args.arming_minutes <= 10:
        parser.error("--arming-minutes must be in (0, 10]")

    try:
        config_path = args.config.resolve()
        config = validate_configuration(config_path)
        preflight = model_preflight(config, args.key_file)
    except Exception as exc:
        print(f"PRECHECK FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    plan = {
        "status": "ready_to_execute" if args.execute else "planned",
        "session_root": str(config.root),
        "task": config.task,
        "agent_cli": preflight["agent_cli"],
        "provider": preflight["provider"],
        "allowed_move": {"arm": "right", "joint": 1, "delta_rad": EXPECTED_DELTA_RAD},
        "max_commands": 1,
        "arming_minutes": args.arming_minutes,
    }
    print(json.dumps(plan, indent=2))
    if not args.execute:
        print("No Agent, robot service, or motion command was started.")
        return 0

    phrase = "RUN RAX AGENT"
    print("WARNING: the model may now choose the one admitted physical command.")
    print("The operator must remain at the keyboard; software hold is not a physical e-stop.")
    if input(f"Type exactly '{phrase}' to continue: ").strip() != phrase:
        print("not armed")
        return 1

    report = run_trial(
        config,
        config_path,
        key_file=args.key_file,
        arming_minutes=args.arming_minutes,
    )
    print(json.dumps(report, indent=2))
    print(f"Report: {report['report']}", file=sys.stderr)
    if report["interrupted"]:
        return 130
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
