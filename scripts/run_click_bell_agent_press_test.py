#!/usr/bin/env python3
"""Run one operator-present DeepSeek ClickBell guarded-press trial."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services import arming  # noqa: E402
from services.controller.config import Configuration  # noqa: E402
from services.json_io import write_private_json  # noqa: E402
from services.piper.click_bell_press import ClickBellPressSettings  # noqa: E402
from services.piper.episode_recording import EpisodeRecordingSettings  # noqa: E402
from scripts.run_piper_agent_test import _launcher_command, model_preflight  # noqa: E402


TASK_FILE = PROJECT / "tasks" / "click_bell" / "AGENT_GUARDED_PRESS.md"
PRESS_TOOL = "click_bell_guarded_press"
CONFIRM_TOOL = "click_bell_confirm_visual_result"


def validate_configuration(path: Path):
    config = Configuration.load(path)
    if config.robot != "piper" or config.task != "click_bell":
        raise ValueError("guarded press config must use robot=piper and task=click_bell")
    if config.task_description.strip() != TASK_FILE.read_text().strip():
        raise ValueError(f"task description must come from {TASK_FILE}")
    base_motion = config.backend_options.get("agent_motion") or {}
    if isinstance(base_motion, dict) and base_motion.get("enabled") is True:
        raise ValueError("generic piper_move_joint must remain disabled for this trial")
    preview = config.task_options.get("motion") or {}
    if isinstance(preview, dict) and preview.get("enabled") is True:
        raise ValueError("approach preview must remain disabled for the guarded press trial")
    settings = ClickBellPressSettings.from_options(
        config.task_options.get("guarded_press"),
        backend_options=config.backend_options,
    )
    recording = EpisodeRecordingSettings.from_options(
        config.task_options.get("recording"), required=True
    )
    if settings is None:
        raise ValueError("ClickBell guarded press is not enabled")
    if (config.root / "agent-lifecycle.json").exists():
        raise ValueError("guarded press commissioning requires a fresh session")
    return config, settings, recording


def _evidence(root: Path):
    requests = []
    audit = root / "mcp.jsonl"
    if audit.is_file():
        for line in audit.read_text().splitlines():
            event = json.loads(line)
            if event.get("event") == "mcp_request" and event.get("tool") in {
                PRESS_TOOL,
                CONFIRM_TOOL,
                "request_operator",
            }:
                requests.append(event)
    receipt_paths = sorted((root / "piper-motion").glob("agent_click_bell_press_*.json"))
    result_paths = sorted((root / "click-bell-results").glob("*.json"))
    return (
        requests,
        [json.loads(path.read_text()) for path in receipt_paths],
        [json.loads(path.read_text()) for path in result_paths],
        [str(path) for path in receipt_paths],
        [str(path) for path in result_paths],
    )


def _evaluate(returncode, requests, receipts, results, final_arming) -> dict:
    reasons = []
    if returncode != 0:
        reasons.append(f"Agent launcher exited with {returncode}")
    calls = [item.get("tool") for item in requests]
    if calls.count(PRESS_TOOL) != 1:
        reasons.append(f"expected one {PRESS_TOOL} request, found {calls.count(PRESS_TOOL)}")
    if calls.count("request_operator") != 1:
        reasons.append("expected one operator visual confirmation request")
    if calls.count(CONFIRM_TOOL) != 1:
        reasons.append(f"expected one {CONFIRM_TOOL} request")
    if len(receipts) != 1:
        reasons.append(f"expected one motion receipt, found {len(receipts)}")
    elif (
        receipts[0].get("kind") != "piper_agent_click_bell_guarded_press"
        or receipts[0].get("status") not in {"passed", "contact_unverified"}
        or receipts[0].get("retreat", {}).get("status") != "passed"
        or receipts[0].get("recording", {}).get("status") != "passed"
    ):
        reasons.append("motion receipt did not confirm press, retreat, and recording")
    if len(results) != 1:
        reasons.append(f"expected one visual result, found {len(results)}")
    elif results[0].get("evaluation", {}).get("status") not in {
        "passed",
        "failed",
        "unverified",
    }:
        reasons.append("visual result has no valid task evaluation")
    if final_arming.get("armed") is not False:
        reasons.append("session was not disarmed after the trial")
    return {"status": "passed" if not reasons else "failed", "reasons": reasons}


def run_trial(config, config_path, *, key_file, arming_minutes, run_launcher=subprocess.run):
    started_at = datetime.now(timezone.utc).isoformat()
    returncode = None
    launcher_error = None
    interrupted = False
    arming.write(config.root, True, minutes=arming_minutes)
    try:
        completed = run_launcher(
            _launcher_command(config, config_path, key_file), cwd=PROJECT
        )
        returncode = completed.returncode
    except KeyboardInterrupt:
        interrupted = True
        launcher_error = "KeyboardInterrupt"
    except Exception as exc:
        launcher_error = f"{type(exc).__name__}: {exc}"
    finally:
        arming.write(config.root, False, reason="ClickBell Agent guarded press finished")

    final_arming = arming.read(config.root)
    requests, receipts, results, receipt_paths, result_paths = _evidence(config.root)
    evaluation = _evaluate(returncode, requests, receipts, results, final_arming)
    report = {
        "schema_version": 1,
        "kind": "piper_agent_click_bell_press_commissioning",
        "status": evaluation["status"],
        "started_at": started_at,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "session_root": str(config.root),
        "operator_input": str(config_path),
        "launcher_returncode": returncode,
        "launcher_error": launcher_error,
        "interrupted": interrupted,
        "final_arming": final_arming,
        "tool_requests": [
            {
                "time": item.get("time"),
                "call_id": item.get("call_id"),
                "tool": item.get("tool"),
                "arguments": item.get("arguments"),
            }
            for item in requests
        ],
        "motion_receipts": receipt_paths,
        "visual_results": result_paths,
        "evaluation": evaluation,
    }
    report_path = config.root / "piper-agent-click-bell-press.json"
    write_private_json(report_path, report, directory_mode=0o700)
    report["report"] = str(report_path)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--key-file", type=Path)
    parser.add_argument("--arming-minutes", type=float, default=10.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not 0 < args.arming_minutes <= 15:
        parser.error("--arming-minutes must be in (0, 15]")
    try:
        config_path = args.config.resolve()
        config, settings, recording = validate_configuration(config_path)
        preflight = model_preflight(config, args.key_file)
    except Exception as exc:
        print(f"PRECHECK FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    plan = {
        "status": "ready_to_execute" if args.execute else "planned",
        "session_root": str(config.root),
        "agent_cli": preflight["agent_cli"],
        "provider": preflight["provider"],
        "tools": [PRESS_TOOL, "request_operator", CONFIRM_TOOL],
        "arm": settings.arm,
        "speed_percent": settings.speed_percent,
        "max_contact_travel_m": 0.005,
        "recording_fps": recording.fps,
        "save_depth": recording.save_depth,
        "calls_per_arming": 1,
    }
    print(json.dumps(plan, indent=2))
    if not args.execute:
        print("No Agent, robot service, or motion command was started.")
        return 0

    phrase = "RUN CLICK BELL GUARDED PRESS"
    print("WARNING: DeepSeek may call the commissioned physical press path exactly once.")
    print("Keep the workspace clear and answer the visual confirmation request after retreat.")
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
