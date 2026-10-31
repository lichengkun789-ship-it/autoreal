#!/usr/bin/env python3
"""Run one operator-present RAX task and collect task-independent evidence."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from harness.operator import launcher_command, model_preflight  # noqa: E402
from services import arming  # noqa: E402
from services.controller.config import Configuration  # noqa: E402
from services.json_io import write_private_json  # noqa: E402
from services.task_manifest import evaluate_task, load_task_manifest  # noqa: E402


VISUAL_OUTCOMES = {
    "success": True,
    "成功": True,
    "failure": False,
    "失败": False,
    "unclear": None,
    "不确定": None,
}


def validate_configuration(path: Path) -> Configuration:
    config = Configuration.load(path)
    manifest = load_task_manifest(config.task, required=True)
    if manifest.evaluator is None:
        raise ValueError(f"task {config.task!r} has no evaluator")
    if not config.task_description.strip():
        raise ValueError("task description is empty")
    if config.robot == "piper":
        cartesian = config.backend_options.get("cartesian_motion") or {}
        gripper = config.backend_options.get("gripper_motion") or {}
        recording = config.backend_options.get("episode_recording") or {}
        if cartesian.get("enabled") is not True:
            raise ValueError("Piper autonomous task requires cartesian_motion.enabled=true")
        if gripper.get("enabled") is not True:
            raise ValueError("Piper autonomous task requires gripper_motion.enabled=true")
        if recording.get("enabled") is not True:
            raise ValueError("Piper autonomous task requires episode_recording.enabled=true")
        if config.observation_profile != "rgbd":
            raise ValueError("Piper autonomous task requires observation_profile=rgbd")
    if (config.root / "agent-lifecycle.json").exists():
        raise ValueError("an experiment requires a fresh session")
    return config


def _json_files(root: Path, pattern: str) -> tuple[list[dict], list[str]]:
    paths = sorted(root.glob(pattern))
    return [json.loads(path.read_text()) for path in paths], [str(path) for path in paths]


def _requests(root: Path) -> list[dict]:
    path = root / "mcp.jsonl"
    if not path.is_file():
        return []
    return [
        event
        for line in path.read_text().splitlines()
        if (event := json.loads(line)).get("event") == "mcp_request"
    ]


def _visual_result(root: Path) -> dict:
    directory = root / "operator" / "requests"
    answered = []
    if directory.is_dir():
        for request_path in sorted(directory.glob("*.json")):
            if request_path.name.endswith(".reply.json"):
                continue
            request = json.loads(request_path.read_text())
            reply_path = request_path.with_name(
                f"{request_path.stem}.reply.json"
            )
            if request.get("kind") != "visual_confirmation" or not reply_path.is_file():
                continue
            reply = json.loads(reply_path.read_text())
            first = re.split(
                r"[\s,:，：]", str(reply.get("reply", "")).strip(), maxsplit=1
            )[0].lower()
            answered.append(
                {
                    "request": request,
                    "reply": reply,
                    "visual_success": VISUAL_OUTCOMES.get(first),
                    "recognized": first in VISUAL_OUTCOMES,
                }
            )
    if not answered:
        return {
            "visual_success": None,
            "visual_reason": "No answered operator visual confirmation",
            "evidence": None,
        }
    latest = answered[-1]
    success = latest["visual_success"] if latest["recognized"] else None
    return {
        "visual_success": success,
        "visual_reason": f"Operator visual confirmation: {latest['reply'].get('reply', '')}",
        "evidence": latest,
    }


def collect_evidence(config: Configuration) -> dict:
    root = config.root
    plans, plan_paths = _json_files(root, "piper-cartesian-plans/*.json")
    motions, motion_paths = _json_files(root, "piper-motion/*.json")
    recordings, recording_paths = _json_files(root, "recordings/*/manifest.json")
    requests = _requests(root)
    visual = _visual_result(root)
    evaluation = evaluate_task(
        config.task,
        {
            "visual_success": visual["visual_success"],
            "visual_reason": visual["visual_reason"],
        },
    )
    return {
        "requests": requests,
        "plans": plans,
        "plan_paths": plan_paths,
        "motions": motions,
        "motion_paths": motion_paths,
        "recordings": recordings,
        "recording_paths": recording_paths,
        "visual": visual,
        "task_evaluation": evaluation,
    }


def evaluate_run(returncode, evidence, final_arming) -> dict:
    reasons = []
    warnings = []
    if returncode != 0:
        reasons.append(f"Agent launcher exited with {returncode}")
    if evidence["task_evaluation"].get("status") != "passed":
        reasons.append(
            "task evaluator did not pass: "
            f"{evidence['task_evaluation'].get('reason', 'unknown reason')}"
        )
    passed_recordings = [
        item
        for item in evidence["recordings"]
        if item.get("kind") == "piper_task_episode_recording"
        and item.get("scope") == "task"
        and item.get("status") == "passed"
    ]
    if len(passed_recordings) != 1:
        reasons.append(
            "expected one passed complete-session recording, found "
            f"{len(passed_recordings)}"
        )
    generic_tools = {
        event.get("tool") for event in evidence["requests"] if event.get("tool")
    }
    task_specific = sorted(name for name in generic_tools if name.startswith("click_bell_"))
    if task_specific:
        reasons.append(f"task-specific robot tools were used: {task_specific}")
    if final_arming.get("armed") is not False:
        reasons.append("session was not disarmed after the trial")
    if not evidence["plans"]:
        warnings.append("no Cartesian plan receipt was produced")
    if not evidence["motions"]:
        warnings.append("no generic Piper motion receipt was produced")
    return {
        "status": "passed" if not reasons else "failed",
        "reasons": reasons,
        "warnings": warnings,
        "task_evaluation": evidence["task_evaluation"],
    }


def run_trial(
    config,
    config_path,
    *,
    key_file,
    arming_minutes,
    run_launcher=subprocess.run,
):
    started_at = datetime.now(timezone.utc).isoformat()
    returncode = None
    launcher_error = None
    interrupted = False
    arming.write(config.root, True, minutes=arming_minutes)
    try:
        completed = run_launcher(
            launcher_command(config, config_path, key_file), cwd=PROJECT
        )
        returncode = completed.returncode
    except KeyboardInterrupt:
        interrupted = True
        launcher_error = "KeyboardInterrupt"
    except Exception as exc:
        launcher_error = f"{type(exc).__name__}: {exc}"
    finally:
        arming.write(config.root, False, reason="RAX task experiment finished")

    final_arming = arming.read(config.root)
    evidence = collect_evidence(config)
    evaluation = evaluate_run(returncode, evidence, final_arming)
    report = {
        "schema_version": 1,
        "kind": "rax_task_experiment",
        "task": config.task,
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
            for item in evidence["requests"]
        ],
        "plans": evidence["plan_paths"],
        "motion_receipts": evidence["motion_paths"],
        "recordings": evidence["recording_paths"],
        "visual_result": evidence["visual"],
        "evaluation": evaluation,
    }
    report_path = config.root / "task-experiment.json"
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
        config = validate_configuration(config_path)
        preflight = model_preflight(config, args.key_file)
    except Exception as exc:
        print(f"PRECHECK FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    plan = {
        "status": "ready_to_execute" if args.execute else "planned",
        "task": config.task,
        "session_root": str(config.root),
        "agent_cli": preflight["agent_cli"],
        "provider": preflight["provider"],
        "model": config.agent_model or "provider-default",
        "agent_mode": config.agent_mode,
        "robot_interface": "generic_piper_tools",
        "task_specific_robot_tools": False,
        "recording_scope": "complete_backend_session",
    }
    print(json.dumps(plan, indent=2))
    if not args.execute:
        print("No Agent, robot service, or motion command was started.")
        return 0

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
