#!/usr/bin/env python3
"""Legacy specialized ClickBell runner retained for artifact compatibility.

New sessions use ``run_task_experiment.py`` through ``run_click_bell_experiment.sh``.
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

from services import arming  # noqa: E402
from services.controller.config import Configuration  # noqa: E402
from services.json_io import write_private_json  # noqa: E402
from harness.operator import launcher_command, model_preflight  # noqa: E402
from services.piper.click_bell.gripper import ClickBellGripperSettings  # noqa: E402
from services.piper.click_bell.press import ClickBellPressSettings  # noqa: E402
from services.piper.episode_recording import EpisodeRecordingSettings  # noqa: E402
from services.task_manifest import evaluate_task  # noqa: E402


TASK_FILE = PROJECT / "tasks" / "click_bell" / "TASK.md"
PLAN_TOOL = "click_bell_plan_press"
RECOVERY_TOOL = "click_bell_recover_observation_pose"
PRESS_TOOL = "click_bell_guarded_press"
GRIPPER_TOOL = "click_bell_set_gripper"
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
    settings = ClickBellPressSettings.from_options(
        config.task_options.get("guarded_press"),
        backend_options=config.backend_options,
    )
    gripper = ClickBellGripperSettings.from_options(
        config.task_options.get("gripper"), backend_options=config.backend_options
    )
    recording = EpisodeRecordingSettings.from_options(
        config.task_options.get("recording"), required=True
    )
    if settings is None:
        raise ValueError("ClickBell guarded press is not enabled")
    if gripper is None:
        raise ValueError("ClickBell Agent gripper is not enabled")
    if (config.root / "agent-lifecycle.json").exists():
        raise ValueError("guarded press commissioning requires a fresh session")
    return config, settings, gripper, recording


def _evidence(root: Path):
    requests = []
    audit = root / "mcp.jsonl"
    if audit.is_file():
        for line in audit.read_text().splitlines():
            event = json.loads(line)
            if event.get("event") == "mcp_request" and event.get("tool") in {
                RECOVERY_TOOL,
                PLAN_TOOL,
                GRIPPER_TOOL,
                PRESS_TOOL,
                CONFIRM_TOOL,
                "request_operator",
            }:
                requests.append(event)
    receipt_paths = sorted((root / "piper-motion").glob("agent_click_bell_press_*.json"))
    recovery_paths = sorted(
        (root / "piper-motion").glob("agent_click_bell_recovery_*.json")
    )
    gripper_paths = sorted(
        (root / "piper-motion").glob("agent_click_bell_gripper_*.json")
    )
    result_paths = sorted((root / "click-bell-results").glob("*.json"))
    recording_paths = sorted((root / "recordings").glob("*/manifest.json"))
    plan_paths = sorted((root / "click-bell-plans").glob("*.json"))
    return (
        requests,
        [json.loads(path.read_text()) for path in plan_paths],
        [json.loads(path.read_text()) for path in recovery_paths],
        [json.loads(path.read_text()) for path in receipt_paths],
        [json.loads(path.read_text()) for path in gripper_paths],
        [json.loads(path.read_text()) for path in result_paths],
        [str(path) for path in receipt_paths],
        [str(path) for path in gripper_paths],
        [str(path) for path in result_paths],
        [json.loads(path.read_text()) for path in recording_paths],
        [str(path) for path in recording_paths],
        [str(path) for path in plan_paths],
        [str(path) for path in recovery_paths],
    )


def _evaluate(
    returncode,
    requests,
    plans,
    receipts,
    gripper_receipts,
    results,
    recordings,
    final_arming,
    recovery_receipts=(),
) -> dict:
    reasons = []
    warnings = []
    if returncode != 0:
        reasons.append(f"Agent launcher exited with {returncode}")
    calls = [item.get("tool") for item in requests]
    recovery_calls = [index for index, name in enumerate(calls) if name == RECOVERY_TOOL]
    if len(recovery_receipts) != len(recovery_calls):
        reasons.append(
            "recovery calls and receipts differ "
            f"({len(recovery_calls)} calls, {len(recovery_receipts)} receipts)"
        )
    elif any(
        item.get("kind") != "piper_agent_click_bell_recovery"
        or item.get("status") != "passed"
        for item in recovery_receipts
    ):
        reasons.append("ClickBell recovery receipt did not pass")
    if calls.count(PRESS_TOOL) != 1:
        reasons.append(f"expected one {PRESS_TOOL} request, found {calls.count(PRESS_TOOL)}")
    admitted_plans = [
        plan
        for plan in plans
        if plan.get("dynamic") is True
        and plan.get("execution_admissible") is True
        and plan.get("used") is True
    ]
    if not admitted_plans:
        reasons.append("no admitted dynamic plan was consumed")
    if len(plans) > 1:
        warnings.append(f"Agent produced {len(plans)} plans before completing the task")
    if len(receipts) != 1:
        reasons.append(f"expected one motion receipt, found {len(receipts)}")
    elif (
        receipts[0].get("kind") != "piper_agent_click_bell_guarded_press"
        or receipts[0].get("status") not in {"passed", "contact_unverified"}
        or receipts[0].get("retreat", {}).get("status") != "passed"
    ):
        reasons.append("motion receipt did not confirm press and retreat")
    elif admitted_plans and receipts[0].get("admission", {}).get("plan_id") not in {
        plan.get("plan_id") for plan in admitted_plans
    }:
        reasons.append("motion receipt is not bound to the consumed dynamic plan")
    if len(gripper_receipts) != 2:
        reasons.append(
            f"expected two gripper receipts, found {len(gripper_receipts)}"
        )
    else:
        by_action = {receipt.get("action"): receipt for receipt in gripper_receipts}
        if set(by_action) != {"close", "open"}:
            reasons.append("gripper receipts do not contain one close and one open")
        elif any(
            receipt.get("kind") != "piper_agent_click_bell_gripper"
            or receipt.get("status") != "passed"
            for receipt in by_action.values()
        ):
            reasons.append("gripper receipts did not confirm motion")
        elif receipts and by_action["open"].get("press_id") != receipts[0].get("press_id"):
            reasons.append("gripper open receipt is not bound to the guarded press")
    task_evaluation = None
    if len(results) != 1:
        reasons.append(f"expected one visual result, found {len(results)}")
    else:
        recorded = results[0].get("evaluation", {})
        status = recorded.get("status")
        visual_success = {"passed": True, "failed": False, "unverified": None}.get(status)
        if status not in {"passed", "failed", "unverified"}:
            reasons.append("visual result has no valid task evaluation")
        else:
            task_evaluation = evaluate_task(
                "click_bell",
                {
                    "visual_success": visual_success,
                    "visual_reason": recorded.get("reason"),
                },
            )
    if len(recordings) != 1:
        reasons.append(f"expected one complete task recording, found {len(recordings)}")
    elif (
        recordings[0].get("kind") != "piper_task_episode_recording"
        or recordings[0].get("scope") != "task"
        or recordings[0].get("status") != "passed"
    ):
        reasons.append("complete task recording did not pass")
    if final_arming.get("armed") is not False:
        reasons.append("session was not disarmed after the trial")
    return {
        "status": "passed" if not reasons else "failed",
        "reasons": reasons,
        "warnings": warnings,
        "task_evaluation": task_evaluation,
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
        arming.write(config.root, False, reason="ClickBell Agent guarded press finished")

    final_arming = arming.read(config.root)
    (
        requests,
        plans,
        recovery_receipts,
        receipts,
        gripper_receipts,
        results,
        receipt_paths,
        gripper_paths,
        result_paths,
        recordings,
        recording_paths,
        plan_paths,
        recovery_paths,
    ) = _evidence(config.root)
    evaluation = _evaluate(
        returncode,
        requests,
        plans,
        receipts,
        gripper_receipts,
        results,
        recordings,
        final_arming,
        recovery_receipts,
    )
    report = {
        "schema_version": 1,
        "kind": "piper_agent_click_bell_experiment",
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
        "dynamic_plans": plan_paths,
        "recovery_receipts": recovery_paths,
        "motion_receipts": receipt_paths,
        "gripper_receipts": gripper_paths,
        "visual_results": result_paths,
        "task_recording": recording_paths[0] if len(recording_paths) == 1 else None,
        "evaluation": evaluation,
    }
    report_path = config.root / "click-bell-experiment.json"
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
        config, settings, gripper, recording = validate_configuration(config_path)
        preflight = model_preflight(config, args.key_file)
    except Exception as exc:
        print(f"PRECHECK FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    plan = {
        "status": "ready_to_execute" if args.execute else "planned",
        "session_root": str(config.root),
        "agent_cli": preflight["agent_cli"],
        "provider": preflight["provider"],
        "model": config.agent_model or "deepseek-flash",
        "tools": [
            RECOVERY_TOOL,
            PLAN_TOOL,
            GRIPPER_TOOL,
            PRESS_TOOL,
            "request_operator",
            CONFIRM_TOOL,
        ],
        "arm": settings.arm,
        "gripper": {
            "arm": gripper.arm,
            "open_width_m": gripper.open_width_m,
            "closed_width_m": gripper.closed_width_m,
            "step_m": gripper.runner_settings.step_m,
            "effort": gripper.runner_settings.effort,
        },
        "speed_percent": settings.speed_percent,
        "max_contact_travel_m": 0.005,
        "recording_fps": recording.fps,
        "save_depth": recording.save_depth,
        "recording_scope": "complete_task",
        "motion_sequence": [
            "recover_interrupted_path_if_required",
            "fresh_rgbd_dynamic_plan",
            "gripper_close",
            "direct_guarded_press_and_retreat",
            "gripper_open",
        ],
    }
    print(json.dumps(plan, indent=2))
    if not args.execute:
        print("No Agent, robot service, or motion command was started.")
        return 0

    print("Starting the DeepSeek ClickBell task immediately.")
    print("Keep the workspace clear and answer the visual confirmation request after retreat.")
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
