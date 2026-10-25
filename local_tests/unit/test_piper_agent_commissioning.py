"""The full-Agent commissioning wrapper is tested without a model or hardware."""

import json
from pathlib import Path
from types import SimpleNamespace

from services import arming
from scripts.run_piper_agent_test import _launcher_command, run_trial
from tasks.piper_agent_joint_step.evaluator import EXPECTED_ARGUMENTS, evaluate


def passed_receipt():
    return {
        "kind": "piper_agent_joint_step",
        "status": "passed",
        "motion_command_attempted": True,
        "requested": {**EXPECTED_ARGUMENTS, "speed_percent": 3, "execute": True},
        "hold": {"state": "hold_latched", "hold_sdk_calls_returned": True},
    }


def test_commissioning_evaluator_requires_one_exact_request_and_passed_hold():
    evidence = {
        "launcher_returncode": 0,
        "motion_requests": [{"arguments": EXPECTED_ARGUMENTS}],
        "motion_receipts": [passed_receipt()],
        "final_arming": {"armed": False},
    }
    assert evaluate(evidence)["success"] is True

    evidence["motion_requests"].append({"arguments": EXPECTED_ARGUMENTS})
    result = evaluate(evidence)
    assert result["success"] is False
    assert "exactly one" in "; ".join(result["reasons"])


def test_trial_arms_only_around_launcher_and_disarms_after_success(tmp_path):
    root = tmp_path / "session"
    root.mkdir()
    config_path = root / "operator-input.json"
    config_path.write_text("{}")
    config = SimpleNamespace(root=root, agent_cli="dsh")

    def fake_launcher(command, cwd):
        assert arming.read(root)["armed"] is True
        assert command[-3:] == ["--non-interactive", "--no-gemini", "--disable-shell"]
        (root / "mcp.jsonl").write_text(json.dumps({
            "time": 1.0,
            "event": "mcp_request",
            "call_id": "one",
            "tool": "piper_move_joint",
            "arguments": EXPECTED_ARGUMENTS,
        }) + "\n")
        receipts = root / "piper-motion"
        receipts.mkdir()
        (receipts / "agent_joint_step_test.json").write_text(json.dumps(passed_receipt()))
        return SimpleNamespace(returncode=0)

    report = run_trial(
        config,
        config_path,
        key_file=None,
        arming_minutes=1,
        run_launcher=fake_launcher,
    )

    assert report["status"] == "passed"
    assert arming.read(root)["armed"] is False
    assert Path(report["report"]).is_file()


def test_trial_disarms_when_launcher_fails(tmp_path):
    root = tmp_path / "session"
    root.mkdir()
    config_path = root / "operator-input.json"
    config_path.write_text("{}")

    def failed_launcher(command, cwd):
        assert arming.read(root)["armed"] is True
        raise RuntimeError("model failed")

    report = run_trial(
        SimpleNamespace(root=root, agent_cli="dsh"),
        config_path,
        key_file=None,
        arming_minutes=1,
        run_launcher=failed_launcher,
    )

    assert report["status"] == "failed"
    assert report["launcher_error"] == "RuntimeError: model failed"
    assert arming.read(root)["armed"] is False


def test_shell_is_disabled_only_for_deepseek_commissioning(tmp_path):
    config_path = tmp_path / "operator-input.json"
    dsh = _launcher_command(SimpleNamespace(agent_cli="dsh"), config_path, None)
    claude = _launcher_command(SimpleNamespace(agent_cli="claude"), config_path, None)

    assert "--disable-shell" in dsh
    assert "--disable-shell" not in claude
