import json

from scripts.run_task_experiment import _visual_result, evaluate_run
from services.task_manifest import evaluate_task


def test_visual_result_reads_the_latest_answered_operator_confirmation(tmp_path):
    folder = tmp_path / "operator" / "requests"
    folder.mkdir(parents=True)
    (folder / "0123456789ab.json").write_text(
        json.dumps(
            {
                "id": "0123456789ab",
                "kind": "visual_confirmation",
                "status": "answered",
            }
        )
    )
    (folder / "0123456789ab.reply.json").write_text(
        json.dumps({"reply": "成功，按钮已按下", "done": True, "by": "operator"})
    )

    result = _visual_result(tmp_path)

    assert result["visual_success"] is True
    assert evaluate_task("click_bell", result)["status"] == "passed"


def test_run_evaluation_rejects_task_specific_robot_tools():
    evidence = {
        "task_evaluation": {"status": "passed", "reason": "confirmed"},
        "recordings": [
            {
                "kind": "piper_task_episode_recording",
                "scope": "task",
                "status": "passed",
            }
        ],
        "requests": [{"tool": "click_bell_guarded_press"}],
        "plans": [{}],
        "motions": [{}],
    }

    report = evaluate_run(0, evidence, {"armed": False})

    assert report["status"] == "failed"
    assert "task-specific robot tools" in report["reasons"][0]
