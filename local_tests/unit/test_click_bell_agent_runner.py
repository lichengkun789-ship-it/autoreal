from scripts.run_click_bell_agent_preview_test import _evaluate


def test_preview_runner_evaluation_requires_one_passed_motion_and_disarm():
    value = _evaluate(
        0,
        [{"tool": "click_bell_approach_preview", "arguments": {}}],
        [
            {
                "kind": "piper_agent_click_bell_approach_preview",
                "status": "passed",
                "retreat": {"status": "passed"},
            }
        ],
        {"armed": False},
    )
    assert value == {"status": "passed", "reasons": []}


def test_preview_runner_evaluation_rejects_missing_motion_or_arming_cleanup():
    value = _evaluate(1, [], [], {"armed": True})
    assert value["status"] == "failed"
    assert len(value["reasons"]) == 4
