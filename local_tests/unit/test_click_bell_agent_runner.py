from scripts.run_click_bell_experiment import _evaluate as evaluate_press


def test_press_runner_binds_dynamic_plan_and_complete_recording():
    plan_id = "0123456789ab"
    value = evaluate_press(
        0,
        [
            {"tool": "click_bell_plan_press", "arguments": {}},
            {"tool": "click_bell_set_gripper", "arguments": {"action": "close"}},
            {
                "tool": "click_bell_guarded_press",
                "arguments": {"plan_id": plan_id},
            },
            {"tool": "click_bell_set_gripper", "arguments": {"action": "open"}},
            {"tool": "request_operator", "arguments": {}},
            {"tool": "click_bell_confirm_visual_result", "arguments": {}},
        ],
        [
            {
                "plan_id": plan_id,
                "dynamic": True,
                "execution_admissible": True,
                "used": True,
            }
        ],
        [
            {
                "kind": "piper_agent_click_bell_guarded_press",
                "status": "contact_unverified",
                "press_id": "press",
                "retreat": {"status": "passed"},
                "admission": {"plan_id": plan_id},
            }
        ],
        [
            {"kind": "piper_agent_click_bell_gripper", "status": "passed", "action": "close"},
            {
                "kind": "piper_agent_click_bell_gripper",
                "status": "passed",
                "action": "open",
                "press_id": "press",
            },
        ],
        [{"evaluation": {"status": "passed"}}],
        [{"kind": "piper_task_episode_recording", "scope": "task", "status": "passed"}],
        {"armed": False},
    )

    assert value["status"] == "passed"
    assert value["reasons"] == []
    assert value["task_evaluation"]["status"] == "passed"


def test_press_runner_allows_one_passed_recovery_before_planning():
    plan_id = "0123456789ab"
    value = evaluate_press(
        0,
        [
            {"tool": "click_bell_recover_observation_pose", "arguments": {}},
            {"tool": "click_bell_plan_press", "arguments": {}},
            {"tool": "click_bell_set_gripper", "arguments": {"action": "close"}},
            {"tool": "click_bell_guarded_press", "arguments": {"plan_id": plan_id}},
            {"tool": "click_bell_set_gripper", "arguments": {"action": "open"}},
            {"tool": "request_operator", "arguments": {}},
            {"tool": "click_bell_confirm_visual_result", "arguments": {}},
        ],
        [{"plan_id": plan_id, "dynamic": True, "execution_admissible": True, "used": True}],
        [
            {
                "kind": "piper_agent_click_bell_guarded_press",
                "status": "contact_unverified",
                "press_id": "press",
                "retreat": {"status": "passed"},
                "admission": {"plan_id": plan_id},
            }
        ],
        [
            {"kind": "piper_agent_click_bell_gripper", "status": "passed", "action": "close"},
            {"kind": "piper_agent_click_bell_gripper", "status": "passed", "action": "open", "press_id": "press"},
        ],
        [{"evaluation": {"status": "passed"}}],
        [{"kind": "piper_task_episode_recording", "scope": "task", "status": "passed"}],
        {"armed": False},
        [{"kind": "piper_agent_click_bell_recovery", "status": "passed"}],
    )

    assert value["status"] == "passed"
    assert value["reasons"] == []


def test_press_runner_evaluates_evidence_not_an_exact_call_trace():
    plan_id = "0123456789ab"
    value = evaluate_press(
        0,
        [
            {"tool": "status", "arguments": {}},
            {"tool": "click_bell_plan_press", "arguments": {}},
            {"tool": "piper_observe", "arguments": {}},
            {"tool": "click_bell_set_gripper", "arguments": {"action": "close"}},
            {"tool": "click_bell_guarded_press", "arguments": {"plan_id": plan_id}},
            {"tool": "click_bell_set_gripper", "arguments": {"action": "open"}},
            {"tool": "request_operator", "arguments": {}},
            {"tool": "request_operator", "arguments": {}},
            {"tool": "click_bell_confirm_visual_result", "arguments": {}},
        ],
        [{"plan_id": plan_id, "dynamic": True, "execution_admissible": True, "used": True}],
        [{
            "kind": "piper_agent_click_bell_guarded_press",
            "status": "contact_unverified",
            "press_id": "press",
            "retreat": {"status": "passed"},
            "admission": {"plan_id": plan_id},
        }],
        [
            {"kind": "piper_agent_click_bell_gripper", "status": "passed", "action": "close"},
            {"kind": "piper_agent_click_bell_gripper", "status": "passed", "action": "open", "press_id": "press"},
        ],
        [{"evaluation": {"status": "passed", "reason": "operator saw the cap move"}}],
        [{"kind": "piper_task_episode_recording", "scope": "task", "status": "passed"}],
        {"armed": False},
    )

    assert value["status"] == "passed"
    assert value["reasons"] == []
