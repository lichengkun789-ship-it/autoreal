from services.task_manifest import (
    adapter_target,
    evaluate_task,
    load_task_manifest,
)
from services.task_tools import TASK_TOOLSETS, task_motion_tool_names, task_tool_names


def test_click_bell_manifest_drives_skill_and_evaluator_without_robot_adapter():
    manifest = load_task_manifest("click_bell", required=True)
    assert manifest.agent_skill == "click-bell"
    assert manifest.evaluator == "evaluator:evaluate"
    assert adapter_target("piper", "click_bell") is None
    assert ("piper", "click_bell") not in TASK_TOOLSETS


def test_click_bell_uses_no_task_specific_robot_tools():
    options = {
        "guarded_press": {"enabled": True},
        "gripper": {"enabled": True},
    }
    names = task_tool_names("click_bell", robot="piper", task_options=options)
    motion = task_motion_tool_names("click_bell", robot="piper", task_options=options)
    assert names == ()
    assert motion == ()


def test_task_evaluator_is_loaded_from_manifest():
    result = evaluate_task(
        "click_bell",
        {"visual_success": True, "visual_reason": "operator saw the cap move"},
    )
    assert result == {
        "status": "passed",
        "success": True,
        "reason": "operator saw the cap move",
    }
