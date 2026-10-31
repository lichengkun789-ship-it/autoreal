"""Repository boundaries that keep task definitions independent of robot drivers."""

import ast
import inspect
import json
from pathlib import Path

from services.piper.backend import DEFAULTS
from services.piper.contact_motion import ContactStopSettings
from services.piper.ik import plan_pose
from services.piper.safety import DEFAULT_MAX_JOINT_STEP_RAD
from services.task_tools import TASK_TOOLSETS


PROJECT = Path(__file__).resolve().parents[2]


def test_task_packages_do_not_import_piper_or_arming_implementations():
    violations = []
    for path in (PROJECT / "tasks").rglob("*.py"):
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
                if node.module == "services":
                    names.extend(f"services.{alias.name}" for alias in node.names)
            else:
                continue
            blocked = [
                name
                for name in names
                if name == "piper_sdk"
                or name.startswith("piper_sdk.")
                or name == "services.arming"
                or name.startswith("services.piper")
            ]
            violations.extend(f"{path.relative_to(PROJECT)}: {name}" for name in blocked)
    assert violations == []


def test_task_tool_registry_is_keyed_by_robot_and_task():
    assert all(
        isinstance(key, tuple)
        and len(key) == 2
        and all(isinstance(value, str) and value for value in key)
        for key in TASK_TOOLSETS
    )
    assert ("piper", "click_bell") not in TASK_TOOLSETS


def test_piper_joint_step_budget_is_consistent_across_entrypoints():
    assert DEFAULT_MAX_JOINT_STEP_RAD == 0.25
    assert DEFAULTS["max_joint_step_rad"] == DEFAULT_MAX_JOINT_STEP_RAD
    assert (
        ContactStopSettings.__dataclass_fields__["max_joint_step_rad"].default
        == DEFAULT_MAX_JOINT_STEP_RAD
    )
    assert (
        inspect.signature(plan_pose).parameters["max_joint_step_rad"].default
        == DEFAULT_MAX_JOINT_STEP_RAD
    )
    for name in ("piper.json", "piper_rgbd.json", "piper_rgbd_calibrated.json"):
        options = json.loads((PROJECT / "examples" / name).read_text())
        assert options["max_joint_step_rad"] == DEFAULT_MAX_JOINT_STEP_RAD
