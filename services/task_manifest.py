"""Load task metadata and robot-specific task adapters without task-name branches."""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
import json
from pathlib import Path
import re


PROJECT = Path(__file__).resolve().parents[1]
TASKS = PROJECT / "tasks"
ADAPTER_FILE = "task_adapters.json"
IDENTIFIER = re.compile(r"[A-Za-z0-9_-]+")


@dataclass(frozen=True)
class TaskManifest:
    task_id: str
    required_capabilities: tuple[str, ...] = ()
    evaluator: str | None = None
    agent_skill: str | None = None


def _target(value: str):
    module, separator, attribute = value.partition(":")
    if not separator or not module or not attribute:
        raise ValueError(f"invalid Python target {value!r}; expected module:attribute")
    return getattr(import_module(module), attribute)


def load_task_manifest(task_id: str, *, required: bool = False) -> TaskManifest:
    if not isinstance(task_id, str) or not IDENTIFIER.fullmatch(task_id):
        raise ValueError(f"invalid task id {task_id!r}")
    path = TASKS / task_id / "manifest.json"
    if not path.is_file():
        if required:
            raise FileNotFoundError(f"task {task_id!r} has no manifest: {path}")
        return TaskManifest(task_id)
    value = json.loads(path.read_text())
    allowed = {
        "schema_version",
        "task_id",
        "required_capabilities",
        "evaluator",
        "agent_skill",
    }
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(f"{path}: unknown manifest fields {sorted(unknown)}")
    if value.get("schema_version") not in (1, 2):
        raise ValueError(f"{path}: unsupported schema_version")
    if value.get("task_id") != task_id:
        raise ValueError(f"{path}: task_id does not match its directory")
    capabilities = value.get("required_capabilities", [])
    if not isinstance(capabilities, list) or any(
        not isinstance(item, str) or not IDENTIFIER.fullmatch(item)
        for item in capabilities
    ):
        raise ValueError(f"{path}: required_capabilities must contain identifiers")
    evaluator = value.get("evaluator")
    if evaluator is not None and not isinstance(evaluator, str):
        raise ValueError(f"{path}: evaluator must be a Python target")
    skill = value.get("agent_skill")
    if skill is not None and (
        not isinstance(skill, str) or not IDENTIFIER.fullmatch(skill)
    ):
        raise ValueError(f"{path}: agent_skill must be an identifier")
    return TaskManifest(task_id, tuple(capabilities), evaluator, skill)


def task_agent_skills() -> frozenset[str]:
    skills = set()
    for path in TASKS.glob("*/manifest.json"):
        if path.parent.name.startswith("_"):
            continue
        manifest = load_task_manifest(path.parent.name)
        if manifest.agent_skill:
            skills.add(manifest.agent_skill)
    return frozenset(skills)


def adapter_target(robot: str, task_id: str) -> str | None:
    path = PROJECT / "services" / robot / ADAPTER_FILE
    if not path.is_file():
        return None
    value = json.loads(path.read_text())
    if set(value) != {"schema_version", "robot", "tasks"}:
        raise ValueError(f"{path}: expected schema_version, robot, and tasks")
    if value["schema_version"] != 1 or value["robot"] != robot:
        raise ValueError(f"{path}: invalid robot adapter manifest")
    tasks = value["tasks"]
    if not isinstance(tasks, dict):
        raise ValueError(f"{path}: tasks must be an object")
    target = tasks.get(task_id)
    if target is not None and not isinstance(target, str):
        raise ValueError(f"{path}: adapter for {task_id!r} must be a Python target")
    return target


def adapter_class(robot: str, task_id: str):
    target = adapter_target(robot, task_id)
    return _target(target) if target else None


def evaluate_task(task_id: str, evidence: dict) -> dict:
    manifest = load_task_manifest(task_id, required=True)
    if manifest.evaluator is None:
        raise ValueError(f"task {task_id!r} has no evaluator")
    evaluator = manifest.evaluator
    if evaluator.startswith("."):
        evaluator = f"tasks.{task_id}{evaluator}"
    elif "." not in evaluator.partition(":")[0]:
        evaluator = f"tasks.{task_id}.{evaluator}"
    result = _target(evaluator)(evidence)
    if not isinstance(result, dict) or result.get("status") not in {
        "passed",
        "failed",
        "unverified",
    }:
        raise ValueError(f"task {task_id!r} evaluator returned an invalid result")
    return result
