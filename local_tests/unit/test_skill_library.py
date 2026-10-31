import json
from pathlib import Path
import subprocess

import pytest

from conftest import configuration
from services.skill_library import (
    checkout_library,
    library_root,
    merge_workspace,
    validate_library,
)


def _git(path, *args):
    return subprocess.run(
        ["git", "-C", str(path), *args],
        check=True,
        text=True,
        capture_output=True,
    ).stdout.strip()


def _write_skill(library: Path, *, status="draft", successes=1):
    skill = library / "skills" / "observe-target"
    (skill / "code").mkdir(parents=True)
    (skill / "evidence" / "images").mkdir(parents=True)
    (skill / "SKILL.md").write_text(
        "---\n"
        "name: observe-target\n"
        "description: Observe a target from a fresh frame.\n"
        f"status: {status}\n"
        "robot: mock\n"
        "entry: code/run.py\n"
        "version: 1\n"
        "uses: []\n"
        "---\n\n# Observe target\n"
    )
    (skill / "code" / "run.py").write_text(
        "def main(ctx, **params):\n"
        "    return {'ok': True, 'measured': {}, 'why': 'observed'}\n"
    )
    rows = []
    for index in range(successes):
        image = f"evidence/images/run-{index}.jpg"
        (skill / image).write_bytes(b"not-a-real-image")
        rows.append(
            {
                "session": f"session-{index}",
                "date": "2026-10-31",
                "commit": "pending",
                "outcome": "success",
                "unattended": True,
                "conditions": "mock scene",
                "checks": {},
                "images": [image],
            }
        )
    (skill / "evidence" / "runs.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows)
    )
    (library / "INDEX.md").write_text(
        "# AutoRAX skill library\n\n"
        "| Skill | Status | What it does | When to use |\n"
        "| --- | --- | --- | --- |\n"
        f"| [observe-target](skills/observe-target/SKILL.md) | {status} | observe | target visible |\n"
    )


def test_research_checkout_merges_valid_committed_skill(tmp_path):
    config = configuration(tmp_path, agent_mode="research")
    workspace = config.root / "agent-workspace"
    workspace.mkdir(parents=True)
    checkout_library(config, workspace)
    library = workspace / "skill-library"
    _write_skill(library)
    _git(library, "add", ".")
    _git(library, "commit", "-m", "Add observe target skill")

    result = merge_workspace(config)

    assert result["status"] == "merged"
    assert (library_root("mock") / "skills/observe-target/code/run.py").is_file()


def test_verified_skill_requires_two_unattended_successes(tmp_path):
    config = configuration(tmp_path, agent_mode="research")
    workspace = config.root / "agent-workspace"
    workspace.mkdir(parents=True)
    checkout_library(config, workspace)
    library = workspace / "skill-library"
    _write_skill(library, status="verified", successes=1)

    with pytest.raises(ValueError, match="two unattended successful runs"):
        validate_library(library, robot="mock")


def test_execution_checkout_is_detached_and_cannot_merge(tmp_path):
    config = configuration(tmp_path, agent_mode="execution")
    workspace = config.root / "agent-workspace"
    workspace.mkdir(parents=True)
    result = checkout_library(config, workspace)

    assert result["branch"] is None
    with pytest.raises(PermissionError, match="research mode"):
        merge_workspace(config)


def test_library_rejects_files_outside_index_and_skills(tmp_path):
    config = configuration(tmp_path, agent_mode="research")
    workspace = config.root / "agent-workspace"
    workspace.mkdir(parents=True)
    checkout_library(config, workspace)
    library = workspace / "skill-library"
    (library / "unexpected.py").write_text("raise SystemExit\n")

    with pytest.raises(ValueError, match="unexpected top-level paths"):
        validate_library(library, robot="mock")
