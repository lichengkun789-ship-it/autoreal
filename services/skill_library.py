"""Persistent, Git-backed executable skill library for one robot."""

from __future__ import annotations

import ast
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess

from services.storage_root import artifact_root


SLUG = re.compile(r"[a-z0-9]+(?:-[a-z0-9]+)*")
STATUSES = {"draft", "verified", "deprecated"}
GIT_ENV = {
    **os.environ,
    "GIT_AUTHOR_NAME": "AutoRAX",
    "GIT_AUTHOR_EMAIL": "autorax@localhost",
    "GIT_COMMITTER_NAME": "AutoRAX",
    "GIT_COMMITTER_EMAIL": "autorax@localhost",
}


def library_root(robot: str) -> Path:
    return artifact_root() / "skill-library" / robot


def _git(directory: Path, *arguments: str, check: bool = True) -> str:
    result = subprocess.run(
        ["git", "-C", str(directory), *arguments],
        text=True,
        capture_output=True,
        env=GIT_ENV,
    )
    if check and result.returncode:
        message = (result.stderr or result.stdout).strip()
        raise RuntimeError(f"git {' '.join(arguments)} failed: {message}")
    return result.stdout.strip()


def ensure_library(robot: str) -> Path:
    root = library_root(robot)
    if (root / ".git").is_dir():
        return root
    root.mkdir(parents=True, mode=0o700, exist_ok=True)
    if any(root.iterdir()):
        raise ValueError(f"skill library exists but is not a Git repository: {root}")
    subprocess.run(
        ["git", "init", "-b", "main", str(root)],
        check=True,
        text=True,
        capture_output=True,
        env=GIT_ENV,
    )
    (root / "skills").mkdir()
    (root / "skills" / ".gitkeep").write_text("")
    (root / "INDEX.md").write_text(
        "# AutoRAX skill library\n\n"
        "| Skill | Status | What it does | When to use |\n"
        "| --- | --- | --- | --- |\n"
    )
    _git(root, "add", "INDEX.md", "skills/.gitkeep")
    _git(root, "commit", "-m", f"Initialize {robot} skill library")
    return root


def _frontmatter(path: Path) -> dict[str, object]:
    text = path.read_text()
    if not text.startswith("---\n") or "\n---\n" not in text[4:]:
        raise ValueError(f"{path}: SKILL.md requires YAML frontmatter")
    block = text.split("\n---\n", 1)[0][4:]
    result = {}
    for line in block.splitlines():
        key, separator, raw = line.partition(":")
        if not separator:
            raise ValueError(f"{path}: malformed frontmatter line {line!r}")
        value = raw.strip()
        if value.startswith("["):
            try:
                value = json.loads(value.replace("'", '"'))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}: invalid list for {key.strip()}") from exc
        elif value.isdigit():
            value = int(value)
        result[key.strip()] = value
    return result


def validate_library(root: Path, *, robot: str) -> dict:
    root = Path(root).resolve()
    index = root / "INDEX.md"
    skills_root = root / "skills"
    if not index.is_file() or not skills_root.is_dir():
        raise ValueError("skill library requires INDEX.md and skills/")
    unexpected = {
        path.name for path in root.iterdir() if path.name not in {".git", "INDEX.md", "skills"}
    }
    if unexpected:
        raise ValueError(f"skill library has unexpected top-level paths: {sorted(unexpected)}")
    if index.is_symlink() or skills_root.is_symlink():
        raise ValueError("skill library INDEX.md and skills/ may not be symlinks")
    for path in skills_root.iterdir():
        if path.name == ".gitkeep":
            if path.is_symlink() or not path.is_file():
                raise ValueError("skills/.gitkeep must be a regular file")
        elif not path.is_dir():
            raise ValueError(f"unexpected file in skills/: {path.name}")
    index_text = index.read_text()
    summaries = []
    for skill in sorted(path for path in skills_root.iterdir() if path.is_dir()):
        if skill.is_symlink() or not SLUG.fullmatch(skill.name):
            raise ValueError(f"invalid skill directory {skill}")
        for path in skill.rglob("*"):
            if path.is_symlink():
                raise ValueError(f"skill library may not contain symlinks: {path}")
        document = skill / "SKILL.md"
        entry = skill / "code" / "run.py"
        runs = skill / "evidence" / "runs.jsonl"
        for required in (document, entry, runs):
            if not required.is_file():
                raise ValueError(f"{skill.name}: missing {required.relative_to(skill)}")
        metadata = _frontmatter(document)
        expected = {
            "name",
            "description",
            "status",
            "robot",
            "entry",
            "version",
        }
        if not expected.issubset(metadata):
            raise ValueError(f"{skill.name}: incomplete SKILL.md frontmatter")
        if metadata["name"] != skill.name or metadata["robot"] != robot:
            raise ValueError(f"{skill.name}: name or robot does not match the library")
        if metadata["status"] not in STATUSES or metadata["entry"] != "code/run.py":
            raise ValueError(f"{skill.name}: invalid status or entry")
        if not isinstance(metadata["version"], int) or metadata["version"] < 1:
            raise ValueError(f"{skill.name}: version must be a positive integer")
        for source in (skill / "code").rglob("*.py"):
            ast.parse(source.read_text(), filename=str(source))
        if f"](skills/{skill.name}/SKILL.md)" not in index_text:
            raise ValueError(f"{skill.name}: INDEX.md has no matching row")
        evidence = []
        for number, line in enumerate(runs.read_text().splitlines(), 1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{skill.name}: invalid evidence line {number}") from exc
            if row.get("outcome") not in {"success", "failure"}:
                raise ValueError(f"{skill.name}: evidence line {number} has invalid outcome")
            images = row.get("images")
            if not isinstance(images, list) or not images:
                raise ValueError(f"{skill.name}: evidence line {number} needs images")
            for image in images:
                if not isinstance(image, str) or not image.startswith("evidence/images/"):
                    raise ValueError(f"{skill.name}: evidence image must stay under evidence/images")
                image_path = (skill / image).resolve()
                if not image_path.is_relative_to((skill / "evidence" / "images").resolve()):
                    raise ValueError(f"{skill.name}: evidence image escapes evidence/images")
                if not image_path.is_file():
                    raise ValueError(f"{skill.name}: missing evidence image {image}")
            evidence.append(row)
        if not evidence:
            raise ValueError(f"{skill.name}: at least one evidence run is required")
        unattended_successes = sum(
            row.get("outcome") == "success" and row.get("unattended") is True
            for row in evidence
        )
        if metadata["status"] == "verified" and unattended_successes < 2:
            raise ValueError(
                f"{skill.name}: verified requires two unattended successful runs"
            )
        summaries.append(
            {
                "name": skill.name,
                "status": metadata["status"],
                "version": metadata["version"],
                "runs": len(evidence),
                "unattended_successes": unattended_successes,
            }
        )
    return {"robot": robot, "skills": summaries}


def checkout_library(config, workspace: Path) -> dict:
    central = ensure_library(config.robot)
    target = Path(workspace) / "skill-library"
    if target.exists():
        raise FileExistsError(f"workspace skill library already exists: {target}")
    subprocess.run(
        ["git", "clone", "--quiet", str(central), str(target)],
        check=True,
        text=True,
        capture_output=True,
        env=GIT_ENV,
    )
    _git(target, "config", "user.name", "AutoRAX Agent")
    _git(target, "config", "user.email", "agent@autorax.local")
    if config.agent_mode == "research":
        branch = "session-" + re.sub(r"[^a-zA-Z0-9-]", "-", config.root.name)[-80:]
        _git(target, "checkout", "-q", "-b", branch)
    else:
        _git(target, "checkout", "-q", "--detach")
        branch = None
    return {
        "mode": config.agent_mode,
        "path": "skill-library",
        "branch": branch,
        "commit": _git(target, "rev-parse", "HEAD"),
    }


def library_status(config) -> dict:
    workspace = config.root / "agent-workspace" / "skill-library"
    if not (workspace / ".git").is_dir():
        return {"available": False, "mode": config.agent_mode}
    branch = _git(workspace, "branch", "--show-current") or None
    return {
        "available": True,
        "mode": config.agent_mode,
        "branch": branch,
        "commit": _git(workspace, "rev-parse", "HEAD"),
        "dirty": bool(_git(workspace, "status", "--porcelain")),
        "merge_available": config.agent_mode == "research",
    }


def merge_workspace(config) -> dict:
    if config.agent_mode != "research":
        raise PermissionError("skill library merge is available only in research mode")
    workspace = config.root / "agent-workspace" / "skill-library"
    central = ensure_library(config.robot)
    if _git(workspace, "status", "--porcelain"):
        raise ValueError("commit all skill-library changes before requesting a merge")
    branch = _git(workspace, "branch", "--show-current")
    if not branch or branch == "main":
        raise ValueError("session skill library must be on its own branch")
    if int(_git(workspace, "rev-list", "--count", "origin/main..HEAD")) < 1:
        raise ValueError("skill library branch has no committed contribution")
    validation = validate_library(workspace, robot=config.robot)
    lock_path = central.parent / f".{config.robot}.merge.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if _git(central, "status", "--porcelain"):
            raise RuntimeError("persistent skill library is not clean")
        incoming = "incoming-" + re.sub(r"[^a-zA-Z0-9-]", "-", config.root.name)[-64:]
        _git(central, "fetch", str(workspace), f"HEAD:refs/heads/{incoming}")
        try:
            _git(central, "merge", "--no-ff", "--no-edit", incoming)
            merged = _git(central, "rev-parse", "HEAD")
        except BaseException:
            _git(central, "merge", "--abort", check=False)
            raise
        finally:
            _git(central, "branch", "-D", incoming, check=False)
    return {
        "status": "merged",
        "robot": config.robot,
        "branch": branch,
        "commit": merged,
        "validation": validation,
    }
