"""Workspace generation follows backend capabilities instead of task-specific layout."""

import json
from pathlib import Path
import re
from types import SimpleNamespace

import pytest

from services.workspace import ROBOT_FACTS, Setup, build, recipe_for, render
from services.workspace.templates import registry

ROOT = Path(__file__).resolve().parents[2]


def test_piper_workspace_is_read_only_and_renders_without_markup(tmp_path):
    workspace = tmp_path / "ws"
    setup = Setup("piper", "generic_task", observation_profile="state-only")
    manifest = build(recipe_for(), setup, workspace)
    for path in workspace.rglob("*.md"):
        text = path.read_text()
        assert "{{" not in text and "{%" not in text, path
        assert not re.search(r"[Ff]ranka|click.?bell", text), path

    skills = {path.name for path in (workspace / ".agents/skills").iterdir()}
    assert "robot-interface" in skills
    assert "experience-memory" in skills
    assert "manual-robot-control" not in skills
    assert "autonomous-control" not in skills
    assert "live-stream" not in skills
    assert "wrist-camera-inspection" not in skills

    contract = json.loads((workspace / "MCP_CONTRACT.json").read_text())
    assert contract["robot"] == "piper"
    assert contract["agent_cli"] == "claude"
    assert set(setup.tools) == set(contract["tools"])
    assert "piper_move_joint" not in contract["tools"]
    assert "piper_plan_pose" in contract["tools"]
    assert manifest["contract"] == contract
    assert "no camera images" in (workspace / "MCP_SESSION.md").read_text()
    assert "Motion is not available" in (workspace / "AGENTS.md").read_text()


def test_piper_motion_workspace_advertises_only_the_bounded_motion_tool(tmp_path):
    workspace = tmp_path / "ws"
    setup = Setup(
        "piper",
        "generic_task",
        observation_profile="state-only",
        motion_enabled=True,
    )
    build(recipe_for(), setup, workspace)
    contract = json.loads((workspace / "MCP_CONTRACT.json").read_text())
    assert "piper_move_joint" in contract["tools"]
    skills = {path.name for path in (workspace / ".agents/skills").iterdir()}
    assert "manual-robot-control" in skills
    assert "autonomous-control" in skills
    autonomous = (workspace / ".agents/skills/autonomous-control/SKILL.md").read_text()
    assert "piper_move_joint" in autonomous
    assert "not general manipulation control" in autonomous


def test_piper_rgbd_workspace_advertises_images_without_unimplemented_camera_skills(tmp_path):
    workspace = tmp_path / "ws"
    setup = Setup("piper", "generic_task", observation_profile="rgbd")
    build(recipe_for(), setup, workspace)
    contract = (workspace / "MCP_SESSION.md").read_text()
    assert "aligned depth" in contract
    assert "Base-frame camera extrinsics are not yet available" in contract
    assert setup.context()["robot"]["camera_available"] is True
    skills = {path.name for path in (workspace / ".agents/skills").iterdir()}
    assert "live-stream" not in skills
    assert "wrist-camera-inspection" not in skills
    assert "object-localization" not in skills
    interface = (workspace / ".agents/skills/robot-interface/SKILL.md").read_text()
    assert "records aligned depth and intrinsics" in interface
    assert "No camera image is returned" not in interface


def test_calibrated_piper_rgbd_workspace_exposes_pixel_localization_skill(tmp_path):
    workspace = tmp_path / "ws"
    setup = Setup(
        "piper",
        "generic_task",
        observation_profile="rgbd",
        calibrated_geometry=True,
    )
    build(recipe_for(), setup, workspace)
    contract = json.loads((workspace / "MCP_CONTRACT.json").read_text())
    assert "piper_pixel_to_position" in contract["tools"]
    assert "object-localization" in {
        path.name for path in (workspace / ".agents/skills").iterdir()
    }
    assert "operator-supplied base-frame extrinsics" in (
        workspace / "MCP_SESSION.md"
    ).read_text()


def test_setup_detects_calibrated_geometry_from_backend_config():
    config = SimpleNamespace(
        robot="piper",
        task="generic_task",
        agent_cli="claude",
        observation_profile="rgbd",
        subagents=0,
        backend_options={
            "cameras": [
                {
                    "name": "cam_high",
                    "serial": "placeholder",
                    "mount": "right_base",
                    "extrinsics": "private/cam_high.json",
                }
            ]
        },
    )
    setup = Setup.from_config(config)
    assert setup.calibrated_geometry is True
    assert setup.pixel_localization_available is True
    assert "piper_pixel_to_position" in setup.tools


def test_click_bell_workspace_adds_only_its_read_only_perception_tool():
    setup = Setup(
        "piper",
        "click_bell",
        observation_profile="rgbd",
        calibrated_geometry=True,
    )
    assert "click_bell_localize_press_cap" in setup.tools
    assert "piper_move_joint" not in setup.tools


def test_click_bell_task_motion_adds_only_the_commissioned_preview_tool():
    setup = Setup(
        "piper",
        "click_bell",
        observation_profile="rgbd",
        calibrated_geometry=True,
        task_motion_enabled=True,
    )
    assert setup.motion_available is True
    assert setup.base_motion_available is False
    assert "click_bell_approach_preview" in setup.tools
    assert "piper_move_joint" not in setup.tools


def test_subagent_text_only_with_subagents(tmp_path):
    build(recipe_for(), Setup("piper", subagents=0), tmp_path / "none")
    build(recipe_for(), Setup("piper", subagents=2), tmp_path / "two")
    assert "subagents" not in (tmp_path / "none/AGENTS.md").read_text()
    assert "up to 2 subagents" in (tmp_path / "two/AGENTS.md").read_text()


def test_deepseek_workspace_records_the_selected_harness(tmp_path):
    workspace = tmp_path / 'dsh'
    build(recipe_for(), Setup('piper', cli='dsh'), workspace)
    assert json.loads((workspace/'MCP_CONTRACT.json').read_text())['agent_cli'] == 'dsh'


def test_blocks_registry_and_unknown_names_fail():
    assert set(registry()) == {"session.contract-pointer", "images.forwarding"}
    with pytest.raises(ValueError, match="Unknown workspace template name"):
        render("{{ nonexistent }}", Setup("piper"))
    with pytest.raises(ValueError, match="Unknown workspace block"):
        render('{{ block("missing") }}', Setup("piper"))


def test_every_template_variable_is_a_setup_fact():
    for path in [
        *ROOT.glob("auto_research_agent/*.md"),
        *ROOT.glob("agent_skills/**/*.md"),
        *ROOT.glob("agent_blocks/**/*.md"),
    ]:
        if path.name == "README.md" and path.parent.name == "agent_skills":
            continue
        render(path.read_text(), Setup("piper"))
