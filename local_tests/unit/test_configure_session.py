"""scripts/configure_session.py keeps Piper wiring in operator-owned configuration."""
import json
import os
from pathlib import Path
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[2]


def configure(tmp_path, *args, **environment):
    env = {k: v for k, v in os.environ.items() if k != 'RAX_BACKEND_OPTIONS'}
    env.update({'RAX_ARTIFACT_ROOT': str(tmp_path), **environment})
    return subprocess.run([sys.executable, str(PROJECT/'scripts/configure_session.py'), '--task', 'stack_cups',
                           '--task-text', 'Stack the cups.', *args], capture_output=True, text=True, env=env,
                          timeout=60)


def test_a_piper_session_without_arm_configuration_is_refused(tmp_path):
    result = configure(tmp_path)
    assert result.returncode == 2
    assert 'a piper session needs left and right arm CAN names' in result.stderr
    assert not (tmp_path/'sessions').exists()


def test_backend_options_default_to_the_environment_file(tmp_path):
    example = PROJECT/'examples/piper.json'
    result = configure(tmp_path, RAX_BACKEND_OPTIONS=str(example))
    assert result.returncode == 0, result.stderr
    saved = json.loads(Path(result.stdout.strip()).read_text())
    assert saved['robot'] == 'piper'
    assert saved['observation_profile'] == 'state-only'
    assert saved['backend_options'] == json.loads(example.read_text())
    assert Path(saved['root']).is_relative_to(tmp_path/'sessions')


def test_the_mock_robot_needs_no_options(tmp_path):
    result = configure(tmp_path, '--robot', 'mock')
    assert result.returncode == 0, result.stderr
    assert json.loads(Path(result.stdout.strip()).read_text())['backend_options'] == {}


def test_task_options_are_stored_separately_from_robot_options(tmp_path):
    task_options = tmp_path / 'task.json'
    task_options.write_text(json.dumps({'motion': {'enabled': False}}))
    result = configure(
        tmp_path,
        '--robot',
        'mock',
        '--task-options',
        str(task_options),
    )
    assert result.returncode == 0, result.stderr
    saved = json.loads(Path(result.stdout.strip()).read_text())
    assert saved['task_options'] == {'motion': {'enabled': False}}
    assert saved['backend_options'] == {}


def test_deepseek_harness_selects_its_provider_default(tmp_path):
    result = configure(tmp_path, '--robot', 'mock', '--agent-cli', 'dsh')
    assert result.returncode == 0, result.stderr
    saved = json.loads(Path(result.stdout.strip()).read_text())
    assert saved['agent_cli'] == 'dsh'
    assert saved['agent_provider'] == 'deepseek-api'


def test_provider_must_belong_to_the_selected_harness(tmp_path):
    result = configure(tmp_path, '--robot', 'mock', '--agent-cli', 'dsh',
                       '--agent-provider', 'anthropic')
    assert result.returncode != 0
    assert 'dsh agent_provider' in result.stderr
