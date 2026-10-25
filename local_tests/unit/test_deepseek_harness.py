"""DeepSeek Harness selection, credentials, isolation, and workspace deployment."""
import json
from pathlib import Path

import pytest

from conftest import configuration
from harness import launcher as dispatcher
from harness.deepseek_cli import launcher
from harness.deepseek_cli.login import Credential
from harness.deepseek_cli.runner import arguments


def private(path, text='test-only-secret'):
    path.write_text(text)
    path.chmod(0o600)
    return path


def fake_dsh(tmp_path):
    prefix = tmp_path/'dsh-install'
    package = prefix/'node_modules/@deepseek-ai/dsh'
    package.mkdir(parents=True)
    (package/'package.json').write_text(json.dumps({'version': '0.2.1-rc.2'}))
    binary = prefix/'node_modules/.bin/dsh'
    binary.parent.mkdir(parents=True, exist_ok=True)
    binary.write_text('#!/bin/sh\n')
    return binary


def test_configuration_and_dispatch_select_dsh(tmp_path):
    config = configuration(tmp_path, provider='deepseek-api', agent_cli='dsh')
    path = tmp_path/'operator-input.json'
    path.write_text(json.dumps(config.to_dict()))
    assert dispatcher.selected(['--config', str(path)]) is launcher
    with pytest.raises(ValueError, match='dsh agent_provider'):
        configuration(tmp_path, provider='anthropic', agent_cli='dsh')
    with pytest.raises(ValueError, match='does not support subagents'):
        configuration(tmp_path, provider='deepseek-api', agent_cli='dsh', subagents=1)


def test_credentials_never_serialize_secret_and_refuse_loose_files(tmp_path):
    key = private(tmp_path/'deepseek.key')
    credential = Credential('deepseek-api', key_file=key)
    route, rows = credential.patches('deepseek-flash', 'max')
    assert route == 'deepseek-official'
    assert key.read_text() not in json.dumps(credential.record()) + json.dumps(rows)
    key.chmod(0o644)
    with pytest.raises(ValueError, match='private'):
        Credential('deepseek-api', key_file=key)


def test_command_sandbox_has_no_network_and_only_workspace_is_writable(tmp_path):
    boundary = {'bwrap': '/usr/bin/bwrap', 'workspace': str(tmp_path/'workspace'),
                'socket_directory': str(tmp_path/'socket'), 'hidden': [], 'reexposed': []}
    result = arguments(boundary, ['/bin/sh', '-c', 'true'])
    assert '--unshare-net' in result and '--unshare-pid' in result
    assert result[-3:] == ['/bin/sh', '-c', 'true']
    assert result[result.index('--bind')+1] == boundary['workspace']


def test_deploy_builds_dsh_profile_without_exposing_key(tmp_path, monkeypatch):
    binary = fake_dsh(tmp_path)
    node = tmp_path/'node24'
    node.write_text('')
    monkeypatch.setattr(launcher, 'installed_node', lambda _: node)
    key = private(tmp_path/'deepseek.key')
    config = configuration(tmp_path, provider='deepseek-api', agent_cli='dsh')
    deployed = launcher.deploy(config, binary=binary, key_file=key)
    assert (deployed['workspace']/'AGENTS.md').is_file()
    assert not (deployed['workspace']/'CLAUDE.md').exists()
    contract = json.loads((deployed['workspace']/'MCP_CONTRACT.json').read_text())
    assert contract['agent_cli'] == 'dsh'
    profile = (deployed['dsh_home']/'profiles/headless/cordis.patch.yml').read_text()
    assert '@deepseek-ai/dsh-mcp-client' in profile
    assert key.read_text() not in profile + (config.root/'agent-login.json').read_text()
    environment = launcher.agent_environment(config, deployed)
    assert environment['RAX_MODEL_KEY'] == key.read_text()
    assert str(key) not in environment.values()


def test_commissioning_profile_can_remove_shell_tools(tmp_path, monkeypatch):
    binary = fake_dsh(tmp_path)
    node = tmp_path/'node24'
    node.write_text('')
    monkeypatch.setattr(launcher, 'installed_node', lambda _: node)
    key = private(tmp_path/'deepseek.key')
    config = configuration(tmp_path, provider='deepseek-api', agent_cli='dsh')
    deployed = launcher.deploy(
        config, binary=binary, key_file=key, disable_shell=True
    )

    login = json.loads((config.root/'agent-login.json').read_text())
    assert login['disable_shell'] is True
    rows = json.loads(
        (deployed['dsh_home']/'profiles/headless/cordis.patch.yml').read_text()
    )
    disabled = {row['id'] for row in rows if row.get('disabled') is True}
    assert {'tool-bash', 'tool-pwsh', 'tool-jobs'} <= disabled


def test_native_events_record_resume_identity_and_dashboard_usage(tmp_path):
    home = tmp_path/'dsh-home'
    home.mkdir()
    deployed = {'root': tmp_path, 'dsh_home': home, 'model': 'deepseek-flash'}
    seen = set()
    launcher.capture_event(deployed, json.dumps({'type': 'session', 'sessionId': 'one'}), seen=seen)
    event = {'type': 'status', 'phase': 'step_end', 'turn': 1, 'step': 1,
             'usage': {'inputTokens': 10, 'outputTokens': 5, 'cacheReadTokens': 3}}
    assert launcher.capture_event(deployed, json.dumps(event), seen=seen) == 1
    assert launcher.capture_event(deployed, json.dumps(event), seen=seen) is None
    assert launcher.latest_session(home) == 'one'
    usage = json.loads((tmp_path/'model-usage.jsonl').read_text())
    assert usage['input_tokens'] == 10 and usage['cache_read_input_tokens'] == 3
    assert json.loads((tmp_path/'model-calls.json').read_text())['calls'] == 1
