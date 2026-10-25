"""Run native DeepSeek Harness as the AutoRAX robot agent.

The trusted operator process owns credentials, the DSH profile, the command sandbox,
and the robot service. Model-selected shell commands have no network and can write
only the generated agent workspace. The robot remains reachable only through MCP.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

from harness.deepseek_cli.installation import VERSION, installation, installed_node
from harness.deepseek_cli.login import Credential, DEFAULT_MODELS
from services.controller.config import Configuration
from services.json_io import write_private_json


PROJECT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
HOME_NAME = 'dsh-home'
WORKSPACE_NAME = 'agent-workspace'
LOGIN_FILE = 'agent-login.json'
DEFAULT_GEMINI_KEY = Path.home()/'.config/rax/openrouter.key'
DEFAULT_KEYS = {
    'deepseek-api': Path.home()/'.config/rax/deepseek.key',
    'openrouter': Path.home()/'.config/rax/openrouter.key',
}
HOME = Path.home()
HOME_SECRETS = (HOME/'.ssh', HOME/'.aws', HOME/'.config/gcloud', HOME/'.netrc',
                HOME/'.git-credentials', HOME/'.claude', HOME/'.claude.json', HOME/'.codex',
                HOME/'.config/rax')
SITE_PATHS_ENVIRONMENT = 'RAX_PROTECTED_PATHS'
RESUME_PROMPT = ('This session was interrupted and has now been resumed in the same workspace. '
                 'Reread memory/PROGRESS.md, call status, and continue from the live state. '
                 'Do not repeat finished work.')


def dsh_binary(path=None):
    default = PROJECT/f'runtime/agent-backends/deepseek-harness/tools/dsh-{VERSION}/node_modules/.bin/dsh'
    binary = Path(path or os.environ.get('RAX_DSH_BINARY') or shutil.which('dsh') or default).resolve()
    if not binary.is_file():
        raise RuntimeError('DeepSeek Harness is required; pass --agent-binary or run scripts/install_dsh.py')
    installation(binary)
    return binary


def socket_path(root):
    directory = Path(f'/tmp/rax-{os.getuid()}')
    directory.mkdir(mode=0o700, exist_ok=True)
    directory.chmod(0o700)
    return directory/(hashlib.sha256(str(Path(root).resolve()).encode()).hexdigest()[:16] + '.sock')


def site_paths(config):
    listed = [*os.environ.get(SITE_PATHS_ENVIRONMENT, '').split(':'),
              *config.backend_options.get('protected_paths', ())]
    paths = []
    for entry in filter(None, listed):
        path = Path(entry).expanduser()
        if not path.is_absolute():
            raise ValueError(f'Protected paths must be absolute: {entry!r}')
        paths.append(path.resolve())
    return paths


def hidden_paths(config):
    paths = [PROJECT.resolve(), config.root.resolve(), *HOME_SECRETS, *site_paths(config)]
    existing = list(dict.fromkeys(path for path in paths if path.exists()))
    return [path for path in existing if not any(path != other and path.is_relative_to(other)
                                                  for other in existing)]


def agent_python(config):
    configured = config.backend_options.get('agent_python')
    return Path(configured or sys.executable).resolve()


def populate_workspace(config, workspace, *, gemini=True):
    from services.workspace import Setup, build, recipe_for
    for name in ('code', 'memory', 'output', 'runtime/frames', 'tmp'):
        (workspace/name).mkdir(parents=True, exist_ok=True)
    (config.root/'stream').mkdir(mode=0o755, exist_ok=True)
    (workspace/'runtime'/'stream').symlink_to(config.root/'stream', target_is_directory=True)
    built = build(recipe_for(), Setup.from_config(config, gemini=gemini), workspace)
    (workspace/'TASK.md').write_text(f'# Task: {config.task}\n\n{config.task_description.strip()}\n')
    return built


def credential(deployed):
    value = json.loads(Path(deployed['login']).read_text())
    return Credential(value['provider'], key_file=value['key_file'], base_url=value.get('base_url'))


def deploy(config, *, binary=None, model=None, key_file=None, base_url=None, gemini_key_file=None,
           disable_shell=False):
    if config.agent_cli != 'dsh':
        raise ValueError('DeepSeek launcher requires agent_cli=dsh')
    binary = dsh_binary(binary)
    node = installed_node(binary)
    login = Credential(config.agent_provider, key_file=key_file, base_url=base_url)
    root = config.root
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    workspace, home = root/WORKSPACE_NAME, root/HOME_NAME
    if (root/'state.json').exists() or workspace.exists():
        raise FileExistsError('Use a fresh session root')
    workspace.mkdir(mode=0o700)
    home.mkdir(mode=0o700)
    model = model or config.agent_model or DEFAULT_MODELS[config.agent_provider]
    write_private_json(root/LOGIN_FILE, {
        'agent_cli': 'dsh', 'provider': config.agent_provider, 'model': model,
        **login.record(), 'binary': str(binary), 'node': str(node),
        'gemini_key_file': str(gemini_key_file) if gemini_key_file else None,
        'disable_shell': bool(disable_shell),
    })
    write_private_json(root/'configuration.json', config.to_dict())
    write_private_json(root/'workspace-manifest.json',
                       populate_workspace(config, workspace, gemini=bool(gemini_key_file)))
    deployed = {'root': root, 'workspace': workspace, 'dsh_home': home, 'dsh': binary,
                'node': node, 'model': model, 'configuration': root/'configuration.json',
                'login': root/LOGIN_FILE, 'socket': socket_path(root), 'resume_session': None,
                'disable_shell': bool(disable_shell)}
    write_home(config, deployed)
    return deployed


def redeploy(config, *, binary=None):
    root = config.root
    value = json.loads((root/LOGIN_FILE).read_text())
    active_binary = dsh_binary(binary or value['binary'])
    deployed = {'root': root, 'workspace': root/WORKSPACE_NAME, 'dsh_home': root/HOME_NAME,
                'dsh': active_binary, 'node': installed_node(active_binary), 'model': value['model'],
                'configuration': root/'configuration.json', 'login': root/LOGIN_FILE,
                'socket': socket_path(root), 'resume_session': latest_session(root/HOME_NAME),
                'disable_shell': bool(value.get('disable_shell', False))}
    for key in ('workspace', 'dsh_home', 'configuration', 'login'):
        if not Path(deployed[key]).exists():
            raise FileNotFoundError(f'Cannot resume: {deployed[key]} is missing')
    credential(deployed)
    return deployed


def boundary(config, deployed, socket):
    bwrap = shutil.which('bwrap')
    if not bwrap:
        raise RuntimeError('bubblewrap is required for DeepSeek Harness command isolation')
    workspace = Path(deployed['workspace']).resolve()
    stream = (config.root/'stream').resolve()
    reexposed = [stream] if stream.exists() else []
    system_roots = [Path(path).resolve() for path in ('/usr', '/bin', '/lib', '/lib64', '/etc')
                    if Path(path).exists()]
    return {'bwrap': bwrap, 'workspace': str(workspace),
            'socket_directory': str(Path(socket).parent.resolve()),
            'hidden': [str(path) for path in hidden_paths(config)],
            'reexposed': [str(path) for path in reexposed],
            'file_read_roots': [str(workspace), *map(str, reexposed), *map(str, system_roots)]}


def patches(config, deployed, socket):
    value = json.loads(Path(deployed['login']).read_text())
    route, provider = credential(deployed).patches(value['model'], config.agent_effort)
    home = Path(deployed['dsh_home'])
    rows = [*provider,
        {'id': 'agent-default-model', 'config': {'provider': route, 'model': value['model']}},
        {'id': 'approval', 'config': {'policy': 'never'}},
        {'id': 'permission', 'config': {'defaultPreset': 'rax',
            'presets': {'rax': {'sandbox': 'workspace-write', 'approval': 'never'}}}},
        {'id': 'sandbox-policy', 'config': {'mode': 'workspace-write',
            'workspaceRoot': str(deployed['workspace'])}},
        {'id': 'sandbox', 'config': {'runnerCommand': [str(agent_python(config)), '-I',
            str(HERE/'runner.py'), str(home/'boundary.json')],
            'runnerFailureSignatures': ['bwrap:', 'RAX_DSH_SANDBOX:']}},
        {'id': 'fs-sandbox', 'name': str(HERE/'filesystem.mjs'),
            'config': {'cwd': str(deployed['workspace'])}},
        {'id': 'bash-sandbox', 'config': {'timeoutMs': min(config.session_seconds, 600) * 1000}},
        {'id': 'session-persistence-jsonl',
            'config': {'root': str(home/'sessions'), 'compression': 'none'}},
        {'insert': [{'id': 'rax-mcp', 'name': '@deepseek-ai/dsh-mcp-client', 'required': True,
            'config': {'serverName': 'robot', 'transport': 'stdio',
                'command': str(agent_python(config)),
                'args': ['-I', str(PROJECT/'harness/mcp_bridge.py'), str(socket)],
                'toolCallTimeoutMs': (config.session_seconds + 300) * 1000,
                'failOnStartupError': True}}]},
    ]
    disabled = ['tool-web', 'tool-subagent', 'tool-subagent-fork', 'tool-subagent-control',
                'tool-subagent-list-agents', 'workflow-ptc', 'tool-workflow',
                'tool-plugin-manager', 'plugin-manager', 'session-title-llm', 'tool-goal', 'plan-mode']
    if deployed.get('disable_shell'):
        disabled.extend(('tool-bash', 'tool-pwsh', 'tool-jobs'))
    return [*rows, *({'id': name, 'disabled': True} for name in disabled)]


def write_home(config, deployed, socket=None):
    socket = socket or deployed['socket']
    home = Path(deployed['dsh_home'])
    profile = home/'profiles/headless'
    profile.mkdir(parents=True, exist_ok=True)
    if not (profile/'package.json').exists():
        (profile/'package.json').write_text(json.dumps({'name': 'autorax-dsh-headless', 'private': True,
            'dependencies': {}, 'dsh': {'profile': {
                'bundles': ['@deepseek-ai/dsh-base', '@deepseek-ai/dsh-headless']}}}, indent=2)+'\n')
        (profile/'cordis.yml').write_text('[]\n')
    write_private_json(home/'boundary.json', boundary(config, deployed, socket))
    write_private_json(profile/'cordis.patch.yml', patches(config, deployed, socket))


def agent_command(deployed, *, prompt):
    command = [str(deployed['node']), str(deployed['dsh']), '--profile', 'headless', '--json']
    if deployed.get('resume_session'):
        command += ['--session-id', deployed['resume_session']]
    return [*command, prompt]


def agent_environment(config, deployed):
    environment = {key: os.environ[key] for key in ('TERM', 'LANG', 'LC_ALL', 'COLORTERM', 'TZ')
                   if key in os.environ}
    python = agent_python(config)
    environment.update({
        'HOME': str(deployed['workspace']),
        'PATH': f'{deployed["node"].parent}:{python.parent}:/usr/local/bin:/usr/bin:/bin',
        'PYTHONPATH': str(deployed['workspace']), 'PYTHONDONTWRITEBYTECODE': '1',
        'RAX_MCP_SOCKET': str(deployed['socket']), 'DSH_HOME': str(deployed['dsh_home']),
        'RAX_DSH_INSTALL': str(installation(deployed['dsh'])),
        'RAX_DSH_BOUNDARY': str(Path(deployed['dsh_home'])/'boundary.json'),
        'RAX_DSH_AUDIT': str(Path(deployed['dsh_home'])/'request-audit.jsonl'),
        'NODE_OPTIONS': f'--import={HERE/"request_audit.mjs"}',
    })
    environment.update(credential(deployed).environment())
    return environment


def latest_session(home):
    record = Path(home)/'rax-session.json'
    return json.loads(record.read_text())['session_id'] if record.exists() else None


def capture_event(deployed, line, *, seen=None):
    try:
        event = json.loads(line)
    except ValueError:
        return None
    if not isinstance(event, dict):
        return None
    home, root = Path(deployed['dsh_home']), Path(deployed['root'])
    if event.get('type') == 'session':
        identifier = event.get('sessionId') or event.get('session_id')
        if identifier:
            write_private_json(home/'rax-session.json', {'session_id': identifier})
    if event.get('type') == 'error':
        write_private_json(home/'rax-error.json', {'error': event.get('message', str(event))})
    if event.get('type') != 'status' or event.get('phase') != 'step_end':
        return None
    identity = (latest_session(home), event.get('turn'), event.get('step'))
    if seen is not None and identity in seen:
        return None
    if seen is not None:
        seen.add(identity)
    usage = event.get('usage') or {}
    row = {'time': time.time(), 'model': deployed['model'],
           'input_tokens': int(usage.get('inputTokens', 0) or 0),
           'output_tokens': int(usage.get('outputTokens', 0) or 0),
           'cache_read_input_tokens': int(usage.get('cacheReadTokens', 0) or 0)}
    with (root/'model-usage.jsonl').open('a') as stream:
        stream.write(json.dumps(row)+'\n')
    calls_file = root/'model-calls.json'
    previous = json.loads(calls_file.read_text()).get('calls', 0) if calls_file.exists() else 0
    write_private_json(calls_file, {'calls': previous + 1})
    return previous + 1


def start_robot_service(config, deployed, log, *, started_at, timeout=120):
    socket = deployed['socket']
    socket.unlink(missing_ok=True)
    args = [sys.executable, '-m', 'services.controller.frontend', '--config', str(deployed['configuration']),
            '--socket', str(socket), '--started-at', str(started_at)]
    login = json.loads(Path(deployed['login']).read_text())
    if login.get('gemini_key_file'):
        args += ['--key-file', login['gemini_key_file']]
    environment = {**os.environ, 'PYTHONPATH': str(PROJECT)}
    server = subprocess.Popen(args, cwd=PROJECT, env=environment, stdout=log, stderr=log)
    deadline = time.monotonic() + timeout
    while not socket.exists():
        if server.poll() is not None or time.monotonic() >= deadline:
            stop(server)
            raise RuntimeError(f'Robot service startup failed; inspect {config.root}/frontend.log')
        time.sleep(.1)
    return server


def stop(process, grace=10, *, group=False):
    if not process or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM) if group else process.terminate()
        process.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL) if group else process.kill()
        process.wait(timeout=10)
    except ProcessLookupError:
        pass


def launch(config, deployed, *, prompt, seconds=None):
    credential(deployed)
    write_home(config, deployed)
    server = agent = None
    started_at = time.time()
    max_calls = config.max_model_calls
    seen, limit_reached = set(), threading.Event()
    with (config.root/'frontend.log').open('ab') as log:
        try:
            server = start_robot_service(config, deployed, log, started_at=started_at)
            agent = subprocess.Popen(agent_command(deployed, prompt=prompt), cwd=deployed['workspace'],
                env=agent_environment(config, deployed), stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)

            def read_output():
                for line in agent.stdout:
                    calls = capture_event(deployed, line, seen=seen)
                    print(line, end='', flush=True)
                    if calls is not None and calls >= max_calls:
                        limit_reached.set()
                        stop(agent, group=True)
                        break

            reader = threading.Thread(target=read_output, daemon=True)
            reader.start()
            try:
                returncode = agent.wait(timeout=seconds or config.session_seconds)
            except subprocess.TimeoutExpired:
                stop(agent, group=True)
                returncode = 124
            reader.join(timeout=5)
            return 75 if limit_reached.is_set() and returncode else returncode
        finally:
            stop(agent, group=True)
            stop(server, grace=30)


def attempt_seconds(attempts):
    return sum(max(0.0, item['finished_at'] - item['started_at']) for item in attempts
               if isinstance(item.get('started_at'), (int, float))
               and isinstance(item.get('finished_at'), (int, float)))


def resumable_lifecycle(record):
    if not record.is_file():
        raise ValueError('Nothing to resume: this session was never launched')
    previous = json.loads(record.read_text())
    if previous.get('status') == 'prepared-only':
        raise ValueError('The session was only prepared; launch it instead')
    if not previous.get('finished_at'):
        previous.update(status='crashed', finished_at=previous.get('started_at', time.time()))
    return [*previous.pop('attempts', []), previous]


def default_key(provider):
    configured = os.environ.get('RAX_AGENT_KEY_FILE')
    return Path(configured).expanduser() if configured else DEFAULT_KEYS.get(provider)


def _interrupt(*_):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--agent-binary', type=Path, help='Pinned dsh executable')
    parser.add_argument('--agent-key-file', '--key-file', dest='key_file', type=Path,
                        help='Private DeepSeek/OpenRouter API key file')
    parser.add_argument('--agent-base-url', help='HTTP(S) endpoint for provider=custom')
    parser.add_argument('--agent-model', help='Override the provider default model')
    parser.add_argument('--gemini-key-file', type=Path)
    parser.add_argument('--no-gemini', action='store_true')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--disable-shell', action='store_true',
                        help='Remove shell and background-job tools from the Agent surface')
    parser.add_argument('--exec-prompt')
    parser.add_argument('--non-interactive', action='store_true',
                        help='Accepted for the common launcher; DSH is always headless')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    config = Configuration.load(args.config)
    signal.signal(signal.SIGTERM, _interrupt)
    config.root.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (config.root/'launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        record = config.root/'agent-lifecycle.json'
        lifecycle = {'pid': os.getpid(), 'started_at': time.time(), 'status': 'preparing',
                     'agent_cli': 'dsh'}
        seconds = config.session_seconds
        if args.resume:
            attempts = resumable_lifecycle(record)
            seconds -= int(attempt_seconds(attempts))
            if seconds < 60:
                raise SystemExit('The session has no wall-clock budget left to resume')
            lifecycle.update(resumed=True, attempts=attempts)
        elif record.exists():
            raise FileExistsError('This session was already launched; use --resume or a fresh configuration')
        write_private_json(record, lifecycle)
        try:
            gemini_key = None if args.no_gemini else args.gemini_key_file or (
                DEFAULT_GEMINI_KEY if DEFAULT_GEMINI_KEY.is_file() else None)
            if args.resume:
                deployed = redeploy(config, binary=args.agent_binary)
            else:
                deployed = deploy(config, binary=args.agent_binary, model=args.agent_model,
                    key_file=args.key_file or default_key(config.agent_provider),
                    base_url=args.agent_base_url, gemini_key_file=gemini_key,
                    disable_shell=args.disable_shell)
            if args.prepare_only:
                print(json.dumps({key: str(value) for key, value in deployed.items()}, indent=2))
                lifecycle['status'] = 'prepared-only'
                return
            prompt = args.exec_prompt or (RESUME_PROMPT if args.resume else
                                          (deployed['workspace']/'START_PROMPT.md').read_text().strip())
            lifecycle['status'] = 'running'
            write_private_json(record, lifecycle)
            returncode = launch(config, deployed, prompt=prompt, seconds=seconds)
            lifecycle.update(status='exited', returncode=returncode)
            raise SystemExit(returncode)
        except KeyboardInterrupt:
            lifecycle['status'] = 'stopped'
            raise SystemExit(130)
        except Exception as error:
            lifecycle.update(status='failed', error_type=type(error).__name__)
            raise
        finally:
            lifecycle['finished_at'] = time.time()
            write_private_json(record, lifecycle)


if __name__ == '__main__':
    main()
