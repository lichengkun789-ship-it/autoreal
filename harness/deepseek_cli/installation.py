"""Pinned DeepSeek Harness installation kept outside committed source files."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


VERSION = '0.2.1-rc.2'
MINIMUM_NODE_MAJOR = 24


def node_version(node):
    output = subprocess.check_output([str(node), '--version'], text=True).strip().lstrip('v')
    try:
        return tuple(int(part) for part in output.split('.')[:3])
    except ValueError as exc:
        raise ValueError(f'Cannot parse Node.js version {output!r}') from exc


def require_node(node=None):
    binary = Path(node or os.environ.get('RAX_NODE_BINARY') or shutil.which('node') or '').resolve()
    if not binary.is_file():
        raise RuntimeError('Node.js is required; pass --node or set RAX_NODE_BINARY')
    version = node_version(binary)
    if version[0] < MINIMUM_NODE_MAJOR:
        raise RuntimeError(f'DeepSeek Harness requires Node.js {MINIMUM_NODE_MAJOR}+; found {version[0]}')
    return binary


def installation(binary):
    for parent in Path(binary).resolve().parents:
        manifest = parent/'node_modules/@deepseek-ai/dsh/package.json'
        if manifest.is_file():
            if json.loads(manifest.read_text()).get('version') != VERSION:
                raise ValueError(f'DeepSeek Harness must be pinned to {VERSION}')
            return parent
    raise ValueError(f'DeepSeek Harness must come from a pinned {VERSION} npm installation')


def installed_node(binary):
    prefix = installation(binary)
    record = prefix/'rax-installation.json'
    if not record.is_file():
        raise ValueError('DeepSeek Harness installation receipt is missing; use scripts/install_dsh.py')
    node = require_node(json.loads(record.read_text()).get('node'))
    return node


def install(prefix, cache, *, node=None, npm=None):
    prefix, cache = Path(prefix).resolve(), Path(cache).resolve()
    node = require_node(node)
    npm = Path(npm or node.with_name('npm'))
    if not npm.is_file():
        found = shutil.which('npm')
        if not found:
            raise RuntimeError('npm is required to install DeepSeek Harness')
        npm = Path(found).resolve()
    manifest = prefix/'node_modules/@deepseek-ai/dsh/package.json'
    environment = {**os.environ, 'PATH': f'{node.parent}:{os.environ.get("PATH", "")}' }
    for name in ('NODE_TLS_REJECT_UNAUTHORIZED', 'NPM_CONFIG_STRICT_SSL', 'npm_config_strict_ssl'):
        environment.pop(name, None)
    environment['NPM_CONFIG_STRICT_SSL'] = 'true'
    if not manifest.exists():
        prefix.mkdir(parents=True, exist_ok=True)
        subprocess.run([str(npm), 'install', '--ignore-scripts', '--no-audit', '--no-fund',
            '--strict-ssl=true', '--registry=https://registry.npmjs.org/',
            '--prefix', str(prefix), '--cache', str(cache), f'@deepseek-ai/dsh@{VERSION}'],
            check=True, env=environment)
    if json.loads(manifest.read_text()).get('version') != VERSION:
        raise ValueError(f'Use a separate installation prefix for DeepSeek Harness {VERSION}')
    helper = prefix/'node_modules/@deepseek-ai/dsh-subprocess-local/scripts/ensure-spawn-helper.mjs'
    subprocess.run([str(node), str(helper)], check=True, env=environment)
    lock = prefix/'package-lock.json'
    record = {'version': VERSION, 'binary': str(prefix/'node_modules/.bin/dsh'), 'node': str(node),
              'lock_sha256': hashlib.sha256(lock.read_bytes()).hexdigest()}
    (prefix/'rax-installation.json').write_text(json.dumps(record, indent=2)+'\n')
    return record
