"""Offline bubblewrap runner for commands selected by DeepSeek Harness."""
import json
import os
from pathlib import Path
import sys


def arguments(boundary, command):
    if not command:
        raise ValueError('Missing sandbox command')
    args = [boundary['bwrap'], '--ro-bind', '/', '/', '--dev', '/dev', '--unshare-net',
            '--unshare-pid', '--proc', '/proc', '--tmpfs', '/tmp', '--die-with-parent']
    masked = []
    for item in boundary['hidden']:
        path = Path(item)
        if path.is_dir():
            args += ['--tmpfs', str(path)]
            masked.append(str(path))
        elif path.exists():
            args += ['--ro-bind', '/dev/null', str(path)]
    for path in boundary.get('reexposed', ()):
        args += ['--ro-bind', path, path]
    workspace_mount = '--ro-bind' if boundary.get('workspace_read_only') else '--bind'
    args += [workspace_mount, boundary['workspace'], boundary['workspace'],
             '--bind', boundary['socket_directory'], boundary['socket_directory']]
    for path in masked:
        args += ['--remount-ro', path]
    return [*args, '--chdir', boundary['workspace'], '--', *command]


def main():
    try:
        boundary = json.loads(Path(sys.argv[1]).read_text())
        supplied = sys.argv[2:]
        command = supplied[supplied.index('--')+1:]
        args = arguments(boundary, command)
        allowed = ('HOME', 'PATH', 'PYTHONPATH', 'PYTHONDONTWRITEBYTECODE', 'RAX_MCP_SOCKET',
                   'TMPDIR', 'LANG', 'LC_ALL', 'TERM', 'USER', 'LOGNAME', 'TZ')
        environment = {key: value for key, value in os.environ.items() if key in allowed}
        os.execve(args[0], args, environment)
    except (OSError, ValueError, IndexError, KeyError) as error:
        print(f'RAX_DSH_SANDBOX: {type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
