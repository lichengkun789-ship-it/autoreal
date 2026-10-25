"""MCP_CONTRACT.json and MCP_SESSION.md: the session's tool list in the workspace."""
import json
from pathlib import Path


def describe(setup) -> str:
    if not setup.camera_available:
        geometry = 'This backend currently supplies measured robot state and no camera images.'
    elif setup.camera_geometry:
        geometry = ('RGB images, aligned depth, and camera intrinsics are supplied. '
                    'Base-frame camera extrinsics are not yet available.')
    else:
        geometry = 'Only RGB images and measured robot state are supplied.'
    listing = ('Tools: ' + ', '.join(f'`{name}`' for name in setup.tools)
               + '. The live tool schemas give arguments and limits. `status` reports the session budgets '
               'and the live robot; `request_operator` asks the human operator for help or a scene reset.')
    return (f'# Session tools\n\nRobot: {setup.context()["robot"]["name"]} (real hardware). '
            f'Observation profile: {setup.observation_profile}.\n\n{geometry}\n\n{listing}\n')


def manifest(setup):
    return {'robot': setup.robot, 'agent_cli': setup.cli, 'observation_profile': setup.observation_profile,
            'subagents': setup.subagents, 'tools': list(setup.tools)}


def write_contract(workspace, setup):
    workspace = Path(workspace)
    (workspace / 'MCP_CONTRACT.json').write_text(json.dumps(manifest(setup), indent=2) + '\n')
    (workspace / 'MCP_SESSION.md').write_text(describe(setup))
