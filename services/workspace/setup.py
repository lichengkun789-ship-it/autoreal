"""The facts a workspace is generated from; roles live in recipes, not here."""
from dataclasses import dataclass

# Words shared sources use for the robot instead of branching on it. Tool names are the
# backend's MCP tools (services/backends.py); keep them in sync with the backend.
ROBOT_FACTS = {
    'piper': {'name': 'dual Piper system', 'status': 'piper_status', 'observe': 'piper_observe',
              'move': 'piper_move_joint', 'move_joints': 'piper_move_joint',
              'gripper': None, 'pixel': None,
              'stream': None,
              'tools': ('piper_status', 'piper_observe', 'piper_check_joints', 'piper_move_joint'),
              'motion_tools': ('piper_move_joint',),
              'overview_camera': None, 'overview': 'read-only robot-state observation',
              'cameras': (), 'camera_available': False, 'motion_available': True,
              'motion_requires_opt_in': True},
    'mock': {'name': 'mock robot', 'status': 'status', 'observe': 'mock_observe', 'move': 'mock_move',
             'move_joints': 'mock_move', 'gripper': 'mock_move', 'pixel': 'mock_observe',
             'stream': None, 'tools': ('mock_observe', 'mock_move'), 'overview_camera': 'none',
             'overview': 'no camera', 'cameras': (), 'camera_available': False,
             'motion_available': True, 'motion_tools': ('mock_move',),
             'motion_requires_opt_in': False},
}
CLIS = ('claude', 'dsh')


@dataclass(frozen=True)
class Setup:
    robot: str = 'piper'
    task: str | None = None
    cli: str = 'claude'
    observation_profile: str = 'state-only'
    subagents: int = 0
    gemini: bool = True
    motion_enabled: bool = False

    def __post_init__(self):
        if self.robot not in ROBOT_FACTS:
            raise ValueError(f'Unknown robot {self.robot!r}')
        if self.cli not in CLIS:
            raise ValueError('Unknown agent CLI')
        if self.observation_profile not in ('rgbd', 'rgb-only', 'state-only'):
            raise ValueError('Unknown observation profile')
        if type(self.motion_enabled) is not bool:
            raise ValueError('motion_enabled must be true or false')

    @classmethod
    def from_config(cls, config, *, gemini=True):
        agent_motion = config.backend_options.get('agent_motion') or {}
        opted_in = agent_motion.get('enabled') is True if isinstance(agent_motion, dict) else False
        return cls(config.robot, config.task, config.agent_cli, config.observation_profile,
                   config.subagents, gemini, opted_in)

    @property
    def camera_geometry(self):
        return self.observation_profile == 'rgbd'

    @property
    def camera_available(self):
        return bool(ROBOT_FACTS[self.robot]['camera_available'])

    @property
    def motion_available(self):
        facts = ROBOT_FACTS[self.robot]
        supported = bool(facts['motion_available'])
        return supported and (self.motion_enabled if facts['motion_requires_opt_in'] else True)

    @property
    def tools(self):
        """Every MCP tool the session exposes, sorted."""
        facts = ROBOT_FACTS[self.robot]
        robot_tools = set(facts['tools'])
        if not self.motion_available:
            robot_tools.difference_update(facts['motion_tools'])
        return tuple(sorted({'status', 'request_operator', *robot_tools,
                             *(('gemini_generate',) if self.gemini else ())}))

    def axes(self):
        """Which skill layers a workspace takes: the robot it controls and how it perceives."""
        return {'robot': self.robot, 'perception': 'depth' if self.camera_geometry else 'rgb-only'}

    def context(self):
        """Template variables: facts only, named for readability in the sources."""
        robot = {**ROBOT_FACTS[self.robot], 'motion_available': self.motion_available}
        return {
            'agent_cli': self.cli,
            'task': self.task,
            'observation_profile': self.observation_profile,
            'camera_geometry': self.camera_geometry,
            'subagent_cap': self.subagents,
            'gemini': self.gemini,
            'robot': robot,
        }
