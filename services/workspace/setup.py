"""The facts a workspace is generated from; roles live in recipes, not here."""
from dataclasses import dataclass

from services.task_tools import task_tool_names

# Words shared sources use for the robot instead of branching on it. Tool names are the
# backend's MCP tools (services/backends.py); keep them in sync with the backend.
ROBOT_FACTS = {
    'piper': {'name': 'dual Piper system', 'status': 'piper_status', 'observe': 'piper_observe',
              'move': 'piper_move_joint', 'move_joints': 'piper_move_joint',
              'gripper': None, 'pixel': 'piper_pixel_to_position',
              'stream': None,
              'tools': ('piper_status', 'piper_observe', 'piper_check_joints',
                        'piper_plan_pose', 'piper_pixel_to_position', 'piper_move_joint'),
              'motion_tools': ('piper_move_joint',),
              'geometry_tools': ('piper_pixel_to_position',),
              'overview_camera': 'cam_high', 'overview': 'fixed and wrist RealSense views',
              'cameras': ('cam_high', 'cam_right_wrist', 'cam_left_wrist'),
              'camera_available': True, 'stream_available': False,
              'wrist_camera_inspection_available': False, 'motion_available': True,
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
    calibrated_geometry: bool = False
    task_motion_enabled: bool = False
    task_specific_tools: tuple[str, ...] = ()

    def __post_init__(self):
        if self.robot not in ROBOT_FACTS:
            raise ValueError(f'Unknown robot {self.robot!r}')
        if self.cli not in CLIS:
            raise ValueError('Unknown agent CLI')
        if self.observation_profile not in ('rgbd', 'rgb-only', 'state-only'):
            raise ValueError('Unknown observation profile')
        if type(self.motion_enabled) is not bool:
            raise ValueError('motion_enabled must be true or false')
        if type(self.calibrated_geometry) is not bool:
            raise ValueError('calibrated_geometry must be true or false')
        if type(self.task_motion_enabled) is not bool:
            raise ValueError('task_motion_enabled must be true or false')
        if not isinstance(self.task_specific_tools, tuple):
            raise ValueError('task_specific_tools must be a tuple')

    @classmethod
    def from_config(cls, config, *, gemini=True):
        agent_motion = config.backend_options.get('agent_motion') or {}
        opted_in = agent_motion.get('enabled') is True if isinstance(agent_motion, dict) else False
        cameras = config.backend_options.get('cameras') or []
        task_options = getattr(config, 'task_options', {}) or {}
        task_specific = task_tool_names(config.task, task_options=task_options)
        task_opted_in = any(
            name in {
                'click_bell_approach_preview',
                'click_bell_guarded_press',
                'click_bell_set_gripper',
            }
            for name in task_specific
        )
        calibrated = (config.observation_profile == 'rgbd'
                      and any(isinstance(camera, dict) and camera.get('mount')
                              and camera.get('extrinsics') for camera in cameras))
        return cls(config.robot, config.task, config.agent_cli, config.observation_profile,
                   config.subagents, gemini, opted_in, calibrated, task_opted_in,
                   task_specific)

    @property
    def camera_geometry(self):
        return self.observation_profile == 'rgbd'

    @property
    def camera_available(self):
        return (self.observation_profile != 'state-only'
                and bool(ROBOT_FACTS[self.robot]['camera_available']))

    @property
    def stream_available(self):
        return self.camera_available and bool(ROBOT_FACTS[self.robot].get('stream_available'))

    @property
    def pixel_localization_available(self):
        facts = ROBOT_FACTS[self.robot]
        return (self.observation_profile == 'rgbd' and self.calibrated_geometry
                and bool(facts.get('pixel')))

    @property
    def wrist_camera_inspection_available(self):
        facts = ROBOT_FACTS[self.robot]
        return (self.camera_available and self.base_motion_available
                and bool(facts.get('wrist_camera_inspection_available')))

    @property
    def base_motion_available(self):
        facts = ROBOT_FACTS[self.robot]
        supported = bool(facts['motion_available'])
        return supported and (self.motion_enabled if facts['motion_requires_opt_in'] else True)

    @property
    def motion_available(self):
        return self.base_motion_available or self.task_motion_enabled

    @property
    def tools(self):
        """Every MCP tool the session exposes, sorted."""
        facts = ROBOT_FACTS[self.robot]
        robot_tools = set(facts['tools'])
        if not self.base_motion_available:
            robot_tools.difference_update(facts['motion_tools'])
        if not self.pixel_localization_available:
            robot_tools.difference_update(facts.get('geometry_tools', ()))
        if self.pixel_localization_available:
            robot_tools.update(self.task_specific_tools or task_tool_names(
                self.task, motion_enabled=self.task_motion_enabled
            ))
        return tuple(sorted({'status', 'request_operator', *robot_tools,
                             *(('gemini_generate',) if self.gemini else ())}))

    def axes(self):
        """Which skill layers a workspace takes: the robot it controls and how it perceives."""
        return {'robot': self.robot, 'perception': 'depth' if self.camera_geometry else 'rgb-only'}

    def context(self):
        """Template variables: facts only, named for readability in the sources."""
        facts = ROBOT_FACTS[self.robot]
        robot = {**facts, 'camera_available': self.camera_available,
                 'motion_available': self.base_motion_available,
                 'task_motion_available': self.task_motion_enabled,
                 'pixel': facts.get('pixel') if self.pixel_localization_available else None}
        return {
            'agent_cli': self.cli,
            'task': self.task,
            'observation_profile': self.observation_profile,
            'camera_geometry': self.camera_geometry,
            'subagent_cap': self.subagents,
            'gemini': self.gemini,
            'robot': robot,
        }
