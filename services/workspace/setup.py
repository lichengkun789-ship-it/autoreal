"""The facts a workspace is generated from; roles live in recipes, not here."""
from dataclasses import dataclass

from services.task_manifest import load_task_manifest
from services.task_tools import task_motion_tool_names, task_tool_names

# Words shared sources use for the robot instead of branching on it. Tool names are the
# backend's MCP tools (services/backends.py); keep them in sync with the backend.
ROBOT_FACTS = {
    'piper': {'name': 'dual Piper system', 'status': 'piper_status', 'observe': 'piper_observe',
              'move': 'piper_move_joint', 'move_joints': 'piper_move_joint',
              'gripper': 'piper_gripper', 'pixel': 'piper_pixel_to_position',
              'cartesian_plan': 'piper_plan_pose',
              'cartesian_execute': 'piper_execute_plan',
              'cartesian_retreat': 'piper_retreat_plan',
              'cartesian_discard': 'piper_discard_plan',
              'hold': 'piper_hold',
              'stream': None,
              'tools': ('piper_status', 'piper_observe', 'piper_check_joints',
                        'piper_plan_pose', 'piper_pixel_to_position', 'piper_move_joint',
                        'piper_execute_plan', 'piper_retreat_plan', 'piper_discard_plan',
                        'piper_hold',
                        'piper_gripper'),
              'motion_tools': ('piper_move_joint', 'piper_execute_plan',
                               'piper_retreat_plan', 'piper_discard_plan', 'piper_hold',
                               'piper_gripper'),
              'geometry_tools': ('piper_pixel_to_position',),
              'overview_camera': 'cam_high', 'overview': 'fixed and wrist RealSense views',
              'cameras': ('cam_high', 'cam_right_wrist', 'cam_left_wrist'),
              'camera_available': True, 'stream_available': False,
              'wrist_camera_inspection_available': False, 'motion_available': True,
              'motion_requires_opt_in': True},
    'mock': {'name': 'mock robot', 'status': 'status', 'observe': 'mock_observe', 'move': 'mock_move',
             'move_joints': 'mock_move', 'gripper': 'mock_move', 'pixel': 'mock_observe',
             'cartesian_plan': None, 'cartesian_execute': None, 'cartesian_retreat': None,
             'cartesian_discard': None,
             'hold': None,
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
    task_motion_tools: tuple[str, ...] = ()
    task_skill: str | None = None
    agent_mode: str = 'research'
    base_motion_tools: tuple[str, ...] = ()

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
        if not isinstance(self.task_motion_tools, tuple):
            raise ValueError('task_motion_tools must be a tuple')
        if self.agent_mode not in ('research', 'execution'):
            raise ValueError('agent_mode must be research or execution')
        if not isinstance(self.base_motion_tools, tuple):
            raise ValueError('base_motion_tools must be a tuple')
        if self.task_skill is None and self.task:
            object.__setattr__(self, 'task_skill', load_task_manifest(self.task).agent_skill)

    @classmethod
    def from_config(cls, config, *, gemini=True):
        agent_motion = config.backend_options.get('agent_motion') or {}
        cartesian_motion = config.backend_options.get('cartesian_motion') or {}
        gripper_motion = config.backend_options.get('gripper_motion') or {}
        enabled_motion_tools = []
        if isinstance(agent_motion, dict) and agent_motion.get('enabled') is True:
            enabled_motion_tools.append('piper_move_joint')
        if isinstance(cartesian_motion, dict) and cartesian_motion.get('enabled') is True:
            enabled_motion_tools.extend(
                ('piper_execute_plan', 'piper_retreat_plan', 'piper_discard_plan',
                 'piper_hold')
            )
        if isinstance(gripper_motion, dict) and gripper_motion.get('enabled') is True:
            enabled_motion_tools.append('piper_gripper')
        opted_in = bool(enabled_motion_tools)
        cameras = config.backend_options.get('cameras') or []
        task_options = getattr(config, 'task_options', {}) or {}
        task_specific = task_tool_names(
            config.task, robot=config.robot, task_options=task_options
        )
        task_motion = task_motion_tool_names(
            config.task, robot=config.robot, task_options=task_options
        )
        manifest = load_task_manifest(config.task)
        calibrated = (config.observation_profile == 'rgbd'
                      and any(isinstance(camera, dict) and camera.get('mount')
                              and camera.get('extrinsics') for camera in cameras))
        return cls(config.robot, config.task, config.agent_cli, config.observation_profile,
                   config.subagents, gemini, opted_in, calibrated, bool(task_motion),
                   task_specific, task_motion, manifest.agent_skill,
                   getattr(config, 'agent_mode', 'research'),
                   tuple(enabled_motion_tools))

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
        elif self.base_motion_tools:
            robot_tools.difference_update(
                set(facts['motion_tools']) - set(self.base_motion_tools)
            )
        if not self.pixel_localization_available:
            robot_tools.difference_update(facts.get('geometry_tools', ()))
        if self.pixel_localization_available:
            robot_tools.update(
                self.task_specific_tools
                or task_tool_names(self.task, robot=self.robot, task_options={})
            )
        harness_tools = {'status', 'request_operator'}
        if self.agent_mode == 'research':
            harness_tools.add('skill_library_merge')
        return tuple(sorted({*harness_tools, *robot_tools,
                             *(('gemini_generate',) if self.gemini else ())}))

    def axes(self):
        """Which skill layers a workspace takes: the robot it controls and how it perceives."""
        return {'robot': self.robot, 'perception': 'depth' if self.camera_geometry else 'rgb-only'}

    def context(self):
        """Template variables: facts only, named for readability in the sources."""
        facts = ROBOT_FACTS[self.robot]
        cartesian_available = (
            self.base_motion_available
            and bool(facts.get('cartesian_execute'))
            and (
                not self.base_motion_tools
                or facts.get('cartesian_execute') in self.base_motion_tools
            )
        )
        gripper_available = (
            self.base_motion_available
            and bool(facts.get('gripper'))
            and (
                not self.base_motion_tools
                or facts.get('gripper') in self.base_motion_tools
            )
        )
        robot = {**facts, 'camera_available': self.camera_available,
                 'motion_available': self.base_motion_available,
                 'task_motion_available': self.task_motion_enabled,
                 'pixel': facts.get('pixel') if self.pixel_localization_available else None,
                 'cartesian_plan': facts.get('cartesian_plan') if cartesian_available else None,
                 'cartesian_execute': facts.get('cartesian_execute') if cartesian_available else None,
                 'cartesian_retreat': facts.get('cartesian_retreat') if cartesian_available else None,
                 'cartesian_discard': facts.get('cartesian_discard') if cartesian_available else None,
                 'hold': facts.get('hold') if cartesian_available else None,
                 'gripper': facts.get('gripper') if gripper_available else None}
        return {
            'agent_cli': self.cli,
            'agent_mode': self.agent_mode,
            'task': self.task,
            'observation_profile': self.observation_profile,
            'camera_geometry': self.camera_geometry,
            'subagent_cap': self.subagents,
            'gemini': self.gemini,
            'robot': robot,
        }
