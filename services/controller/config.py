"""Operator-owned session configuration. No values are accepted from agent code."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import re

from harness.backends import NAMES as AGENT_CLIS, PROVIDERS as PROVIDERS_BY_CLI

AGENT_EFFORTS = ('high', 'xhigh', 'max')
PROVIDERS = tuple(dict.fromkeys(provider for cli in AGENT_CLIS for provider in PROVIDERS_BY_CLI[cli]))
PROFILES = ('rgbd', 'rgb-only', 'state-only')


@dataclass(frozen=True)
class Configuration:
    root: Path                      # session directory under the artifact root
    task: str                       # short task id, e.g. "stack_cups"
    task_description: str = ''      # TASK.md body written by the operator
    robot: str = 'piper'            # services.backends registry key
    # Backend-specific operator settings (bridge URLs, cameras, workspace box, ...).
    backend_options: dict = field(default_factory=dict)
    # Task-owned operator settings. Hardware limits remain in the robot backend.
    task_options: dict = field(default_factory=dict)
    observation_profile: str = 'state-only'
    agent_cli: str = 'claude'
    agent_provider: str = 'claude-login'
    agent_effort: str = 'xhigh'
    agent_model: str | None = None  # Provider default (Opus 5.5) when unset.
    session_seconds: int = 4 * 3600
    max_model_calls: int = 4000
    subagents: int = 0              # 0: the Agent tool is removed.

    def __post_init__(self):
        object.__setattr__(self, 'root', Path(self.root).resolve())
        from services.storage_root import ENVIRONMENT, artifact_root
        if not self.root.is_relative_to(artifact_root()):
            raise ValueError(f'Session artifacts must be under {artifact_root()} (set {ENVIRONMENT} to change it)')
        if not re.fullmatch(r'[A-Za-z0-9_-]+', self.task):
            raise ValueError('Invalid task id')
        if not isinstance(self.task_description, str):
            raise ValueError('task_description must be text')
        from services.backends import BACKENDS
        if self.robot not in BACKENDS:
            raise ValueError(f"Unknown robot; choose one of {', '.join(sorted(BACKENDS))}")
        if not isinstance(self.backend_options, dict):
            raise ValueError('backend_options must be an object')
        if not isinstance(self.task_options, dict):
            raise ValueError('task_options must be an object')
        if self.observation_profile not in PROFILES:
            raise ValueError(f'observation_profile must be one of {PROFILES}')
        if self.agent_cli not in AGENT_CLIS:
            raise ValueError(f'agent_cli must be one of {AGENT_CLIS}')
        providers = PROVIDERS_BY_CLI[self.agent_cli]
        if self.agent_provider not in providers:
            raise ValueError(f'{self.agent_cli} agent_provider must be one of {providers}')
        if self.agent_cli == 'dsh' and self.subagents:
            raise ValueError('DeepSeek Harness does not support subagents in this real-robot backend')
        if self.agent_effort not in AGENT_EFFORTS:
            raise ValueError(f'agent_effort must be one of {AGENT_EFFORTS}')
        if type(self.session_seconds) is not int or not 60 <= self.session_seconds <= 7 * 24 * 3600:
            raise ValueError('session_seconds must be 60..604800')
        if type(self.max_model_calls) is not int or self.max_model_calls < 1:
            raise ValueError('max_model_calls must be positive')
        if type(self.subagents) is not int or not 0 <= self.subagents <= 8:
            raise ValueError('subagents must be 0..8')

    @property
    def camera_geometry(self):
        return self.observation_profile == 'rgbd'

    @classmethod
    def from_dict(cls, value):
        return cls(**dict(value))

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text()))

    def to_dict(self):
        return json.loads(json.dumps(asdict(self), default=str))
