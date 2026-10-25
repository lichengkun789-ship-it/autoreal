"""The workspace recipe for the agent that controls the real robot."""
from .recipe import PROJECT, Recipe, SkillRule, Vocab

VOCAB = Vocab(memory='memory', notes='memory/PROGRESS.md', frames='runtime/frames/', inventory='status')

# Skill sources by axis. Every workspace takes the task skills, the perception skills of
# its observation mode ('depth' or 'rgb-only') and robot, the control skills of its robot,
# and the auto-research skills. Agents see one flat set with robot-neutral names.
SKILL_LAYERS = ('task', 'perception/common', 'perception/{perception}', 'perception/{perception}-{robot}',
                'control/common', 'control/{robot}', 'auto-research')

# Skills advertise capabilities. Leave a skill out when the selected backend cannot
# provide that capability yet, instead of teaching the agent about unavailable tools.
AGENT = Recipe(
    'real-robot/agent',
    PROJECT / 'auto_research_agent',
    SKILL_LAYERS,
    vocab=VOCAB,
    omit_skills=(
        SkillRule('gemini', lambda setup: not setup.gemini),
        SkillRule('manual-robot-control', lambda setup: not setup.motion_available),
        SkillRule('autonomous-control', lambda setup: not setup.motion_available),
        SkillRule('live-stream', lambda setup: not setup.stream_available),
        SkillRule('wrist-camera-inspection', lambda setup: not setup.wrist_camera_inspection_available),
    ),
)


def recipe_for(options=frozenset()):
    return AGENT
