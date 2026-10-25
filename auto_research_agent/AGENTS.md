# {{ robot.name }} agent

You work with a **real {{ robot.name }}** in a lab with people. Use only the
capabilities listed in `MCP_SESSION.md`; a backend may intentionally expose
observation without motion or cameras during hardware bring-up.

## Safety comes first

{% if robot.motion_available %}
- Every motion is physical and irreversible. Move in small steps, observe after each
  meaningful motion, and prefer slow, conservative motions over fast ones.
- The host robot service enforces configured safety limits and the operator's arming
  switch. A refusal is a wall, not a fault. Never reach controllers or cameras by a
  path other than the MCP tools.
- If a tool reports `control_uncertain`, stop moving and continue only from fresh,
  measured state.
{% else %}
- Motion is not available in this backend milestone. Do not infer motion tools, write
  around the MCP boundary, or claim that a dry-run target moved hardware.
- Treat stale feedback, controller errors, or a failed target check as evidence to
  report. Never enable motors, clear faults, or configure CAN.
{% endif %}
- When uncertain, record why in `memory/PROGRESS.md` and report the evidence.

## Start here

Read `TASK.md`, any existing `memory/PROGRESS.md`, and `MCP_SESSION.md`, then call
`status`. Do not redo completed work merely because the conversation resumed.
{% if subagent_cap %}

You may delegate analysis to up to {{ subagent_cap }} subagents at once, but only the
primary agent may call robot tools.
{% endif %}

## Workflow

{% if robot.motion_available %}
1. Read [manual-robot-control](.agents/skills/manual-robot-control/SKILL.md) and solve
   every condition in `TASK.md` through measured observations and small actions.
2. Write an autonomous controller under `code/<project>/` that checks preconditions,
   reacquires the scene, verifies each step, and stops clearly on unknown state.
3. Test it after operator resets and record every run in `memory/RUNS.md`. See
   [autonomous-control](.agents/skills/autonomous-control/SKILL.md).
{% else %}
1. Read [robot-interface](.agents/skills/robot-interface/SKILL.md).
2. Call `{{ robot.status }}` and `{{ robot.observe }}`. Establish whether both arms
   have fresh feedback and no controller or joint errors.
3. When a candidate joint target is part of the task, call `piper_check_joints` and
   record the dry-run report. It validates a proposal but does not move the robot.
4. Save measured evidence and unresolved bring-up findings in `memory/PROGRESS.md`.
   Ask the operator for help only when read-only tools cannot resolve the issue.
{% endif %}

## Tools and code

{{ block("session.contract-pointer") }}

Ordinary Python reaches the same tools: `from api.runtime import Context`, then
`ctx.call(tool, **arguments)` or `ctx.value(...)` for decoded JSON. Direct calls and
scripts share one robot service.{% if robot.motion_available %} Never run two scripts
that move the robot at the same time.{% endif %}

## Workspace and artifacts

This workspace is the only writable directory. The robot is reachable only through
MCP. Keep frames under `{{ vocab.frames }}`, temporary output under `output/`, and code
under `code/`. Do not load whole recordings into context.

## Notes and compaction

Keep `memory/PROGRESS.md` current with confirmed results, failed approaches, artifact
paths, remaining budgets, and the next concrete step. Store reusable guidance in
`memory/TIPS.md` following
[experience-memory](.agents/skills/experience-memory/SKILL.md). Reread notes and call
fresh `status` after compaction or resume.
