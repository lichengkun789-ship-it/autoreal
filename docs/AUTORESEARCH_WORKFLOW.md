# AutoResearch workflow

AutoRAX separates exploratory programming from independent execution. Both modes use
the same robot backend, MCP schemas, arming rules, recordings, and task evaluator.

## Research mode

`--agent-mode research` gives the Agent a network-isolated writable workspace and
sandboxed Python. The expected lifecycle is:

1. read `TASK.md`, the live MCP contract, and `skill-library/INDEX.md`;
2. reuse a compatible verified skill when one exists;
3. otherwise explore with measured observations and admitted robot tools;
4. write and test an end-to-end controller under `code/<project>/`;
5. package a reusable sub-task under `skill-library/skills/<slug>/`;
6. commit that library branch and call `skill_library_merge`.

The host merge tool validates the executable entry, index, evidence paths, robot name,
version and status. A `verified` skill needs at least two unattended successful runs.

## Execution mode

`--agent-mode execution` checks out the persistent robot skill library at a detached
commit. Native file tools are read-only, and shell commands see the workspace through
a read-only mount. The Agent may select and run a compatible frozen skill but cannot
patch it during evaluation. Failed procedures return to a later research session.

For ClickBell, select the mode without changing the canonical wrapper:

```bash
# Explore, write, and test a controller.
RAX_AGENT_MODE=research bash scripts/run_click_bell_experiment.sh

# Independently run a merged, frozen skill.
RAX_AGENT_MODE=execution bash scripts/run_click_bell_experiment.sh
```

## Evaluation

Task manifests name the robot-independent evaluator. Experiment runners assess saved
plans, motion receipts, recordings, and task evidence rather than requiring one exact
Agent tool-call trace. Operator visual confirmation remains the ClickBell success
signal because this installation has neither a sound nor tactile success sensor.

## Adding a task

1. Copy `tasks/_template/` and define its evaluator and `agent_skill` in
   `manifest.json`.
2. Add only robot-independent perception and success logic under `tasks/<task>/`.
3. First compose the task from generic robot tools in its Agent Skill and research
   controller. Add a robot/task adapter only when the required hardware behavior
   cannot be represented by generic observation, motion, hold, gripper, or recording
   capabilities.
4. Any exceptional adapter must report its tools from operator-owned options. Core
   workspace code must not branch on the task name.

General Piper IK is available through `piper_plan_pose`. When operator configuration
enables `cartesian_motion`, a passed plan receives a lease-bound `plan_id` that can be
consumed in caller-selected waypoint segments by `piper_execute_plan` and reversed by
a caller-selected count through `piper_retreat_plan`. The backend owns measured hold,
feedback monitoring, progress, and durable receipts; the Agent decides between calls
whether to continue, partially retreat, fully retreat, replan, or revise a research
controller. `piper_gripper` and complete-session recording are generic. Contact and
force semantics remain a Piper capability gap rather than a ClickBell-specific MCP
macro.
