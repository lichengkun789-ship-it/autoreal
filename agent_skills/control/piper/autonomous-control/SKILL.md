---
name: autonomous-control
description: Build Piper state-feedback programs from the motion tools present in the live contract.
---

# Piper automation

When `piper_execute_plan` is present, build Cartesian behavior with the backend-owned
transaction:

1. read `piper_status` and require fresh, healthy feedback;
2. call `piper_plan_pose` from the live state;
3. require `execution_admissible: true` and retain its `plan_id`;
4. call `piper_execute_plan` for a selected number of waypoints;
5. inspect progress, status, and a fresh observation before deciding to continue;
6. call `piper_retreat_plan` with a selected waypoint count when the strategy needs to
   back up, or omit the count for a full return.

If a planning probe stages a plan that you will not execute, call
`piper_discard_plan` before planning a different target. Discard is only valid before
the first waypoint and never moves the robot.

Plan IDs are lease-bound and cannot be remembered across sessions. Forward and reverse
calls update `progress_waypoints`; the Agent chooses each segment and regains control
between calls. Use `piper_hold` when abandoning a target before replanning. A
successful move may intentionally remain at its target so a later skill step can act
there. Record that state explicitly; do not claim the robot returned until retreat
passed. Planning a new path after completing an old one supersedes the old retreat, so
every multi-plan program needs an explicit reverse or newly planned return path.

`piper_gripper` is task-independent. `open` and `close` use commissioned endpoints;
`move` accepts an absolute width within that commissioned range. It is part of the
Agent policy, not a pre-task operator step.

The Agent is a supervisory planner, not a hard-real-time servo. Each backend call owns
deterministic command timing, feedback checks, and hold behavior. Put observations and
decisions between short segments when the task needs online adaptation.

{% if robot.task_motion_available %}
The current session exposes task-specific Piper motion tools. They are valid building
blocks for an autonomous Python controller even when `piper_move_joint` is absent.
Read the active task skill and live schemas, reacquire perception inputs, then compose
the same task tools through `api.runtime.Context`. Keep plan IDs and receipts explicit;
never replace a failed fresh plan with remembered coordinates. Test the controller in
research mode and place reusable evidence-backed procedures in `skill-library/`.

Task tools remain bounded backend operations. Writing Python changes sequencing and
decision logic; it does not widen arming, motion, contact, workspace, or call budgets.
{% else %}

When Cartesian motion is unavailable, this milestone is not general manipulation
control. It permits scripts to sequence only the arm, joint, direction, speed, and
step envelope admitted by the operator's characterization evidence. There is no Agent
motor enable, reset, or homing. Camera observations may be present in `rgb-only` or
`rgbd` sessions, but images do not provide missing motion, kinematics, calibration,
contact sensing, or an automatic task-success signal.

Before every step, call `piper_status`, require `ready: true`, inspect the arming and
remaining motion budgets, and confirm the requested axis and direction appear in
`agent_motion.allowed_moves`. Call `piper_move_joint` once, inspect its measured travel
and hold report, then reacquire status. Stop on any refusal, controller error, stale
feedback, exhausted budget, unexpected joint travel, or `control_uncertain`; never
retry a failed physical command automatically.

Keep the controller resumable and make its stopping condition a measured joint-state
condition rather than a fixed number of calls. Request operator help when a new
arming lease or a manually restored pose is needed. A new task requiring another
joint or direction first needs a new operator characterization receipt and config
allowlist entry; code must not widen the envelope.

Use the ordinary runtime API:

```python
from api.runtime import Context

with Context() as ctx:
    before = ctx.value("piper_status")
    result = ctx.value("piper_move_joint", arm="right", joint=1, delta_rad=0.002)
    after = ctx.value("piper_status")
```

Run only one controller at a time. The operator remains at the keyboard because the
software last-target hold is not safety-rated and cannot help after process, CAN, or
power failure.
{% endif %}
