---
name: manual-robot-control
description: Request and verify one operator-admitted Piper joint step from measured state. Use for supervised bounded bring-up, not general manipulation.
---

# Supervised bounded Piper control

This state-only milestone exposes no camera, gripper, homing, or general trajectory
tool. Use it only to characterize and verify the specific single-joint moves listed by
`piper_status.agent_motion.allowed_moves`.

1. Call `piper_status`. Require `ready: true`, a live arming lease, remaining command
   and cumulative-delta budget, and the intended axis and direction in the allowlist.
2. Choose a non-zero `delta_rad` no larger than the reported cap. Prefer a smaller step
   than the cap unless the operator's task explicitly requests the characterized step.
3. Call `piper_move_joint` once. Do not synthesize an absolute target or send a second
   move before reading the result.
4. Inspect measured active-joint travel, other-joint travel, post-hold drift, hold
   state, receipt path, and remaining budget. Then call `piper_status` again.
5. Stop on stale feedback, controller error, refusal, exhausted budget, unexpected
   motion, or `control_uncertain`. Ask the operator to inspect and recover; never retry
   automatically.

An MCP success means the bounded command and software-hold checks passed. It does not
mean a manipulation task succeeded. With no camera observation, report only facts
supported by joint and controller feedback.
