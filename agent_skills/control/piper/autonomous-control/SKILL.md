---
name: autonomous-control
description: Build bounded Piper state-feedback scripts from the operator-allowlisted single-joint step tool. Use only when piper_move_joint is present in the live contract.
---

# Bounded Piper automation

This milestone is not general manipulation control. It permits scripts to sequence
only the arm, joint, direction, speed, and step envelope admitted by the operator's
characterization evidence. There is no Agent motor enable, reset, homing, gripper, or
automatic return operation, and this state-only backend has no camera success signal.

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
