---
name: robot-interface
description: Inspect the dual Piper system and validate proposed joint targets during read-only bring-up. Use before interpreting Piper state or checking a target.
---

# Dual Piper interface

This backend milestone is deliberately **read-only**. It can connect to the left and
right Piper CAN interfaces, report measured state, and validate a proposed six-joint
target. It cannot command either arm or gripper.

## Tools

- `piper_status {}` returns both arms' measured joints in radians, gripper width in
  metres, feedback timestamps and ages, controller flags, and overall readiness.
- `piper_observe {}` captures the same fresh state-only observation. No camera image is
  returned in this milestone.
- `piper_check_joints {arm, positions}` checks six absolute joint targets in radians
  against configured joint-limit margins and the per-call delta cap. `dry_run: true`
  means no command was sent.

Readiness requires fresh joint feedback, controller `arm_status == 0`, and no reported
joint-limit or communication errors. A failed check is evidence about the proposed
target, not a request to clip, project, or execute it.

The represented SDK limits are J1 +/-150 deg, J2 0..180 deg, J3 -170..0 deg, J4
+/-100 deg, J5 +/-70 deg, and J6 +/-120 deg. Operator configuration adds an inward
margin and a maximum per-call delta. The result contains the effective safe minimum,
maximum, and delta for every joint.

Do not enable motors, clear faults, set zero positions, change CAN configuration, or
import `piper_sdk` from agent code. Those operations remain outside the agent boundary.
