---
name: robot-interface
description: Inspect the dual Piper system, validate targets, and use the optional bounded joint-step tool. Use before interpreting Piper state or requesting motion.
---

# Dual Piper interface

The Piper backend always reports measured state and validates proposed targets. Some
operator-configured sessions also expose one evidence-bound motion primitive. Trust
the live `MCP_SESSION.md` tool list: absence of `piper_move_joint` means the session is
read-only.

## Tools

- `piper_status {}` returns both arms' measured joints in radians, gripper width in
  metres, feedback timestamps and ages, controller flags, and overall readiness.
- `piper_observe {}` captures the same fresh state-only observation. No camera image is
  returned in this milestone.
- `piper_check_joints {arm, positions}` checks six absolute joint targets in radians
  against configured joint-limit margins and the per-call delta cap. `dry_run: true`
  means no command was sent.
- `piper_move_joint {arm, joint, delta_rad}` may be present. It moves exactly one
  operator-allowlisted joint by a small relative step, records measured travel, and
  re-sends the accepted target as a software-only hold. It requires an operator
  arming lease and has per-lease command and cumulative-travel budgets.

Readiness requires fresh joint feedback, controller `arm_status == 0`, and no reported
joint-limit or communication errors. A failed check is evidence about the proposed
target, not a request to clip, project, or execute it.

The represented SDK limits are J1 +/-150 deg, J2 0..180 deg, J3 -170..0 deg, J4
+/-100 deg, J5 +/-70 deg, and J6 +/-120 deg. Operator configuration adds an inward
margin and a maximum per-call delta. The result contains the effective safe minimum,
maximum, and delta for every joint.

Never infer that another arm, joint, direction, or larger step is available. A refusal
is a hard boundary. Do not enable motors, clear faults, set zero positions, change CAN
configuration, automatically return, or import `piper_sdk` from agent code. Those
operations remain outside the agent boundary. This installation uses software hold,
not a safety-rated physical stop; keep motion operator-supervised.
