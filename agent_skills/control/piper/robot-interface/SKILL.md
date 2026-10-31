---
name: robot-interface
description: Inspect the dual Piper system, validate targets, and use the motion tools present in the live session contract.
---

# Dual Piper interface

The Piper backend always reports measured state and validates proposed targets. Some
operator-configured sessions also expose bounded joint motion and/or generic
plan-bound Cartesian execution. Trust the live `MCP_SESSION.md` tool list: if neither
`piper_move_joint` nor `piper_execute_plan` is present, the session is read-only.

## Tools

- `piper_status {}` returns both arms' measured joints in radians, gripper width in
  metres, feedback timestamps and ages, controller flags, and overall readiness.
- `piper_observe {}` always captures fresh measured state. In an `rgb-only` or `rgbd`
  session it also returns the configured camera previews and records full-resolution
  frames; `rgbd` additionally records aligned depth and intrinsics. Inspect the live
  session contract before assuming images or depth are available. Camera extrinsics
  are not supplied unless the contract explicitly says otherwise.
- `piper_check_joints {arm, positions}` checks six absolute joint targets in radians
  against configured joint-limit margins and the per-call delta cap. `dry_run: true`
  means no command was sent.
- `piper_plan_pose {arm, position_m, rpy_rad?, relative?, waypoints?}` solves local
  numerical IK from fresh measured joints for the configured tool point and checks a
  straight Cartesian path. It
  returns joint waypoints, solver errors, joint-limit margins, and configured workspace
  refusals. Planning never moves the robot. When Cartesian motion is enabled and the
  operator has armed the session, a passed plan includes a short-lived `plan_id` and
  `execution_admissible: true`.
- `piper_execute_plan {plan_id, waypoints?}` may be present. It advances through the
  selected number of planned waypoints and holds there; omit `waypoints` to advance to
  the target. It refuses a
  changed/expired arming lease or stale start pose. On an interrupted move it holds and
  attempts to reverse only the reached path. Keep the returned receipt.
- `piper_retreat_plan {plan_id, waypoints?}` reverses the selected number of already
  executed waypoints and holds; omit `waypoints` to return to the plan start. Forward
  and reverse calls expose `progress_waypoints`, so the Agent chooses how far to back
  up before observing and replanning.
- `piper_discard_plan {plan_id}` invalidates an unexecuted staged plan without moving
  the robot. Use it after a planning probe or when a fresh observation changes the
  target.
- `piper_hold {arm}` latches the fresh measured joint pose and cancels pursuit of a
  prior target. It is available without arming when generic Cartesian motion exists.
- `piper_gripper {arm, action, width_m?}` may be present. `open` and `close` use
  commissioned endpoints; `move` requires an absolute opening in metres. It requires
  the operator arming lease and is independent of the active task.
- `piper_pixel_to_position {frame_id, camera, pixels, radius?}` may be present in a
  calibrated RGB-D session. It converts full-resolution pixels to metric points in
  the corresponding Piper arm base frame and never moves the robot.
- `piper_move_joint {arm, joint, delta_rad}` may be present. It moves exactly one
  operator-allowlisted joint by a small relative step, records measured travel, and
  re-sends the accepted target as a software-only hold. It requires an operator
  arming lease and has per-lease command and cumulative-travel budgets.

Readiness requires fresh joint feedback, controller `arm_status == 0`, and no reported
joint-limit or communication errors. A failed check is evidence about the proposed
target, not a request to clip, project, or execute it. A measured pose inside a
soft-limit margin may be held or moved toward the safe interval, but never moved
deeper into the margin.

`cartesian_motion.require_workspace_checks` is reported in status. When it is false,
the backend still enforces mechanical joint limits, per-waypoint joint change,
translation/rotation caps, feedback freshness, arming and command budgets, but it does
not reject a path using the commissioned Cartesian box or distal-link height.

The represented SDK limits are J1 +/-150 deg, J2 0..180 deg, J3 -170..0 deg, J4
+/-100 deg, J5 +/-70 deg, and J6 +/-120 deg. Operator configuration adds an inward
margin and a maximum per-call delta. The result contains the effective safe minimum,
maximum, and delta for every joint.

Never replay the joint rows returned by `piper_plan_pose` through another interface;
only pass its `plan_id` to the matching execution tool. Never infer that another arm,
joint, direction, or larger step is available. A refusal is a hard boundary. Do not
enable motors, clear faults, set zero positions, change CAN configuration, or import
`piper_sdk` from agent code. This installation uses software hold, not a safety-rated
physical stop; keep motion operator-supervised.
