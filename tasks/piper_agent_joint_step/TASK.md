# Piper Agent joint-step commissioning

## Goal

Use the live RAX tools to decide whether one bounded commissioning move is currently
admissible. If and only if every live gate is satisfied, command exactly one right-arm
J1 relative step of `+0.005 rad`, inspect the measured result, and stop.

## Completion criteria

- Read `piper_status` before motion and confirm `ready: true`, a live operator arming
  lease, right J1 positive in `allowed_moves`, and one remaining command.
- Call `piper_move_joint` exactly once with `arm="right"`, `joint=1`, and
  `delta_rad=0.005`.
- Require `executed: true`, `hold.state: "hold_latched"`, zero remaining command
  budget, and a motion receipt path in the reply.
- Read `piper_status` once more and report only measured facts. Record the receipt and
  final status in `memory/PROGRESS.md`.

## Constraints

- This is a commissioning test, not a manipulation task. Do not use cameras, move the
  gripper, create a controller, request another arming lease, or attempt a second move.
- If any precondition is absent or any call is refused, do not retry or substitute a
  different action. Record the reason and stop without motion.
- Do not treat a successful tool call as evidence of object-level task success.
