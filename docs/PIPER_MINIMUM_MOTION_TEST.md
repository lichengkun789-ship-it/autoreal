# Piper minimum-motion characterization design

This is the design for a later operator-run hardware test. No executable real-motion
runner exists yet, and the RAX Agent backend remains read-only.

## Purpose

The first hardware movement is not a task rollout. It measures whether the proposed
position-hold path arrests a small joint command predictably on this exact robot. A
successful SDK call is evidence of command dispatch only, not evidence of physical
holding.

## Entry gates

- The passive dual-Piper probe receipt is `passed` immediately before the test.
- The operator stop-plan receipt for this robot is `passed`.
- The hardware-free `test_piper_hold.py` suite passes.
- No Agent, policy server, teleoperation program, or second CAN command source runs.
- The selected arm has no payload and is mechanically supported.
- The workspace and possible gravity-driven descent path are clear.
- A named operator remains at the independent physical emergency stop.

Failure of any gate keeps the system in `READ_ONLY`.

## Provisional first-test envelope

| Parameter | Initial hard cap |
| --- | --- |
| Active arms | Exactly one |
| Active joints | Exactly one; prefer J1 only after checking its sweep is clear |
| Absolute commanded delta | At most `0.005 rad` (`0.286 deg`) |
| Controller speed | At most `5%` |
| Command window | At most `2 s` |
| Payload and gripper | No payload; gripper command forbidden |
| Return motion | Forbidden; recover to `READ_ONLY` and reassess first |

These are upper bounds, not evidence that the movement is safe. The operator may only
reduce them after inspecting the installation.

## Required runner behavior

1. Refuse to start unless all entry-gate receipt paths and hashes are recorded.
2. Open exactly one command owner for both arms and confirm fresh joint/status feedback.
3. Record the initial dual-arm state and the native command that would be sent.
4. Require an interactive operator confirmation immediately before dispatch.
5. Command one joint on one arm within the hard cap; leave the other five targets at
   their fresh measured values and send no gripper command.
6. Latch position hold and reject every later motion request before dispatching the
   fresh measured dual-arm hold targets.
7. Record command time, hold-request time, both SDK return times, CAN transmit counters,
   feedback timestamps, per-joint travel, post-request travel, and any controller error.
   A returned `JointCtrl` call is not proof that the CAN frame was accepted.
8. End in `HOLD_LATCHED` or `RECOVERY_REQUIRED`. Manual recovery returns only to
   `READ_ONLY`; the runner never resets, enables, retries, or returns to the start pose.

## Automatic failure conditions

- Joint or status feedback is missing, non-finite, outside known limits, or older than
  `0.1 s` at dispatch time.
- Any arm reports a controller or joint error.
- A second command source or an unexpected CAN transmission is detected.
- The requested joint delta exceeds `0.005 rad`, any other joint target changes, or a
  gripper command is present.
- Either hold SDK call fails, process timing exceeds the command window, or recording
  is incomplete.
- Total commanded-joint travel exceeds `0.010 rad`, any other joint moves more than
  `0.002 rad`, or post-hold drift exceeds `0.002 rad` during the following second.

Any automatic failure requires the operator to use the already-confirmed stop path as
the physical situation demands. Software must not automatically reset or continue.

## Evidence produced

The future runner must emit one immutable JSON receipt containing configuration,
software commit, operator identity, robot asset id, receipt hashes, timestamps, raw
feedback samples, commands, measured limits, final state, and explicit pass/fail
reasons. Video is useful secondary evidence but does not replace joint feedback.
