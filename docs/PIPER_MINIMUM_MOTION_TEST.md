# Piper minimum-motion characterization design

The executable operator runner is `scripts/run_piper_minimum_motion.py`. The RAX Agent
backend remains read-only unless an operator explicitly binds a passed runner receipt
to the bounded Agent tool described in `PIPER_AGENT_MOTION.md`. This installation has
no confirmed physical emergency stop, so both paths deliberately use the same
reduced-protection behavior: on completion or an observed authorization failure, they
re-send the exact last accepted target. This is not safety-rated and cannot help if
the process, CAN link, controller, or power fails.

## Purpose

The first hardware movement is not a task rollout. It measures whether the proposed
position-hold path arrests a small joint command predictably on this exact robot. A
successful SDK call is evidence of command dispatch only, not evidence of physical
holding.

## Entry gates

- The passive dual-Piper probe receipt is `passed` immediately before the test.
- A private `piper_software_hold_only_confirmation` acknowledges every reduced-
  protection condition in `examples/piper_software_hold_confirmation.json`.
- The hardware-free `test_piper_hold.py` suite passes.
- No Agent, policy server, teleoperation program, or second CAN command source runs.
- The robot base is secured and the selected arm has no payload.
- The workspace and possible gravity-driven descent path are clear.
- A named operator remains at the keyboard. No person is inside the robot workspace.

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
3. Observe both arms for about `0.8 s` before dispatch. Reject the run if any joint drifts
   more than `0.0005 rad`. Record `motion_status`, but do not treat it as a motion
   detector: Piper defines it as target attainment for the previous command.
4. Record the initial dual-arm state and the native command that would be sent.
5. Require an interactive operator confirmation immediately before dispatch.
6. Command one joint on one arm within the hard cap; leave the other five targets at
   their fresh measured values and send no gripper command. Apply the additional
   `0.02 rad` mechanical-limit margin to the selected joint; unchanged joints must
   remain inside the hard mechanical limits.
7. Latch position hold and reject every later motion request before re-sending the
   exact last target to the selected arm. Do not derive a new target from feedback.
8. Record command time, hold-request time, both SDK return times, CAN transmit counters,
   feedback timestamps, per-joint travel, post-request travel, and any controller error.
   A returned `JointCtrl` call is not proof that the CAN frame was accepted.
9. End in `HOLD_LATCHED` or `RECOVERY_REQUIRED`. Manual recovery returns only to
   `READ_ONLY`; the runner never resets, disables, retries, or returns to the start
   pose. Enabling the selected arm is possible only through the explicit
   `--allow-enable` operator flag.

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

Any automatic failure leaves recovery manual. Software must not automatically reset,
return, or continue. Because no independent stop exists, the operator must keep the
workspace clear and be prepared for software hold to fail.

## Operator command

First run the plan without `--execute`. After reviewing the receipt, rerun with
`--execute`; the script requires an exact interactive phrase immediately before it
opens the robot connection. Add `--allow-enable` only when the selected arm is not
already enabled and the operator has reviewed that additional command.

```bash
python scripts/run_piper_minimum_motion.py \
  --backend-options /path/to/piper.json \
  --probe-receipt /path/to/passed-piper-readonly-probe.json \
  --confirmation /path/to/private-software-hold-confirmation.json \
  --arm right --joint 1 --delta-rad 0.002 --speed-percent 3

python scripts/run_piper_minimum_motion.py \
  --backend-options /path/to/piper.json \
  --probe-receipt /path/to/passed-piper-readonly-probe.json \
  --confirmation /path/to/private-software-hold-confirmation.json \
  --arm right --joint 1 --delta-rad 0.002 --speed-percent 3 --execute
```

## Evidence produced

The runner emits one immutable JSON receipt containing configuration,
software commit, operator identity, robot asset id, receipt hashes, timestamps, raw
feedback samples, commands, measured limits, final state, and explicit pass/fail
reasons. Video is useful secondary evidence but does not replace joint feedback.
