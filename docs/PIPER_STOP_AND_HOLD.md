# Piper stop, hold, and recovery protocol

This document defines the Piper motion boundary. It does not itself authorize motion.
The optional bounded Agent tool is admitted separately by operator configuration and
passed characterization evidence; no stop, reset, enable, disable, or recovery tool is
exposed to the Agent.

## Terms are not interchangeable

- **Physical emergency stop** is the primary emergency action. A named operator must
  be able to reach it throughout a motion test. Its behavior must be verified against
  the exact robot, controller, payload, and installation before automated movement.
- **SDK emergency stop** is `EmergencyStop(0x01)` / `MotionCtrl_1(0x01, 0, 0)`. It is
  a transmitted CAN command, not a passive operation and not a static hold. An AgileX
  maintainer states that the arm descends with damping after this command.
- **Disable** removes motor enable. It must not be used as a hold because an unsupported
  arm may fall.
- **Reset/resume** (`EmergencyStop(0x02)` / `MotionCtrl_1(0x02, 0, 0)`) is recovery,
  not stopping. It must never run automatically or be exposed to an Agent.
- **Position hold** is the operational interruption path, not the emergency path. The
  bounded motion worker latches out the current command and re-sends its exact accepted
  target to the selected arm. It does not derive a new target from moving feedback. It
  is not safety-rated.

The installed SDK's `ResetPiper` documentation explicitly warns that the arm can lose
power and fall. The upstream Piper SDK issue below also describes the damped descent
behavior of the software emergency stop:

- <https://github.com/agilexrobotics/piper_sdk/issues/20>
- <https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD>

## Selected paths

| Situation | Selected action | Owner | Current status |
| --- | --- | --- | --- |
| Normal pause, revoked authorization, or end of a bounded move | Latched last-target position hold in the single CAN-owning motion worker | Motion worker/operator supervisor | Right J1 positive minimum-motion receipt recorded; other axes require characterization |
| Immediate danger to a person, robot, or scene | Physical emergency stop | Operator | Must be confirmed on this installation |
| Physical path is unavailable and operator deliberately chooses the firmware fallback | `EmergencyStop(0x01)` | Operator only | Not exposed; expect damped descent |
| Reset, fault clear, resume, or enable | Manual recovery procedure | Operator only | Never an Agent action |
| Motor disable | Never used as hold | Operator only | May remove support |

This installation currently has no confirmed physical emergency-stop path. For the
minimum-motion characterization and evidence-matched bounded Agent sessions, the operator may use the explicitly
acknowledged `software_hold_only` mode in `docs/PIPER_MINIMUM_MOTION_TEST.md`. That
reduced-protection mode re-sends the exact last accepted target and does not satisfy or
replace a physical emergency stop. It is not approved for unattended operation.

Dashboard **disarm** only revokes authorization. It does not itself command a hold or
an emergency stop. Stopping an Agent process also must not be treated as a physical
stop mechanism.

## State and ownership rules

1. The administrative initial state is `READ_ONLY`.
2. Future motion requires a fresh passive probe, completed characterization receipts,
   and a new time-bounded operator arming lease before entering `MOTION_ARMED`.
3. A position-hold request latches `HOLD_LATCHED`: new motion is rejected even if an
   old arming lease has time remaining.
4. A physical or firmware emergency stop latches `RECOVERY_REQUIRED`. No reset or
   motion command is accepted from the Agent.
5. Recovery can transition only to `READ_ONLY`, never directly to `MOTION_ARMED`.
6. Missing state, stale feedback, process failure, or an uncertain command outcome is
   fail-closed. It cannot trigger automatic reset, resume, enable, or fault clearing.

## Required architecture

1. The physical emergency stop remains reachable by a named operator and independent
   of the Agent, RAX server, Python process, and workstation.
2. The robot-service motion worker owns the CAN control connection. The Agent may request a
   bounded motion but may not enable, reset, resume, disable, or clear faults.
3. A future operator supervisor owns a latched stop channel outside the Agent tool
   surface. The motion worker performs position hold because only one process may own
   the command connection. A process failure or stale feedback never triggers
   automatic recovery.
4. Recovery requires a fresh read-only probe, explicit operator inspection, and a new
   arming lease.
5. Stop and hold are characterized one arm at a time with no payload and with the arm
   mechanically supported over a clear descent envelope.

## Manual recovery rule

After any hold, physical emergency stop, firmware emergency stop, process crash, or
uncertain command result:

1. Keep further commands blocked and mechanically support the arm.
2. Inspect the people, workcell, payload, cables, arm pose, and clear descent envelope.
3. Let the operator perform reset, fault clearing, and enable using the approved local
   procedure. AutoRAX and the Agent do not perform these actions.
4. Return AutoRAX to `READ_ONLY` and run a fresh passive Piper probe.
5. Require fresh, error-free feedback from both arms.
6. Create a new arming lease only for the next approved test. Never reuse the prior
   lease or continue a partially executed Agent command.

Record the static operator gate before implementing motion:

```bash
python scripts/verify_piper_stop_plan.py

# After supporting the arms, clearing the descent envelope, and testing the physical
# emergency stop on this exact system, fill a private copy of the template:
python scripts/verify_piper_stop_plan.py \
  --confirmation /path/to/piper-stop-confirmation.json
```

The resulting `passed` receipt means only that the operator path and rules were
confirmed. It explicitly keeps `general_motion_authorized=false` and
`minimum_motion_characterization_ready=false`.

The hardware-free implementation is in `services/piper/hold.py`. It latches before
calling the injected SDK interfaces, rejects later motion, treats stale feedback or a
partial dual-arm dispatch as `RECOVERY_REQUIRED`, and provides no reset, enable, or
emergency-stop method. See `docs/PIPER_MINIMUM_MOTION_TEST.md` for the later physical
characterization design.

## Gates before minimum motion

- The passive Piper probe is `passed`. When an independent joint display exists, the
  `piper_pose_cross_check` is also `passed`; otherwise its non-blocking `skipped`
  receipt records why that stronger evidence is unavailable.
- The `piper_operator_stop_plan` receipt is `passed` for this exact hardware.
- The operator verifies that neither physical nor software stop creates an unsafe
  gravity-driven path.
- The fake-interface hold tests pass, then software-stop latency and post-stop travel
  are measured with one very small,
  low-speed move. A failed or interrupted measurement leaves recovery manual.
- Command interruption behavior is measured. Sending a measured pose is not
  called a hold until it is proven stable under interruption and stale feedback.
- No reset, resume, enable, disable, homing, gripper, or uncharacterized motion API is
  present in the RAX Agent backend. When Agent motion is disabled,
  `PiperBackend.mutating` remains empty.
