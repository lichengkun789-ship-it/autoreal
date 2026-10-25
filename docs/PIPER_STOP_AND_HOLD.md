# Piper stop and hold boundary

This document defines the safety boundary for future Piper motion support. It does not
authorize motion and does not add any stop, reset, enable, disable, or motion tool to
the Agent.

## Terms are not interchangeable

- **Physical emergency stop** is the primary operator action. Its behavior must be
  verified against the exact robot, controller, payload, and installation before any
  automated movement.
- **SDK emergency stop** is `EmergencyStop(0x01)` / `MotionCtrl_1(0x01, 0, 0)`. It is
  a transmitted CAN command, not a passive operation and not a static hold. An AgileX
  maintainer states that the arm descends with damping after this command.
- **Disable** removes motor enable. It must not be used as a hold because an unsupported
  arm may fall.
- **Reset/resume** (`EmergencyStop(0x02)` / `MotionCtrl_1(0x02, 0, 0)`) is recovery,
  not stopping. It must never run automatically or be exposed to an Agent.
- **Hold** means maintaining a bounded pose after command flow stops. AutoRAX does not
  yet claim a validated hold implementation.

The installed SDK's `ResetPiper` documentation explicitly warns that the arm can lose
power and fall. The upstream Piper SDK issue below also describes the damped descent
behavior of the software emergency stop:

- <https://github.com/agilexrobotics/piper_sdk/issues/20>
- <https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD>

## Required architecture

1. The physical emergency stop remains reachable by a named operator and independent
   of the Agent, RAX server, Python process, and workstation.
2. A future motion worker owns the CAN control connection. The Agent may request a
   bounded motion but may not enable, reset, resume, disable, or clear faults.
3. A future operator supervisor owns a latched stop channel outside the Agent tool
   surface. A process failure or stale feedback transitions toward stop; it never
   triggers automatic recovery.
4. Recovery requires a fresh read-only probe, explicit operator inspection, and a new
   arming lease.
5. Stop and hold are characterized one arm at a time with no payload and with the arm
   mechanically supported over a clear descent envelope.

## Gates before minimum motion

- A `piper_pose_cross_check` receipt is `passed` using an independent joint display.
- The operator records the exact physical emergency-stop behavior for this hardware.
- The operator verifies that neither physical nor software stop creates an unsafe
  gravity-driven path.
- Software-stop latency and post-stop travel are measured with one very small,
  low-speed move. A failed or interrupted measurement leaves recovery manual.
- Command interruption behavior is measured. Sending the last measured pose is not
  called a hold until it is proven stable under interruption and stale feedback.
- No reset, resume, enable, disable, or motion API is present in the RAX Agent backend
  until all receipts pass.

Until these gates pass, `PiperBackend.mutating` remains empty.
