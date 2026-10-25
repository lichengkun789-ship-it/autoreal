# Piper bring-up checklist

Every step is completed with a person at the physical emergency stop. Do not enable
motion merely because a unit test passes.

1. Pin the Piper SDK version to the robot firmware and record both versions.
2. Verify each configured CAN interface exists, is up, and receives fresh feedback.
3. Start a read-only mock session and run the full automated test suite.
4. Start a read-only Piper session and compare reported joints with the physical pose.
5. Characterize an operator-invoked hold/stop path before exposing any motion tool.
6. Measure stop latency and overrun from a very small, low-speed movement.
7. Characterize one arm at a time, returning to the starting pose after every move.
8. Add backend motion checks and tests using a fake SDK interface.
9. Expose motion tools only after disarm-during-motion and stale-feedback tests pass.
10. Add the measured values to the Piper robot-interface skill.

The Agent never enables motors, clears faults, sets zero positions, modifies gains, or
configures CAN interfaces. Those remain operator procedures outside MCP.
