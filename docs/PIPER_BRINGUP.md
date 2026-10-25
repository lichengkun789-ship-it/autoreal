# Piper bring-up checklist

Every step is completed with a person at the physical emergency stop. Do not enable
motion merely because a unit test passes.

1. Pin the Piper SDK version to the robot firmware and record both versions.
2. Run `python scripts/probe_piper_readonly.py --backend-options FILE`; require a
   `passed` receipt and confirm `motion_commands_sent` is `false`.
3. Start a read-only mock session and run the full automated test suite.
4. If an independent joint display is available, run `python
   scripts/verify_piper_pose.py --backend-options FILE`, collect its readings, then
   rerun with `--confirmation FILE`. Otherwise use `--skip-independent-confirmation
   REASON` and retain the explicit `skipped` receipt. This optional cross-check does
   not replace the required passive probe in step 2.
5. Review `docs/PIPER_STOP_AND_HOLD.md`, fill a private copy of
   `examples/piper_stop_confirmation.json`, and require a `passed` receipt from
   `scripts/verify_piper_stop_plan.py`. An SDK emergency stop is not a static hold.
6. Implement the latched position-hold path with fake interfaces, then measure hold
   latency and overrun from a very small, low-speed movement.
7. Characterize one arm at a time, returning to the starting pose after every move.
8. Add backend motion checks and tests using a fake SDK interface.
9. Expose motion tools only after disarm-during-motion and stale-feedback tests pass.
10. Add the measured values to the Piper robot-interface skill.

The Agent never enables motors, clears faults, sets zero positions, modifies gains, or
configures CAN interfaces. Those remain operator procedures outside MCP.
