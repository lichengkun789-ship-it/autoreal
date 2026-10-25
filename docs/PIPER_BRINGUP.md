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
6. Require `local_tests/unit/test_piper_hold.py` to pass and review
   `docs/PIPER_MINIMUM_MOTION_TEST.md`, then implement the operator-only runner.
7. Measure hold latency and overrun from a very small, low-speed movement.
8. Characterize one arm at a time. Do not automatically return to the starting pose.
9. Add backend motion checks using the measured limits.
10. Expose motion tools only after disarm-during-motion and stale-feedback tests pass.
11. Add the measured values to the Piper robot-interface skill.

The Agent never enables motors, clears faults, sets zero positions, modifies gains, or
configures CAN interfaces. Those remain operator procedures outside MCP.
