# Piper bring-up checklist

Do not enable motion merely because a unit test passes. This installation uses a
reduced-protection, operator-at-keyboard software-hold mode. Agent motion is permitted
only inside the exact envelope covered by passed characterization receipts and remains
operator-supervised.

1. Pin the Piper SDK version to the robot firmware and record both versions.
2. Run `python scripts/probe_piper_readonly.py --backend-options FILE`; require a
   `passed` receipt and confirm `motion_commands_sent` is `false`.
3. Start a read-only mock session and run the full automated test suite.
4. If an independent joint display is available, run `python
   scripts/verify_piper_pose.py --backend-options FILE`, collect its readings, then
   rerun with `--confirmation FILE`. Otherwise use `--skip-independent-confirmation
   REASON` and retain the explicit `skipped` receipt. This optional cross-check does
   not replace the required passive probe in step 2.
5. Review `docs/PIPER_STOP_AND_HOLD.md`. Because no physical emergency stop is
   available, fill a private copy of `examples/piper_software_hold_confirmation.json`.
   This acknowledges reduced protection; it does not claim an emergency stop exists.
6. Require `local_tests/unit/test_piper_hold.py` and
   `local_tests/unit/test_piper_minimum_motion.py` to pass, then use the operator-only
   runner described in `docs/PIPER_MINIMUM_MOTION_TEST.md`.
7. Measure hold latency and overrun from a very small, low-speed movement.
8. Characterize one arm at a time. Do not automatically return to the starting pose.
9. Configure `agent_motion` only for characterized arm/joint/direction combinations;
   see `docs/PIPER_AGENT_MOTION.md`.
10. Require the hardware-free Agent motion, disarm-during-command, stale-feedback, and
    budget tests to pass. Then perform a supervised real-hardware Agent-tool test and
    inspect its receipt before allowing more than one command per arming lease.
11. Add newly characterized axes to configuration one at a time. Never infer coverage
    for the opposite direction or another joint from one receipt.

The Agent never enables motors, clears faults, sets zero positions, modifies gains, or
configures CAN interfaces. Those remain operator procedures outside MCP.
