# Operator scripts

Scripts are trusted host entrypoints. Generated Agents never import or execute these
files; they use the MCP tools exposed by the robot frontend.

## Session entrypoints

- `configure_session.py`: create one operator-owned session configuration.
- `start_agent.sh`: launch the selected AutoRAX Harness for an existing configuration.
- `run_click_bell_experiment.sh`: create and run one complete ClickBell experiment.
- `run_task_experiment.py`: validate a manifest task, arm around the normal Harness,
  reject task-specific robot shortcuts, and summarize generic receipts, recording,
  and operator outcome.
- `reply_operator.py`: answer a pending visual-confirmation request without moving.
- `run_dashboard.sh`: inspect sessions and operator requests.

## Piper commissioning

The remaining `probe_*`, `verify_*`, `calibrate_*`, `audit_*`, `derive_*`,
`characterize_*`, and `run_piper_*` programs collect or validate operator evidence.
They are not Agent tools. Keep one only while a runtime setting or commissioning
receipt still depends on the capability it validates.

Robot behavior belongs in `services/piper/`; task goals belong in `tasks/`; Agent tool
selection belongs in `agent_skills/`. Do not add task control logic to shell wrappers.
