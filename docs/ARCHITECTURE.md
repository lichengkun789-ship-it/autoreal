# AutoRAX architecture

AutoRAX keeps the agent harness, robot backend, task definitions, and runtime
artifacts in separate ownership boundaries.

```text
selected agent harness (Claude Code or DeepSeek Harness)
    -> generated agent workspace
    -> MCP frontend
    -> registered RAX backend
    -> Piper SDK
    -> dual Piper hardware
```

Only `services/piper/` may import `piper_sdk`. Task packages call the stable RAX
runtime API and never import a robot backend. Agent skills explain how to choose and
interpret tools; they do not implement control. Session logs, frames, generated code,
and learned task memory live under `RAX_ARTIFACT_ROOT`.

`harness/launcher.py` dispatches the operator-owned configuration to an adapter in
`harness/claude_cli/` or `harness/deepseek_cli/`. Both adapters generate the same MCP
contract and start the same single-owner robot frontend. Provider credentials and
vendor session homes remain outside the agent workspace.

## Current milestone

The Piper backend is read-only by default. It connects to the configured CAN
interfaces, reports measured dual-arm state, checks feedback freshness, and dry-runs
joint targets against joint limits and a per-call step cap.

Operator configuration can add one mutating tool, `piper_move_joint`. Admission is in
`services/piper/agent_motion.py`; SDK translation and the hold latch remain in
`services/piper/hold.py`; measured execution is shared with the operator
characterization runner in `services/piper/minimum_motion.py`. The tool is absent from
the MCP contract unless matching characterization evidence and an explicit move
allowlist are configured. Session arming lives outside the Agent workspace, and every
call writes a private receipt under the session root.

Task definitions and generated Agent code still call only MCP. They cannot import the
Piper SDK, widen the configured envelope, enable motors, reset faults, or bypass the
single robot-service owner.
