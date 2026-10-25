# AutoRAX architecture

AutoRAX keeps the agent harness, robot backend, task definitions, and runtime
artifacts in separate ownership boundaries.

```text
agent workspace
    -> MCP frontend
    -> registered RAX backend
    -> Piper SDK
    -> dual Piper hardware
```

Only `services/piper/` may import `piper_sdk`. Task packages call the stable RAX
runtime API and never import a robot backend. Agent skills explain how to choose and
interpret tools; they do not implement control. Session logs, frames, generated code,
and learned task memory live under `RAX_ARTIFACT_ROOT`.

## Current milestone

The Piper backend is read-only. It connects to the configured CAN interfaces, reports
measured dual-arm state, checks feedback freshness, and dry-runs joint targets against
joint limits and a per-call step cap. It exposes no mutating tool.

Motion remains disabled until `docs/PIPER_BRINGUP.md` records a characterized hold
operation, command rate, stop latency, measured accuracy, and operator-approved caps.
