# Tasks

Task packages describe goals and evaluation. They never import a robot SDK, open a
CAN interface, or define hardware safety limits. Copy `_template` to
`tasks/<task_id>` and keep the task-specific code inside that directory.

Each task owns:

- `TASK.md`: operator-facing goal and observable completion criteria;
- `manifest.json`: stable task metadata and required capabilities;
- `evaluator.py`: result evaluation that consumes recorded evidence;
- `RESET.md`: operator scene-reset procedure;
- `tests/`: evaluator tests with synthetic fixtures.

Robot control remains behind the RAX tools exposed by `services/<robot>/backend.py`.
Generated controllers and session recordings belong under the external artifact root,
not in this source tree.

`piper_agent_joint_step/` is a hardware-commissioning task rather than an object task.
It exists to verify the complete model-to-MCP-to-backend path under one fixed,
evidence-bound command. Its evaluator consumes private session audit records and
motion receipts; it does not grant motion capability or define hardware limits.
