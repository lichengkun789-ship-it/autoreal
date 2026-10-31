# Component boundaries

| Component | Owns | Must not own |
| --- | --- | --- |
| `services/controller/` | MCP, sessions, audit logs, provider-independent dispatch | Piper SDK calls |
| `services/piper/` | Piper SDK adaptation, tool-point geometry, generic motion/gripper tools, session recording, and receipts | Task goals, prompts, object names, or success criteria |
| `services/piper/trajectory.py` and `cartesian_motion.py` | Generic admitted waypoint execution, progress lifecycle, hold, selectable retreat, and receipts | Object detection or task-success semantics |
| `services/piper/<task_id>/` | Optional hardware behavior that genuinely cannot be composed from generic robot tools | Repackaging an ordinary task sequence as one opaque command |
| `agent_skills/` | Tool-selection workflow and measured operating guidance | Driver code or hidden limits |
| `tasks/` | Task goal, reset procedure, evaluator | CAN, SDK, arming, hardware limits |
| `harness/` | Coding-agent process and credential relay | Robot-specific behavior |
| `services/skill_library.py` | Persistent executable skill validation and controlled Git merge | Robot motion implementation |
| artifact root | Sessions, recordings, generated controllers, learned memory | Source code |

Dependencies point toward stable interfaces: the generated Agent uses MCP, the frontend
dispatches to the selected backend, and `services/task_tools.py` reads
`services/<robot>/task_adapters.json` only for exceptional adapters matching
`(robot, task)`. ClickBell has no registered adapter: its Agent Skill composes generic
Piper tools. Task packages must never import robot services.

Session `memory/` is resumable scratch state, not cross-session memory. Persistent
procedures live under `RAX_ARTIFACT_ROOT/skill-library/<robot>/`; each research session
receives its own Git branch and can merge only through `skill_library_merge` after
format and evidence validation.
