# Component boundaries

| Component | Owns | Must not own |
| --- | --- | --- |
| `services/controller/` | MCP, sessions, audit logs, provider-independent dispatch | Piper SDK calls |
| `services/piper/` | Piper SDK adaptation, state, validation, later motion execution | Task goals or prompts |
| `agent_skills/` | Tool-selection workflow and measured operating guidance | Driver code or hidden limits |
| `tasks/` | Task goal, reset procedure, evaluator | CAN, SDK, arming, hardware limits |
| `harness/` | Coding-agent process and credential relay | Robot-specific behavior |
| artifact root | Sessions, recordings, generated controllers, learned memory | Source code |

Dependencies point toward stable interfaces: tasks and generated controllers use
`auto_research_agent/api`, the API calls the MCP frontend, and the frontend dispatches
to the selected backend. Reverse imports are forbidden.
