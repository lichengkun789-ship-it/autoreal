# AutoRAX

AutoRAX adapts [RAX](https://github.com/robot-agent-exploration/rax) to a dual-arm
Piper robot while keeping the agent harness, hardware backend, and task definitions
independent. The repository is intended to support multiple manipulation tasks; no
task is built into the Piper driver.

The current milestone is intentionally **read-only**. RAX can connect to both Piper
CAN interfaces, inspect measured state, reject stale or faulted feedback, and dry-run
absolute joint targets. It cannot command arm or gripper motion yet.

## Repository layout

| Path | Responsibility |
| --- | --- |
| `services/controller/` | Robot-neutral MCP dispatch, sessions, budgets, and audit logs |
| `services/piper/` | Piper SDK adapter, state normalization, and joint-target validation |
| `harness/` | Sandboxed coding-agent process and model relay |
| `agent_skills/` | Capability-specific instructions installed into an agent workspace |
| `auto_research_agent/` | Generated workspace template and stable Python runtime API |
| `tasks/` | Task packages: goal, reset procedure, evaluator, and tests |
| `examples/` | Operator-owned configuration examples with no secrets |
| `docs/` | Architecture, component boundaries, upstream relationship, and bring-up gates |
| `local_tests/` | Hardware-free unit tests; real-hardware tests must use the `hardware` marker |

Runtime sessions, recordings, checkpoints, datasets, and generated controllers do not
belong in Git. Store them below `RAX_ARTIFACT_ROOT`.

## Upstream model

`origin` is the AutoRAX fork and `upstream` is the official RAX repository. Development
uses branch `real/piper`, based on RAX `real/franka` as the real-robot harness reference.
Franka-specific drivers are not retained. See [docs/UPSTREAM.md](docs/UPSTREAM.md).

## Development setup

Use Python 3.11 or newer for development. The robot-service environment also needs a
Piper SDK version compatible with the installed firmware. Keep that environment and
site-specific controller checkouts outside this repository.

```bash
cd /home/edemlab/autorax_ws/autorax
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
pytest
```

The backend currently targets `piper_sdk==0.6.2`, matching the public feedback API used
by the adapter. Verify that version against the installed firmware during read-only
bring-up. No test in the default suite opens CAN.

## Read-only Piper session

1. Copy `examples/piper.json` outside the repository and set the real CAN interface
   names.
2. Put large session output on a separate data path.
3. Install the robot requirements in the repository environment.

```bash
source .venv/bin/activate
python -m pip install -r requirements-robot.txt

export RAX_ARTIFACT_ROOT=/path/to/autorax-artifacts
export RAX_BACKEND_OPTIONS=/absolute/path/to/piper.json
export RAX_PYTHON="$PWD/.venv/bin/python"

python scripts/configure_session.py \
  --task backend_bringup \
  --task-text "Verify fresh state from both Piper arms without motion."

# Use the configuration path printed by the previous command.
bash scripts/start_agent.sh --config /path/to/operator-input.json
```

`piper_status`, `piper_observe`, and `piper_check_joints` are the only Piper tools.
`piper_check_joints` is a dry run: an accepted result never means a command was sent.

## Adding tasks

Copy `tasks/_template/` to `tasks/<task_id>/`. Keep robot-independent success criteria
in `evaluator.py`, task wording in `TASK.md`, and human reset instructions in
`RESET.md`. Task code must use the stable runtime API and must not import
`services.piper` or `piper_sdk`.

## Motion gate

Motion is a separate milestone. It will be exposed only after the checklist in
[docs/PIPER_BRINGUP.md](docs/PIPER_BRINGUP.md) records a reliable hold path, command
rate, stop latency, measured tracking behavior, stale-feedback handling, and tested
disarm-during-motion behavior. Until then, adding a mutating MCP tool is out of scope.

## License

AutoRAX retains the Apache-2.0 license and attribution of the upstream RAX project.
