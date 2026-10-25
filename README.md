# AutoRAX

AutoRAX adapts [RAX](https://github.com/robot-agent-exploration/rax) to a dual-arm
Piper robot while keeping the agent harness, hardware backend, and task definitions
independent. The repository is intended to support multiple manipulation tasks; no
task is built into the Piper driver.

The default milestone is intentionally **read-only**. RAX can connect to both Piper
CAN interfaces, inspect measured state, reject stale or faulted feedback, dry-run
absolute joint targets, and optionally capture configured RealSense RGB or RGB-D
observations. An operator may opt a session into the evidence-bound `piper_move_joint`
tool after a matching minimum-motion characterization. Gripper, homing, enable, reset,
Cartesian motion, and general trajectory control remain unavailable.

## Repository layout

| Path | Responsibility |
| --- | --- |
| `services/controller/` | Robot-neutral MCP dispatch, sessions, budgets, and audit logs |
| `services/piper/` | Piper SDK adapter, RealSense capture, state normalization, and safety validation |
| `harness/` | Agent selection plus isolated Claude Code and DeepSeek Harness adapters |
| `agent_skills/` | Capability-specific instructions installed into an agent workspace |
| `auto_research_agent/` | Generated workspace template and stable Python runtime API |
| `tasks/` | Task packages: goal, reset procedure, evaluator, and tests |
| `examples/` | Operator-owned configuration examples with no secrets |
| `docs/` | Architecture, component boundaries, upstream relationship, and bring-up gates |
| `local_tests/` | Hardware-free unit tests; real-hardware tests must use the `hardware` marker |

Runtime sessions, recordings, checkpoints, datasets, and generated controllers do not
belong in Git. Store them below `RAX_ARTIFACT_ROOT`.

## Upstream model

`origin` is the independently owned `autoreal` repository and `rax-upstream` tracks
the official RAX repository. Development uses `main`; RAX `real/franka` is retained as
the real-robot architecture reference, but Franka-specific drivers are not retained.
See [docs/UPSTREAM.md](docs/UPSTREAM.md).

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

## Agent harnesses

Sessions select the agent independently from the robot backend. `claude` remains the
default; `dsh` runs the pinned native DeepSeek Harness with `deepseek-api`, `openrouter`,
or a compatible custom endpoint.

```bash
python scripts/configure_session.py \
  --robot mock \
  --task dsh_smoke \
  --task-text "Inspect status without motion." \
  --agent-cli dsh \
  --agent-provider deepseek-api \
  --subagents 0
```

DeepSeek Harness requires Node.js 24+, a pinned local DSH installation, and a private
key file outside the checkout. See [docs/AGENT_HARNESSES.md](docs/AGENT_HARNESSES.md)
for installation, credential handling, `prepare-only` validation, and launch commands.
Changing the harness does not change the Piper tool set or enable motion.

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

With the example's default `agent_motion.enabled: false`, `piper_status`,
`piper_observe`, and `piper_check_joints` are the only Piper tools.
`piper_check_joints` is a dry run: an accepted result never means a command was sent.

### Optional RealSense observation

Use `examples/piper_rgbd.json` as the shape of a private backend-options file and
replace every placeholder serial number. Camera order is bound by serial number rather
than USB enumeration order. Then select the profile when creating a session:

```bash
.venv/bin/python scripts/list_piper_cameras.py

python scripts/configure_session.py \
  --task scene_observation \
  --task-text "Observe the workspace without moving either arm." \
  --backend-options /absolute/path/to/private-piper-rgbd.json \
  --observation-profile rgbd
```

`rgb-only` records RGB without depth; `state-only` never opens a camera. Each
`piper_observe` call writes full frames and `meta.json` below the session's
`agent-workspace/runtime/frames/` directory. The current implementation has camera
intrinsics but no camera-to-robot extrinsics or pixel-to-base conversion, so visual
inspection is available while vision-guided Cartesian motion is not. See
[docs/PIPER_CAMERAS.md](docs/PIPER_CAMERAS.md).

Before starting an agent session, run the operator-side read-only probe. It checks the
Linux interfaces, captures several fresh snapshots, confirms that the backend exposes
no mutating tools, and saves a JSON receipt below the artifact root:

```bash
python scripts/probe_piper_readonly.py --backend-options "$RAX_BACKEND_OPTIONS"
```

The next bring-up gate cross-checks the reported joint pose against a separate joint
display. The first command prints a compact degree table. Fill a private copy of
`examples/piper_pose_confirmation.json`, then run the confirmed check:

```bash
python scripts/verify_piper_pose.py --backend-options "$RAX_BACKEND_OPTIONS"
python scripts/verify_piper_pose.py --backend-options "$RAX_BACKEND_OPTIONS" \
  --confirmation /path/to/operator-pose-confirmation.json
```

When no independent display exists, record that limitation without claiming a pass:

```bash
python scripts/verify_piper_pose.py --backend-options "$RAX_BACKEND_OPTIONS" \
  --skip-independent-confirmation "No independent pendant or angle display is available."
```

This optional cross-check does not block bring-up when the required passive probe has
passed.

Read `docs/PIPER_STOP_AND_HOLD.md` before adding motion. Piper software emergency stop,
motor disable, reset, and position hold have materially different behavior.

Record the operator-owned emergency path and manual recovery agreement without sending
any CAN frame:

```bash
python scripts/verify_piper_stop_plan.py
python scripts/verify_piper_stop_plan.py \
  --confirmation /path/to/piper-stop-confirmation.json
```

Use a private copy of `examples/piper_stop_confirmation.json`; do not commit a receipt
containing site or operator details. A passed receipt confirms the static protocol but
does not authorize motion or claim that position hold has been characterized.

The position-hold latch has hardware-free coverage through injected SDK-shaped
interfaces in `local_tests/unit/test_piper_hold.py`. The operator-only minimum-motion
runner is documented in `docs/PIPER_MINIMUM_MOTION_TEST.md`. It uses a reduced-
protection last-target software hold because this installation has no confirmed
physical emergency stop. A passed receipt can admit only the same arm, joint,
direction, delta, and speed to the optional Agent tool; see
[docs/PIPER_AGENT_MOTION.md](docs/PIPER_AGENT_MOTION.md).

## Adding tasks

Copy `tasks/_template/` to `tasks/<task_id>/`. Keep robot-independent success criteria
in `evaluator.py`, task wording in `TASK.md`, and human reset instructions in
`RESET.md`. Task code must use the stable runtime API and must not import
`services.piper` or `piper_sdk`.

## Motion gate

Motion remains opt-in per session. The backend exposes one mutating MCP tool only when
operator configuration names passed characterization receipts and an explicit
arm/joint/direction allowlist. Runtime checks require a time-bounded operator arming
lease, fresh stable feedback, mechanical limits, per-call caps, and per-lease command
and cumulative-travel budgets. Every call ends in last-target software hold and writes
a private receipt. This is supervised bring-up capability, not safety-rated or
task-level manipulation control.

The first full model-driven commissioning task is
`tasks/piper_agent_joint_step/`. `scripts/run_piper_agent_test.py` validates its exact
task text and one-command motion envelope, starts the standard RAX Agent, audits the
model's tool choice, and always disarms the session when the launcher exits. See
[docs/PIPER_AGENT_MOTION.md](docs/PIPER_AGENT_MOTION.md#full-rax-agent-commissioning)
for the operator-present workflow. This verifies Agent tool selection; it does not
expand the backend into general manipulation control.

## License

AutoRAX retains the Apache-2.0 license and attribution of the upstream RAX project.
