# AutoRAX

AutoRAX adapts [RAX](https://github.com/robot-agent-exploration/rax) to a dual-arm
Piper robot while keeping the agent harness, hardware backend, and task definitions
independent. The repository is intended to support multiple manipulation tasks; no
task is built into the task-neutral Piper backend. Tasks provide descriptions,
perception strategies, evaluators, and reusable Agent skills, while robot motion stays
in generic backend tools.

The default backend remains intentionally **read-only**. RAX can connect to both Piper
CAN interfaces, inspect measured state, reject stale or faulted feedback, dry-run
absolute joint targets, and optionally capture configured RealSense RGB or RGB-D
observations. An operator may opt a session into the evidence-bound `piper_move_joint`
tool after a matching minimum-motion characterization. A separately configured
`cartesian_motion` capability turns passed `piper_plan_pose` results into one-shot
`piper_execute_plan` and `piper_retreat_plan` transactions with arming, live-feedback,
workspace, link-height, joint-limit, hold, recovery, and receipt checks. Execution and
retreat may be split into caller-selected waypoint segments, returning control to the
Agent between segments. `piper_hold` latches measured state, and `piper_gripper`
provides commissioned task-independent open/close/move actions. The configured Piper
tool point comes from robot geometry rather than task code. A complete session records
one continuous three-camera episode plus robot telemetry, and ClickBell success still
requires human visual confirmation. Homing, enable, reset, impedance, and general
force control remain unavailable to the Agent.

## Repository layout

| Path | Responsibility |
| --- | --- |
| `services/controller/` | Robot-neutral MCP dispatch, sessions, budgets, and audit logs |
| `services/piper/` | Task-neutral Piper state, geometry, motion, gripper, recording, and receipts |
| `services/perception/` | Robot-independent camera geometry used by backends and task perception |
| `harness/` | Agent selection plus isolated Claude Code and DeepSeek Harness adapters |
| `agent_skills/` | Capability-specific instructions installed into an agent workspace |
| `auto_research_agent/` | Generated workspace template and stable Python runtime API |
| `tasks/` | Task packages: goal, reset procedure, evaluator, and tests |
| external `skill-library/<robot>/` | Git-backed executable skills shared across sessions |
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

### Research and execution

Every configuration declares `--agent-mode research` or `--agent-mode execution`.
Research sessions can write and run sandboxed Python, and receive a branch of the
persistent robot skill library. Execution sessions mount the workspace read-only and
check out a detached skill-library commit so evaluation cannot patch the procedure.
See [docs/AUTORESEARCH_WORKFLOW.md](docs/AUTORESEARCH_WORKFLOW.md).

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

With the example's default `agent_motion.enabled: false` and
`cartesian_motion.enabled: false`, the base Piper tools are read-only. Enable the two
motion capabilities independently; generic Cartesian execution additionally requires
configured workspace bounds and a distal-link minimum height for every allowed arm.
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
`agent-workspace/runtime/frames/` directory. `examples/piper_rgbd_calibrated.json`
shows how operator-supplied fixed or eye-in-hand extrinsics enable the read-only
`piper_pixel_to_position` tool. Fixed-camera and hand-eye calibration, forward
kinematics, and read-only numerical IK are implemented. Their output is planning
evidence only; it does not authorize a new motion path. See
[docs/PIPER_CAMERAS.md](docs/PIPER_CAMERAS.md). The first task-owned detector is
documented in [docs/CLICK_BELL.md](docs/CLICK_BELL.md).

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
`RESET.md`. Perception should use robot-independent helpers under `services/perception/`.
Add a task-specific robot adapter only when the hardware behavior cannot be expressed
by existing generic robot tools. `services/task_tools.py` loads such adapters from a
manifest without task-name branches, but ClickBell intentionally uses none. Task
packages must not import robot backends, arming code, or hardware limits.

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

ClickBell is now a task Skill composed from `piper_observe`,
`piper_pixel_to_position`, `piper_plan_pose`, segmented `piper_execute_plan`,
`piper_retreat_plan`, `piper_hold`, and `piper_gripper`. The Agent chooses when to
continue, how many waypoints to retreat, and whether a failed strategy should be
replanned or revised into a research-mode controller. The backend records MP4 video,
aligned depth, stills, and joint/effort/speed/gripper telemetry for the complete
session. `request_operator(kind="visual_confirmation")` remains the success signal;
motion or contact effort alone is never proof of success. See
[docs/CLICK_BELL.md](docs/CLICK_BELL.md).

Run the canonical operator-present experiment with:

```bash
bash scripts/run_click_bell_experiment.sh
```

The wrapper creates a normal AutoRAX session and starts the selected Harness. DeepSeek
receives the task description and installed ClickBell Skill, then chooses among the MCP
tools; the wrapper does not directly script robot actions.

## License

AutoRAX retains the Apache-2.0 license and attribution of the upstream RAX project.
