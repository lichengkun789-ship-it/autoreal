# Piper Agent motion

`piper_move_joint` is AutoRAX's first Piper Agent motion primitive. It is deliberately
narrow: one relative joint step on one arm, followed by a last-target software hold.
The default `examples/piper.json` keeps it disabled.

## Admission boundary

Enabling the tool requires all of the following in operator-owned backend options:

- an explicit arm, joint, and direction allowlist;
- one passed, executed `piper_minimum_motion_characterization` receipt covering every
  allowlisted direction at no smaller delta and no lower speed;
- limits no larger than `0.005 rad`, `5%` speed, `0.5 s` command window, 10 commands,
  and `0.05 rad` cumulative requested travel per arming lease;
- a live operator arming lease when the tool is called.

The receipt CAN names must match the configured robot. A positive characterization
does not cover the negative direction, another joint, or the other arm.

Example operator configuration after characterizing right J1 positive:

```json
{
  "arms": {
    "left": {"can_name": "piper_sl"},
    "right": {"can_name": "piper_sr"}
  },
  "feedback_max_age_s": 0.25,
  "joint_limit_margin_rad": 0.02,
  "max_joint_step_rad": 0.05,
  "agent_motion": {
    "enabled": true,
    "allowed_moves": [
      {"arm": "right", "joint": 1, "directions": ["positive"]}
    ],
    "max_delta_rad": 0.005,
    "speed_percent": 3,
    "command_window_s": 0.5,
    "max_commands_per_arming": 1,
    "max_cumulative_delta_rad": 0.005,
    "characterization_receipts": [
      "/absolute/path/to/piper_minimum_motion_receipt.json"
    ]
  },
  "protected_paths": []
}
```

Start with one command per lease. Widen a budget only after inspecting the generated
Agent receipt and performing a separate supervised test.

## Runtime sequence

1. The operator creates a fresh session using the motion-enabled backend options.
2. The backend validates every evidence file before opening the Agent tool surface.
3. The operator starts the session, clears the workspace, remains at the keyboard,
   and runs `python scripts/arm_robot.py SESSION arm --minutes 5`.
4. Before dispatch, the backend checks the allowlist, arming lease, command budget,
   stable dual-arm baseline, fresh feedback, controller errors, motor-enabled feedback,
   mechanical limits, and relative step cap.
5. The selected arm receives one joint-position command. During the command window,
   the backend polls the arming lease. Revocation causes the exact accepted target to
   be re-sent as hold.
6. The backend samples post-hold motion, writes a private receipt under
   `SESSION/piper-motion/`, and returns measured travel and remaining budgets.

The Agent never receives enable, reset, home, disable, emergency-stop, arbitrary
six-joint target, gripper, or automatic-return tools.

## Operator commands

```bash
export RAX_BACKEND_OPTIONS=/absolute/path/to/piper-agent-motion.json

python scripts/configure_session.py \
  --task bounded_piper_bringup \
  --task-text "Make one admitted Piper joint step and report measured travel."

bash scripts/start_agent.sh --config /path/to/session/operator-input.json

# In another operator terminal, after the service is running:
python scripts/arm_robot.py /path/to/session arm --minutes 5

# Revoke authorization at any time. A running guarded command observes this and
# requests last-target software hold; this is not a physical emergency stop.
python scripts/arm_robot.py /path/to/session disarm
```

## Full RAX Agent commissioning

Use the dedicated commissioning task when validating that the model itself reads the
live state and selects the bounded tool. Do not use an open-ended manipulation prompt
for the first model-driven motion.

The full launcher requires the `claude` executable plus one configured provider. For
the default dedicated Claude account, run `scripts/setup_claude_experiment_login.sh`;
Anthropic and OpenRouter sessions can instead use a private key file passed to the
commissioning runner.

Create a fresh session from the repository-owned task text. The motion backend options
must retain the one-command right-J1-positive allowlist characterized earlier:

```bash
source .env

CONFIG=$(.venv/bin/python scripts/configure_session.py \
  --task piper_agent_joint_step \
  --task-file tasks/piper_agent_joint_step/TASK.md \
  --backend-options /absolute/path/to/piper_agent_motion.json \
  --observation-profile state-only \
  --agent-provider claude-login \
  --session-hours 0.25 \
  --max-model-calls 20 \
  --subagents 0)

# Model/configuration preflight only. This does not start a service or arm the robot.
.venv/bin/python scripts/run_piper_agent_test.py --config "$CONFIG"

# Operator-present physical test. The runner asks for the exact confirmation phrase.
.venv/bin/python scripts/run_piper_agent_test.py --config "$CONFIG" --execute
```

For an Anthropic or OpenRouter configuration, add `--key-file /private/key-file` to
the runner command. The file must be owned by the current user and inaccessible to
group and other users.

The runner validates the task text and movement envelope before arming. It runs the
normal RAX model relay, isolated Agent workspace, MCP frontend, and Piper backend. A
run passes only when the private MCP audit contains exactly one matching
`piper_move_joint` request, exactly one passed motion receipt confirms last-target
hold, the model process exits successfully, and the session is disarmed. Normal exit,
model failure, and `Ctrl+C` all execute the disarm path; the short arming lease remains
the fallback for an uncatchable host-process failure.

For DeepSeek Harness commissioning, the runner also removes Bash, PowerShell, and
background-job tools from the model surface. General research sessions retain their
sandboxed shell, but the first physical motion test needs only file reads and MCP robot
tools.

The summary is written to `<session>/piper-agent-commissioning.json`; detailed robot
evidence remains under `<session>/piper-motion/`.

## Current limitation

The tool makes RAX capable of a characterized joint step, not a complete manipulation
task. General task execution still needs separately characterized axes, gripper
support, camera observations, task-level success detection, and a physical stop path
appropriate to unattended operation.
