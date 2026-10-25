# Agent harnesses

AutoRAX currently supports two host-native agent harnesses:

| `agent_cli` | Harness | Providers |
| --- | --- | --- |
| `claude` | Claude Code | `claude-login`, `anthropic`, `openrouter` |
| `dsh` | DeepSeek Harness | `deepseek-api`, `openrouter`, `custom` |

The harness owns the model conversation and tool loop. The robot frontend, Piper
backend, motion admission, operator arming, audit log, and task packages do not change
with the selected harness. Selecting `dsh` never enables robot motion by itself.

## DeepSeek installation

DeepSeek Harness is pinned to `0.2.1-rc.2` and requires Node.js 24 or newer. Keep the
Node and npm installation, DSH package, npm cache, and API key outside tracked source
files. `runtime/` is ignored and may hold the local DSH package:

```bash
cd /home/edemlab/autorax_ws/autorax

NODE=/absolute/path/to/node-v24/bin/node
NPM=/absolute/path/to/node-v24/bin/npm
PREFIX="$PWD/runtime/agent-backends/deepseek-harness/tools/dsh-0.2.1-rc.2"

.venv/bin/python scripts/install_dsh.py \
  --node "$NODE" --npm "$NPM" \
  --prefix "$PREFIX" \
  --cache /home/edemlab/autorax_ws/runtime/cache/npm

export RAX_DSH_BINARY="$PREFIX/node_modules/.bin/dsh"
```

The installer records the exact DSH version, Node path, and package-lock digest in
`rax-installation.json`. The launcher refuses an unpinned installation or Node older
than 24.

## Credentials

Put only a key-file path in the environment. Never put the key value in `.env`, a
session JSON file, command-line argument, or Git:

```bash
mkdir -p ~/.config/rax
chmod 700 ~/.config/rax
install -m 600 /dev/null ~/.config/rax/deepseek.key
# Enter the DeepSeek API key into ~/.config/rax/deepseek.key.
export RAX_AGENT_KEY_FILE="$HOME/.config/rax/deepseek.key"
```

The launcher validates that the key is a regular owner-only file. Its path is stored
in private operator session state; its contents are passed only to the trusted DSH
process. DSH shell commands receive a filtered environment without `RAX_MODEL_KEY`.

DeepSeek account login is intentionally not exposed by the Piper branch yet. Use
`deepseek-api`, `openrouter`, or an operator-controlled `custom` endpoint.

## Configure and validate

Create a read-only mock session first:

```bash
export RAX_ARTIFACT_ROOT=/home/edemlab/autorax_ws/artifacts

CONFIG=$(.venv/bin/python scripts/configure_session.py \
  --robot mock \
  --task dsh_smoke \
  --task-text "Inspect status and do not request motion." \
  --agent-cli dsh \
  --agent-provider deepseek-api \
  --subagents 0)

bash scripts/start_agent.sh \
  --config "$CONFIG" \
  --agent-key-file "$RAX_AGENT_KEY_FILE" \
  --prepare-only
```

`--prepare-only` validates the pinned binary, Node version, credential file, generated
workspace, DSH profile, sandbox boundary, and MCP configuration. It starts neither the
model nor the robot service. A prepared-only session is disposable; create a fresh
configuration for a live run.

For a read-only Piper session, replace `--robot mock` with the normal Piper backend
configuration. Keep `agent_motion.enabled` false until the existing Piper bring-up and
commissioning gates have passed.

## Isolation boundary

- The DSH process runs on the trusted host and is the only process that receives the
  model credential.
- Model-selected shell commands run through bubblewrap with no network namespace and
  only the agent workspace writable.
- DSH file tools are restricted to the workspace, session stream, and system runtime
  paths.
- Robot tools are available only through the private session MCP socket.
- Piper motion remains controlled by backend receipts, allowlists, arming leases, and
  per-session budgets, independent of the harness.
- `scripts/run_piper_agent_test.py` starts DSH commissioning with shell and background
  job tools disabled; general non-commissioning sessions keep the bubblewrap-confined
  shell.

Codex CLI and Kimi Code are available in current upstream RAX simulation branches but
have not yet been ported or validated in AutoRAX `real/piper`.
