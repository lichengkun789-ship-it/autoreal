#!/usr/bin/env bash
set -euo pipefail

PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT"

if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi

export RAX_ARTIFACT_ROOT="${RAX_ARTIFACT_ROOT:-/home/edemlab/autorax_ws/artifacts}"
BACKEND_OPTIONS="${RAX_CLICK_BELL_BACKEND_OPTIONS:-$RAX_ARTIFACT_ROOT/config/piper_click_bell_right_workspace_candidate.json}"
AGENT_MODE="${RAX_AGENT_MODE:-research}"

for path in "$BACKEND_OPTIONS"; do
  if [[ ! -f "$path" ]]; then
    printf 'Required ClickBell configuration not found: %s\n' "$path" >&2
    exit 2
  fi
done

CONFIG="$(.venv/bin/python scripts/configure_session.py \
  --robot piper \
  --task click_bell \
  --task-file tasks/click_bell/TASK.md \
  --backend-options "$BACKEND_OPTIONS" \
  --observation-profile rgbd \
  --agent-cli dsh \
  --agent-provider deepseek-api \
  --agent-mode "$AGENT_MODE" \
  --agent-effort xhigh \
  --session-hours 1 \
  --max-model-calls 80 \
  --subagents 0)"

printf 'ClickBell session: %s\n' "$CONFIG"
exec .venv/bin/python scripts/run_task_experiment.py \
  --config "$CONFIG" \
  --arming-minutes 10 \
  --execute \
  "$@"
