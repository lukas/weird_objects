#!/usr/bin/env bash
# Combine the live RL training + orchestration docs into one big markdown
# file for an external model (GPT) to review.
#
# Usage:  ./make_rl_review_bundle.sh [output.md]
# Default output: RL_REVIEW_BUNDLE_<YYYY-MM-DD>.md next to this script.
#
# Per-run notes (rl_docs/runs/, 500+ files) are intentionally excluded;
# RL_LOG.md is the curated log of those runs and goes last.

set -euo pipefail
cd "$(dirname "$0")"

OUT="${1:-RL_REVIEW_BUNDLE_$(date +%F).md}"
TODAY="$(date +%F)"

# Reading order: goals/status first, then methodology, then orchestration,
# with the big run log at the end.
FILES=(
  README.md
  RL_GOALS.md
  CURRENT_TRUTHS.md
  STATUS.md
  RL_PLAN.md
  RESEARCH_RULES.md
  RUN_INTERPRETATION_RULES.md
  rl_docs/README.md
  rl_docs/GOAL.md
  rl_docs/AGENT.md
  rl_docs/SIM.md
  rl_docs/REWARD.md
  rl_docs/RISE.md
  rl_docs/TURN.md
  rl_docs/EVALS.md
  rl_docs/SKILLS.md
  rl_docs/COMMANDS.md
  rl_docs/EXPERIMENT_LOGS.md
  rl_docs/WANDB.md
  rl_docs/HARDWARE.md
  rl_docs/WISHLIST.md
  rl_move/API.md
  rl_move/RUNLOG.md
  rl_move/orchestrator/README.md
  rl_move/orchestrator/ORCHESTRATOR_PROMPT.md
  rl_move/orchestrator/CAPACITY.md
  RL_LOG.md
)

missing=0
for f in "${FILES[@]}"; do
  [ -f "$f" ] || { echo "MISSING: $f" >&2; missing=1; }
done
[ "$missing" -eq 0 ] || { echo "Aborting: fix the file list above." >&2; exit 1; }

{
  cat <<EOF
# Hexapod prototype_sts3215 — RL training & orchestration review bundle

Generated $TODAY by \`make_rl_review_bundle.sh\`, concatenating the
project's live docs so an external model can review the state of RL
training and orchestration. Each section below is one source file,
verbatim, with its repo path in the header.

Context for the reviewer:

- The robot is an 18-DOF hexapod using Feetech STS3215 servos, controlled by
  an Arduino Uno Q running a Linux-side control loop.
- Training runs on CoreWeave GPU pods (MuJoCo sim + PPO via stable-baselines3),
  driven by an autonomous orchestrator agent; results are pulled to a Mac for
  local sim viewing and to the robot for hardware trials.
- Per-run notes (500+ files under \`rl_docs/runs/\`) are NOT included here;
  \`RL_LOG.md\` (last section) is the curated log of those runs.

## Files included

EOF
  for f in "${FILES[@]}"; do
    echo "- \`$f\`"
  done
  for f in "${FILES[@]}"; do
    printf '\n\n---\n\n# FILE: %s\n\n' "$f"
    cat "$f"
  done
} > "$OUT"

echo "Wrote $OUT ($(wc -c < "$OUT" | tr -d ' ') bytes, ${#FILES[@]} docs)"
