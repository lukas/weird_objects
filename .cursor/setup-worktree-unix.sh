#!/bin/sh
# Cursor runs this in every NEW worktree it creates (wired up via
# .cursor/worktrees.json). The repo venv, RL checkpoints, and .mcp.json are
# machine-local (gitignored), so a fresh worktree is missing them; wire the
# worktree to the main checkout's copies instead of rebuilding anything.
set -eu
ROOT="${ROOT_WORKTREE_PATH:?ROOT_WORKTREE_PATH not set (pass the main checkout path, e.g. /Users/lukas/weird_objects)}"

# direnv env: activate the MAIN checkout's .venv. Never create a per-worktree
# venv — the repo rule is ONE uv-managed venv at $ROOT/.venv.
cat > .envrc <<EOF
export VIRTUAL_ENV=$ROOT/.venv
PATH_add $ROOT/.venv/bin
EOF
if command -v direnv >/dev/null 2>&1; then
  direnv allow . || true
fi

# Share the pulled RL checkpoints (gigabytes, pulled from CoreWeave, not in
# git). A symlink keeps one cache for all worktrees.
POL=hexapod_walker/prototype_sts3215/rl_move/sim/policies
if [ -d "$ROOT/$POL" ] && [ ! -e "$POL" ]; then
  ln -s "$ROOT/$POL" "$POL"
fi

# MCP config is gitignored; copy it so worktree agents get the same servers.
if [ -f "$ROOT/.mcp.json" ] && [ ! -e .mcp.json ]; then
  cp "$ROOT/.mcp.json" .mcp.json
fi
