#!/usr/bin/env bash
# Global robot-deploy lock + log — shared across ALL workspaces/worktrees.
#
# Multiple Cursor workspaces deploy to the same robot; two deploys
# interleaving leaves a mixed tree on the board (and a flash mid-deploy
# is worse).  This serializes them through the HOME directory, which
# every worktree shares:
#
#   ~/.hexapod/deploy.lock/   the lock (atomic mkdir); owner/pid inside
#   ~/.hexapod/deploy.log     append-only history of who deployed what
#
# Usage (see deploy_ssh.sh / deploy_adb.sh / flash_feetech_bridge.sh):
#
#   source "$SRC/deploy_lock.sh"
#   deploy_lock_acquire "ssh push"     # blocks up to $DEPLOY_LOCK_WAIT_S
#   deploy_log "note"                  # optional extra log lines
#   ... deploy ...                     # lock auto-releases on EXIT
#
# Tunables (env): DEPLOY_LOCK_WAIT_S (default 900; 0 = fail fast),
# HEXAPOD_SYNC_DIR (default ~/.hexapod).
#
# Peek at the state from any workspace:
#   cat ~/.hexapod/deploy.lock/owner 2>/dev/null || echo "lock free"
#   tail ~/.hexapod/deploy.log

HEXAPOD_SYNC_DIR="${HEXAPOD_SYNC_DIR:-$HOME/.hexapod}"
DEPLOY_LOCK_DIR="$HEXAPOD_SYNC_DIR/deploy.lock"
DEPLOY_LOG_FILE="$HEXAPOD_SYNC_DIR/deploy.log"
DEPLOY_LOCK_WAIT_S="${DEPLOY_LOCK_WAIT_S:-900}"

_deploy_ctx() {
  # "workspace-dir branch@sha" of the repo the CALLING script lives in.
  local dir branch sha
  dir="$(git -C "${_DEPLOY_CALLER_DIR:-$PWD}" rev-parse --show-toplevel \
         2>/dev/null || echo "$PWD")"
  branch="$(git -C "$dir" rev-parse --abbrev-ref HEAD 2>/dev/null \
            || echo '?')"
  sha="$(git -C "$dir" rev-parse --short HEAD 2>/dev/null || echo '?')"
  printf '%s %s@%s' "$dir" "$branch" "$sha"
}

deploy_log() {
  mkdir -p "$HEXAPOD_SYNC_DIR"
  printf '%s pid=%s %s :: %s\n' \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$$" "$(_deploy_ctx)" "$*" \
    >> "$DEPLOY_LOG_FILE"
}

_deploy_lock_mtime_age_s() {
  local m
  m="$(stat -f %m "$DEPLOY_LOCK_DIR" 2>/dev/null \
       || stat -c %Y "$DEPLOY_LOCK_DIR" 2>/dev/null || echo 0)"
  echo $(( $(date +%s) - m ))
}

deploy_lock_release() {
  local rc=$?
  if [ "$(cat "$DEPLOY_LOCK_DIR/pid" 2>/dev/null)" = "$$" ]; then
    deploy_log "UNLOCK rc=$rc (${_DEPLOY_LOCK_LABEL:-deploy})"
    rm -rf "$DEPLOY_LOCK_DIR"
  fi
}

deploy_lock_acquire() {
  _DEPLOY_LOCK_LABEL="${1:-deploy}"
  _DEPLOY_CALLER_DIR="$(cd "$(dirname "${BASH_SOURCE[1]:-.}")" && pwd)"
  local waited=0 owner opid
  mkdir -p "$HEXAPOD_SYNC_DIR"
  while ! mkdir "$DEPLOY_LOCK_DIR" 2>/dev/null; do
    opid="$(cat "$DEPLOY_LOCK_DIR/pid" 2>/dev/null || true)"
    owner="$(cat "$DEPLOY_LOCK_DIR/owner" 2>/dev/null || echo 'unknown')"
    if [ -n "$opid" ] && ! kill -0 "$opid" 2>/dev/null; then
      deploy_log "STEAL stale lock (dead pid $opid: $owner)"
      echo ">> stealing stale deploy lock (dead pid $opid)" >&2
      rm -rf "$DEPLOY_LOCK_DIR"
      continue
    fi
    if [ -z "$opid" ] && [ "$(_deploy_lock_mtime_age_s)" -gt 120 ]; then
      # Lock dir exists but the owner never wrote its pid — a crashed
      # acquire.  Old enough that it can't be a mid-write race.
      deploy_log "STEAL abandoned lock (no pid, $( _deploy_lock_mtime_age_s )s old)"
      rm -rf "$DEPLOY_LOCK_DIR"
      continue
    fi
    if [ "$waited" -ge "$DEPLOY_LOCK_WAIT_S" ]; then
      echo "!! deploy lock still held after ${waited}s by: $owner" >&2
      echo "   (rm -rf $DEPLOY_LOCK_DIR to force; log: $DEPLOY_LOG_FILE)" >&2
      deploy_log "TIMEOUT after ${waited}s waiting on: $owner"
      return 1
    fi
    if [ $(( waited % 15 )) -eq 0 ]; then
      echo ">> deploy lock held by: $owner — waiting" \
           "($waited/${DEPLOY_LOCK_WAIT_S}s)" >&2
    fi
    sleep 3
    waited=$(( waited + 3 ))
  done
  echo "$$" > "$DEPLOY_LOCK_DIR/pid"
  printf 'pid=%s %s (%s) since %s\n' "$$" "$(_deploy_ctx)" \
    "$_DEPLOY_LOCK_LABEL" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
    > "$DEPLOY_LOCK_DIR/owner"
  trap deploy_lock_release EXIT
  deploy_log "LOCK ($_DEPLOY_LOCK_LABEL)"
}
