#!/usr/bin/env bash
# Unified script runner for the repo:
#   1. Manages a single shared .venv at the repo root (so every project shares
#      one set of installed wheels).
#   2. Resolves the target script either as an absolute path, a path relative
#      to the repo root (e.g. `chandelier/all_polyhedra.py`), or a bare script
#      name (which is searched for under the repo).
#   3. cds into the script's own directory before running it, so relative
#      paths used by the script (writing STLs, reading helper files) resolve
#      next to the script — and exposes the repo root on PYTHONPATH so
#      shared modules like `polyhedra.py` import cleanly from any subfolder.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

VENV_DIR=".venv"
PYTHON="${PYTHON:-python3}"

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment in $VENV_DIR..."
    "$PYTHON" -m venv "$VENV_DIR"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

REQ_FILE="requirements.txt"
STAMP_FILE="$VENV_DIR/.requirements.sha"
CURRENT_HASH="$(shasum "$REQ_FILE" | awk '{print $1}')"

if [ ! -f "$STAMP_FILE" ] || [ "$(cat "$STAMP_FILE")" != "$CURRENT_HASH" ]; then
    echo "Installing/updating dependencies..."
    pip install --upgrade pip
    pip install -r "$REQ_FILE"
    echo "$CURRENT_HASH" > "$STAMP_FILE"
fi

TARGET="${1:-chandelier/all_polyhedra.py}"
if [ "$#" -gt 0 ]; then
    shift
fi

# Resolve target.
if [ -f "$TARGET" ]; then
    TARGET_ABS="$(cd "$(dirname "$TARGET")" && pwd)/$(basename "$TARGET")"
elif [ -f "$ROOT_DIR/$TARGET" ]; then
    TARGET_ABS="$ROOT_DIR/$TARGET"
else
    # bare name → search under the repo (skip .venv etc.)
    HIT="$(find "$ROOT_DIR" -name "$(basename "$TARGET")" -type f \
                  -not -path "*/.venv/*" -not -path "*/.git/*" \
                  -not -path "*/__pycache__/*" 2>/dev/null | head -1)"
    if [ -z "$HIT" ]; then
        echo "Could not find script: $TARGET" >&2
        exit 1
    fi
    TARGET_ABS="$HIT"
fi

TARGET_DIR="$(dirname "$TARGET_ABS")"
TARGET_NAME="$(basename "$TARGET_ABS")"
export PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"

cd "$TARGET_DIR"
# Display path relative to repo root for nicer logs.
REL_DISPLAY="${TARGET_ABS#$ROOT_DIR/}"
echo "Running $REL_DISPLAY $*..."
python "$TARGET_NAME" "$@"
