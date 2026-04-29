#!/usr/bin/env bash
# Run pytest with the shared .venv.  Tests live next to the modules they
# exercise (currently just test_polyhedra.py at the repo root).
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

export PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"

echo "Running tests..."
pytest "$@"
