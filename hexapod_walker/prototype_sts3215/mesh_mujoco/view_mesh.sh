#!/usr/bin/env bash
# Launch the interactive viewer (macOS: MuJoCo's viewer requires mjpython).
# Usage: ./view_mesh.sh [plant|stance]
set -euo pipefail
cd "$(dirname "$0")"
VENV="${VENV:-/Users/lukas/weird_objects/.venv}"
[ -f hexapod_mesh.xml ] || "$VENV/bin/python" build_mesh_model.py --no-render
exec "$VENV/bin/mjpython" view_mesh.py "${1:-plant}"
