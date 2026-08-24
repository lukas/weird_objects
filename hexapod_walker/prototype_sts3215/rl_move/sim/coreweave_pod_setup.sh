#!/bin/bash
# One-time setup inside a hexapod training pod (python:3.11-slim):
# system GL lib + pinned Python deps matching the laptop venv, then
# unpack the code tarball copied to /tmp/code.tgz.
set -euo pipefail

apt-get update -qq
# libosmesa6: headless software GL for MuJoCo rendering (MUJOCO_GL=osmesa).
apt-get install -y -qq libosmesa6 libglib2.0-0 > /dev/null

mkdir -p /workspace/prototype_sts3215
tar -xzf /tmp/code.tgz -C /workspace/prototype_sts3215

pip install --quiet --no-cache-dir uv
UV_PIP=(uv pip install --system --quiet --no-cache)

# Pins mirror the laptop venv (rebuilt 2026-08-07 on Python 3.14 /
# MuJoCo 3.11 — physics A/B against 2.3.7 in RL_PLAN). Keep both sides
# on the SAME mujoco version or local evals disagree with training.
# numpy is 2.4.6 here (2.5.x needs py>=3.12; image is python:3.11) —
# laptop runs 2.5.1, which is fine: numpy is glue, mujoco is physics.
"${UV_PIP[@]}" torch==2.13.0 --index-url https://download.pytorch.org/whl/cpu
"${UV_PIP[@]}" \
    mujoco==3.11.0 \
    stable_baselines3==2.9.0 \
    sb3-contrib==2.9.0 \
    gymnasium==1.3.0 \
    numpy==2.4.6 \
    scipy==1.17.1 \
    wandb==0.28.1 \
    pillow==11.3.0 \
    moviepy==2.2.1 \
    imageio==2.37.4 \
    imageio-ffmpeg==0.6.0 \
    tensorboard==2.21.0 \
    PyYAML==6.0.3 \
    trimesh==5.0.0 \
    shapely==2.1.2 \
    networkx==3.6.1 \
    rtree==1.4.1 \
    manifold3d==3.5.2

# Optional MJX (JAX) physics backend — opt-in (HEXAPOD_MJX=1), the
# default PPO path never imports it. See rl_move/sim/MJX_PORT.md.
# mujoco-mjx must match the mujoco pin above EXACTLY; jax[cuda12] ships
# its own CUDA runtime wheels, so the slim image works on GPU nodes.
if [ "${HEXAPOD_MJX:-0}" = "1" ]; then
    if command -v nvidia-smi > /dev/null 2>&1; then
        # mujoco-warp: the fast impl="warp" backend (~30x the XLA impl
        # on the hexapod — see MJX_PORT.md benchmark table).
        "${UV_PIP[@]}" mujoco-mjx==3.11.0 "jax[cuda12]" mujoco-warp
    else
        "${UV_PIP[@]}" mujoco-mjx==3.11.0 jax
    fi
    uv run python -c "from mujoco import mjx; import jax; print('mjx ok', jax.devices())"
fi

uv run python -c "import mujoco, stable_baselines3, sb3_contrib, torch; print('deps ok')"
echo "SETUP_DONE"
