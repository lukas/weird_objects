"""Benchmark: C-MuJoCo per-env stepping vs batched MJX.

This is the go/no-go tool for the MJX port: run it on a CoreWeave GPU
pod and compare against the C baseline (one core; SubprocVecEnv scales
that by ~n_cores). Both sides run the same workload as
``SimHexapodBalanceEnv._advance``: servo-profile tick + firmware
dead-zone + physics step + IMU accumulation, 16 substeps per 25 Hz
control tick, robot in contact with the ground at the plant pose.

Usage (from prototype_sts3215/):
    python -m rl_move.sim.bench_mjx                    # defaults
    python -m rl_move.sim.bench_mjx --batch 512 4096   # GPU sizing
    python -m rl_move.sim.bench_mjx --ticks 100

Interpretation: "env-steps/s" is control ticks x batch per second —
compare directly against SB3 fps (which is control ticks/s across all
envs). The C baseline number x 48 approximates today's 48-env subproc
throughput on a 48-core pod.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_PROTO = Path(__file__).resolve().parents[2]
if str(_PROTO) not in sys.path:
    sys.path.insert(0, str(_PROTO))

from rl_move.sim.mjx_backend import (  # noqa: E402
    MjxTickStepper, mjx_is_available, tune_for_mjx,
)
from rl_move.sim.servo_model import (  # noqa: E402
    ServoProfile, SimServoParams, apply_params_to_model, build_model,
    joint_qpos_addrs, position_actuator_ids,
)

DT_CTRL = 0.04
SPEED_DEG_S = 400 * 360.0 / 4096.0
ACC_UNITS = 20.0


def _plant_state(model):
    """Drop the robot at a lifted zero pose and settle briefly so both
    benchmarks run a realistic contact-rich state."""
    import mujoco
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.08
    mujoco.mj_forward(model, data)
    for _ in range(int(0.6 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    data.qvel[:] = 0.0
    return data


def bench_c(model, data0, ticks: int) -> float:
    """One env, C MuJoCo, replicating _advance (incl. IMU readout)."""
    import mujoco
    params = SimServoParams.load()
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    cb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    data = mujoco.MjData(model)
    data.qpos[:] = data0.qpos
    mujoco.mj_forward(model, data)
    q0 = data.qpos[qadr].copy()
    prof = ServoProfile(params, q0)
    substeps = int(round(DT_CTRL / model.opt.timestep))
    h = model.opt.timestep
    vel = np.zeros(6)
    prev_v = None
    t0 = time.perf_counter()
    for k in range(ticks):
        prof.command(q0, speed_deg_s=SPEED_DEG_S, acc_units=ACC_UNITS)
        for _ in range(substeps):
            target = prof.tick(h)
            q = data.qpos[qadr]
            err = target - q
            eff = q + np.sign(err) * np.maximum(
                np.abs(err) - prof.deadband_rad, 0.0)
            data.ctrl[pos_act] = eff
            mujoco.mj_step(model, data)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                                     cb, vel, 0)
            v_pt = vel[3:].copy()
            if prev_v is not None:
                _ = (v_pt - prev_v) / h - model.opt.gravity
            prev_v = v_pt
    return time.perf_counter() - t0


def bench_mjx(model, data0, batch: int, ticks: int,
              impl: str | None = None) -> tuple[float, float]:
    """Returns (compile_s, run_s) for `ticks` ticks at batch size."""
    params = SimServoParams.load()
    st = MjxTickStepper(model, batch, params=params, impl=impl)
    qadr = joint_qpos_addrs(model)
    q0 = data0.qpos[qadr].copy()
    st.reset_envs(np.tile(data0.qpos, (batch, 1)),
                  np.zeros((batch, model.nv)),
                  np.tile(q0, (batch, 1)), dt_ctrl=DT_CTRL)
    cmd = st.make_command(np.tile(q0, (batch, 1)),
                          speed_deg_s=SPEED_DEG_S, acc_units=ACC_UNITS)
    t0 = time.perf_counter()
    out = st.tick(cmd)
    np.asarray(out.q)                       # block on compile + first tick
    compile_s = time.perf_counter() - t0
    t0 = time.perf_counter()
    for _ in range(ticks):
        out = st.tick(cmd)
    np.asarray(out.q)                       # block
    return compile_s, time.perf_counter() - t0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ticks", type=int, default=50,
                    help="control ticks per measurement (x16 phys steps)")
    ap.add_argument("--batch", type=int, nargs="+",
                    default=[1, 32, 256, 1024])
    ap.add_argument("--skip-c", action="store_true")
    ap.add_argument("--mjx-iterations", type=int, default=8,
                    help="solver iterations for the MJX copy "
                         "(0 = keep the XML's 50; C baseline always "
                         "runs the production model untouched)")
    ap.add_argument("--mjx-ls-iterations", type=int, default=8)
    ap.add_argument("--impl", default=None, choices=["jax", "warp"],
                    help="mjx backend implementation (warp needs "
                         "mujoco-warp installed; much faster on GPU)")
    args = ap.parse_args()

    # C baseline: the exact production model (hfield, iterations=50).
    model = build_model(fixed_base=False, flat_terrain=True,
                        mesh_visuals=False)
    apply_params_to_model(model, SimServoParams.load())
    data0 = _plant_state(model)
    substeps = int(round(DT_CTRL / model.opt.timestep))

    c_env_steps = None
    if not args.skip_c:
        dt = bench_c(model, data0, args.ticks)
        c_env_steps = args.ticks / dt
        print(f"C MuJoCo (1 env, 1 core): {c_env_steps:8.1f} env-steps/s "
              f"({c_env_steps * substeps:9.0f} phys steps/s)")
        print(f"  x48 subproc estimate:   {c_env_steps * 48:8.1f} env-steps/s")

    if not mjx_is_available():
        print("mujoco-mjx / jax not installed — MJX side skipped "
              "(pip install -r rl_move/sim/requirements-mjx.txt)")
        return
    import jax
    print(f"JAX devices: {jax.devices()}")
    # MJX copy: plane terrain, single ground plane, tuned solver.
    model_mjx = build_model(fixed_base=False, flat_terrain=True,
                            mesh_visuals=False, mjx_compat=True)
    apply_params_to_model(model_mjx, SimServoParams.load())
    if args.mjx_iterations > 0:
        tune_for_mjx(model_mjx, iterations=args.mjx_iterations,
                     ls_iterations=args.mjx_ls_iterations)
        print(f"MJX solver: iterations={model_mjx.opt.iterations} "
              f"ls_iterations={model_mjx.opt.ls_iterations} "
              f"impl={args.impl or 'default'} "
              f"(C baseline keeps {model.opt.iterations})")
    for b in args.batch:
        comp, run = bench_mjx(model_mjx, data0, b, args.ticks,
                              impl=args.impl)
        eps = args.ticks * b / run
        line = (f"MJX batch {b:5d}: {eps:8.1f} env-steps/s "
                f"({eps * substeps:9.0f} phys steps/s, "
                f"compile {comp:.1f}s)")
        if c_env_steps:
            line += f"  [{eps / c_env_steps:6.1f}x one C core]"
        print(line)


if __name__ == "__main__":
    main()
