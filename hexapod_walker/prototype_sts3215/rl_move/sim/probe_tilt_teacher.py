"""Closed-loop audit of the tip-aware HOLD anchor TEACHER
(cw-stand-tiltcomp1 dig-in, 2026-08-13).

The tiltcomp1 FAIL verdict's mechanism clause says: "two
differently-designed teachers (tilt-blind, tilt-aware) converged on
the same stay-tilted habit => the INCENTIVE is the problem (hold
income never prices residual lean)". Two facts cast doubt on that:

  (1) Hold income DOES price lean: reward.k_track is a product
      Gaussian including tilt vs the episode tilt reference at
      track_sigma_deg 1.5, and tipped-start episodes keep the
      reference LEVEL (sim_env._reset_finalize) - staying at ~6deg
      forfeits ~99.9% of tracking income (plus k_roll/k_pitch
      quadratics).
  (2) The tilt-aware teacher (train.bc_anchor_tilt_comp) is a pure
      PROPORTIONAL controller of the CURRENT measured lean:
      comp = clip(soft_deadband(lean, 1.5deg), cap 6deg). Closed
      loop, a P-controller has a steady-state error: if the start
      pose's uncorrected lean is L0 and the achieved lean under a
      tracked correction c is L = L0 - c, the fixed point of
      c = L - dead is L* = (L0 + dead) / 2 - e.g. ~4.25deg for the
      ~7deg tips the probe spawns (dr cap = 0.7 * max_roll), never
      the <=3deg gate bar. The teacher may be mathematically unable
      to demonstrate a leveling PASS regardless of the student.

Discriminating measurement, no training:

  ARM teacher-comp : tilt_comp=1.0 env; the ACTION each tick is the
                     anchor target itself (a perfect student).
  ARM teacher-blind: tilt_comp=0.0 env; action = the constant q_nom
                     target (control - expect the lean to persist).
  ARM policy       : (optional --ckpt) the trained policy in the
                     tilt_comp env, logging its action-vs-target MSE
                     and the target-vs-qnom MSE (anchor signal size -
                     the joint-space-dilution question).

Per tick: true gravity roll/pitch, rel-to-ref lean, reward, contacts,
height. Summary per episode: last-3s median |true roll|, mean reward,
median anchor MSEs, plus the P-controller fixed-point prediction.

READ:
  teacher-comp settles ~4-5deg  -> teacher defect CONFIRMED (P-control
     fixed point); the tiltcomp1 mechanism clause ("incentive gap") is
     unproven and the tipped-exposure closure premise falls.
  teacher-comp levels <=2deg    -> teacher fine; the policy genuinely
     defies supervision + income; the escalation stands as written.

Pure diagnostic - C env on CPU, no env-code changes. Run on a train
pod (COMMANDS.md gotcha 16: snapshot.sh --sync first).

Usage:
    uv run python -m rl_move.sim.probe_tilt_teacher \
        [--ckpt rl_move/sim/policies/ppo_goal_cw_stand_tiltcomp1.zip] \
        --episodes 6 --tip-deg 8 --out /tmp/tilt_teacher.json
"""
from __future__ import annotations

import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (_PROTO, _PROTO / "linux_control"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

RAD2DEG = 180.0 / np.pi
CONTACT_N = 0.5

# The run's hold-relevant stack (cw-stand-tiltcomp1 ledger entry),
# minus rise/lower keys the hold-only probe never exercises.
BASE_CFG = [
    "train.bc_anchor_coef=1.0",
    "reward.hold_still_gate=1.0",
    "reward.hold_flag_fade=1.0",
    "reward.k_support_margin=0.0",
    "train.bc_anchor_foot_z=1.0",
    "bus.servo_params=loaded",
    "dr.tipped_start_prob=1.0",
]


def _mk_env(tilt_comp: float, tip_deg: float, seed: int,
            episode_seconds: float, extra_cfg: list[str] | None):
    from rl_move.config import load_config
    from rl_move.sim.train_ppo_sim import ENV_CLASSES, _parse_cfg_set
    from rl_move.sim.servo_model import SimServoParams

    sets = list(BASE_CFG)
    sets.append(f"dr.tipped_start_deg={tip_deg},{tip_deg}")
    sets.append(f"train.bc_anchor_tilt_comp={tilt_comp}")
    if extra_cfg:
        sets += list(extra_cfg)
    cfg = load_config()
    for key, parsed in _parse_cfg_set(sets).items():
        sect, name = key.split(".", 1)
        cfg.setdefault(sect, {})[name] = parsed
    env = ENV_CLASSES["joint_goal"](
        cfg=cfg, params=SimServoParams.from_cfg(cfg), randomize=True,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=seed)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "walk", "quad", "getup"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "hold" else 0.0)
    return env


def _episode(env, act_fn, episode_seconds: float):
    """One episode; act_fn(obs, info) -> action (18,) in [-1,1]."""
    from rl_move.sim.joint_task import q_rad_to_action
    obs, info = env.reset()
    qnom_act = q_rad_to_action(env._q_nom).astype(np.float32)
    rows = []
    n = int(round(episode_seconds / env.dt))
    term = trunc = False
    for t in range(n):
        act = act_fn(obs, info if t else {"bc_target": qnom_act})
        obs, r, term, trunc, info = env.step(act)
        tr, tp = env._true_roll_pitch()
        tgt = info.get("bc_target")
        rows.append({
            "t": t,
            "true_roll_deg": round(tr * RAD2DEG, 2),
            "true_pitch_deg": round(tp * RAD2DEG, 2),
            "rel_roll_deg": round(
                (env._state.imu_roll - env._tilt_ref0[0]) * RAD2DEG, 2),
            "reward": round(float(r), 4),
            "h_mm": round((float(env.data.xpos[env._chassis_bid, 2])
                           - env._z0) * 1e3, 1),
            "n_contact": int(sum(
                bool(env.data.sensordata[a] > CONTACT_N)
                for a in env._touch_adr if a >= 0)),
            "mse_act_tgt": (None if tgt is None else round(float(
                np.mean((np.asarray(act, np.float32)
                         - np.asarray(tgt, np.float32)) ** 2)), 6)),
            "mse_qnom_tgt": (None if tgt is None else round(float(
                np.mean((qnom_act - np.asarray(tgt, np.float32)) ** 2)),
                6)),
        })
        if term or trunc:
            break
    tail = rows[-int(3.0 / env.dt):]
    med = lambda k: (float(np.median([abs(r_[k]) for r_ in tail
                                      if r_[k] is not None]))
                     if any(r_[k] is not None for r_ in tail) else None)
    return {
        "terminated": bool(term),
        "ticks": len(rows),
        "spawn_roll_deg": rows[0]["true_roll_deg"] if rows else None,
        "tail_med_true_roll_deg": med("true_roll_deg"),
        "tail_med_rel_roll_deg": med("rel_roll_deg"),
        "mean_reward": round(float(np.mean([r_["reward"]
                                            for r_ in rows])), 4),
        "tail_med_mse_act_tgt": med("mse_act_tgt"),
        "tail_med_mse_qnom_tgt": med("mse_qnom_tgt"),
        "rows": rows,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", type=Path, default=None,
                    help="optional trained policy for the policy arm")
    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--tip-deg", type=float, default=8.0)
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cfg-set", action="append", default=None)
    ap.add_argument("--out", type=Path,
                    default=Path("/tmp/tilt_teacher.json"))
    args = ap.parse_args()

    arms: dict[str, list] = {}

    def run_arm(name, tilt_comp, act_builder):
        out = []
        for i in range(args.episodes):
            env = _mk_env(tilt_comp, args.tip_deg, args.seed + i,
                          args.episode_seconds, args.cfg_set)
            ep = _episode(env, act_builder(env), args.episode_seconds)
            env.close()
            out.append(ep)
            print(f"[{name}] ep{i}: spawn {ep['spawn_roll_deg']}deg -> "
                  f"tail |roll| {ep['tail_med_true_roll_deg']}deg, "
                  f"reward/tick {ep['mean_reward']}, "
                  f"term={ep['terminated']}, "
                  f"mse(qnom,tgt) {ep['tail_med_mse_qnom_tgt']}")
        arms[name] = out

    def teacher(env):
        def act(obs, info):
            tgt = info.get("bc_target")
            assert tgt is not None, \
                "bc_target missing - anchor not emitting (check cfg)"
            return np.asarray(tgt, np.float32)
        return act

    run_arm("teacher-comp", 1.0, teacher)
    run_arm("teacher-blind", 0.0, teacher)

    if args.ckpt is not None:
        from stable_baselines3 import PPO
        model = PPO.load(args.ckpt, device="cpu")

        def policy(env):
            def act(obs, info):
                a, _ = model.predict(obs, deterministic=True)
                return a
            return act
        run_arm("policy", 1.0, policy)

    dead = 1.5
    print("\n== SUMMARY ==")
    for name, eps in arms.items():
        tails = [e["tail_med_true_roll_deg"] for e in eps
                 if e["tail_med_true_roll_deg"] is not None]
        spawns = [abs(e["spawn_roll_deg"]) for e in eps]
        rew = [e["mean_reward"] for e in eps]
        print(f"{name:14s} spawn|roll| med {np.median(spawns):.2f}deg  "
              f"tail|roll| med {np.median(tails):.2f}deg  "
              f"reward/tick med {np.median(rew):.3f}  n={len(eps)}")
    if spawns:
        L0 = float(np.median([abs(e['spawn_roll_deg'])
                              for e in arms['teacher-comp']]))
        print(f"P-controller fixed-point prediction for teacher-comp: "
              f"(L0 + dead)/2 = ({L0:.1f} + {dead})/2 = "
              f"{(L0 + dead) / 2:.2f}deg (gate bar was <=3deg)")
    args.out.write_text(json.dumps(
        {"tip_deg": args.tip_deg, "episodes": args.episodes,
         "arms": arms}, indent=1))
    print(f"[out] {args.out}")


if __name__ == "__main__":
    main()
