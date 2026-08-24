"""Alignment audit of the STATE-ALIGNED BC-anchor reference-tick
selection at live policy states (the cw-stand-footlow1 dig-in,
2026-08-12).

Question (pre-registered in CURRENT_TRUTHS / RISE.md): when the det/sto
flat rise stalls at 90-107mm height error under the merged anchor stack
(footz1-hard1 hold fix + anchormix1-r1 lower fix), WHICH reference tick
does sim_env's state-aligned matcher select at the stalled belly state,
and what does its +lookahead pursuit target actually command?

Two candidate mechanisms, distinguishable only by measurement:
  (a) PLATEAU FIXED POINT: the recorded reference
      (rise_ref_belly2plant.npz) spends ticks 0-125 (5.0s) in a flat
      belly hold at h=0 and only reaches +25mm by tick 240 - so a
      low/stalled state matches into a spatially-flat segment where the
      +0.5s (12-tick) lookahead commands a pose ~1-3mm higher: the
      "pursuit" target is a near-fixed point and the anchor actively
      supervises staying put.
  (b) OFF-PATH MATCH: the stalled pose is far from every reference tick
      (large joint-RMS at the argmin), so the match index (and hence
      the target) is arbitrary/wrong.

The probe replays the run's own rise episodes (det + sto, forced start
kinds, the run's exact cfg-set stack) and recomputes, per tick, exactly
what sim_env._step_finish would emit:
  j_state  = argmin_j mean((q_now - ref.q[j])^2)   (state-aligned match)
  jn       = min(j_state + round(lookahead_s/ref.dt), T-1)
  target   = ref.q[jn]
plus the legacy clock index for comparison, the joint-RMS distance at
the match, the reference height at j/jn vs the actual chassis height,
the policy's own action-vs-target MSE (is PPO doing what the anchor
asks?), and the per-foot touch contacts.

Output: JSON trace + a per-episode stall summary to stdout. Pure
diagnostic - no training, no env-code changes, C env on CPU.

Usage (on the run's pod):
    uv run python -m rl_move.sim.probe_anchor_align \
        rl_move/sim/policies/ppo_goal_cw_stand_footlow1.zip \
        --cfg-set ... (the run's stack) --out /tmp/anchor_align.json
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--task", default="joint_goal")
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--starts", nargs="*",
                    default=["flat", "flat", "bridge"],
                    help="forced rise start kinds, one episode each; "
                         "the list runs once deterministic then once "
                         "stochastic")
    ap.add_argument("--cfg-set", action="append", default=None)
    ap.add_argument("--out", type=Path, default=Path("/tmp/anchor_align.json"))
    args = ap.parse_args()

    from rl_move.config import load_config, cfg_get
    from .train_ppo_sim import ENV_CLASSES, _parse_cfg_set
    from .servo_model import SimServoParams
    from .sim_env import load_rise_ref
    from .joint_task import q_rad_to_action

    cfg = load_config()
    if args.cfg_set:
        for key, parsed in _parse_cfg_set(args.cfg_set).items():
            sect, name = key.split(".", 1)
            cfg.setdefault(sect, {})[name] = parsed
    ref_path = cfg_get(cfg, "reward", "rise_ref_path", default=None)
    assert ref_path, "reward.rise_ref_path missing from cfg stack"
    ref = load_rise_ref(str(ref_path))
    T = len(ref["q"])
    look_s = float(cfg_get(cfg, "train", "bc_anchor_lookahead_s",
                           default=0.25))
    ahead = max(int(round(look_s / ref["dt"])), 1)
    ref_h = ref.get("h")
    # HEIGHT-FLOOR pursuit (train.bc_anchor_min_h_ahead_mm, 08-12):
    # mirror sim_env._step_finish exactly — when the floor is active,
    # the target tick must command >= min_h mm above the CURRENT
    # chassis height (first such tick at/after the match; path end if
    # none): effective ahead = max(time_ahead, floor_j - j). Without
    # this the probe audits the LEGACY target, not what a floored run
    # (cw-stand-footlow2+) actually trains against.
    min_h_mm = float(cfg_get(cfg, "train", "bc_anchor_min_h_ahead_mm",
                             default=0.0))
    print(f"[ref] {ref_path}: T={T} dt={ref['dt']} ramp_i0={ref['ramp_i0']} "
          f"lookahead={look_s}s -> {ahead} ticks; "
          f"min_h_ahead={min_h_mm}mm"
          f"{' (ref has no h — floor no-op)' if ref_h is None else ''}")

    env_cls = ENV_CLASSES[args.task]
    env = env_cls(params=SimServoParams.from_cfg(cfg), randomize=False,
                  dr_scale=0.0, episode_seconds=args.episode_seconds,
                  seed=args.seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "walk", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
    # No RSI spawns: the stall is a cold-start phenomenon.
    try:
        cfg.setdefault("goal", {})["rise_rsi_frac"] = 0.0
    except Exception:
        pass

    from stable_baselines3 import PPO
    model = PPO.load(args.checkpoint, device="cpu")

    episodes = []
    for det in (True, False):
        for start in args.starts:
            gen.force_rise_start = start
            obs, info0 = env.reset()
            rows = []
            term = trunc = False
            n = int(round(args.episode_seconds / env.dt))
            for t in range(n):
                act, _ = model.predict(obs, deterministic=det)
                obs, _r, term, trunc, info = env.step(act)
                qnow = np.asarray(env.data.qpos[env._qadr], dtype=float)
                d2 = ((ref["q"] - qnow[None, :]) ** 2).mean(axis=1)
                j = int(np.argmin(d2))
                jn_time = min(j + ahead, T - 1)
                jn = jn_time
                floor_on = False
                h_rel_pre = float(env.data.xpos[env._chassis_bid, 2]) \
                    - env._z0
                if min_h_mm > 0.0 and ref_h is not None:
                    ks = np.flatnonzero(
                        ref_h[j:] >= h_rel_pre + min_h_mm * 1e-3)
                    floor_j = (j + int(ks[0])) if len(ks) else T - 1
                    jn = min(j + max(ahead, floor_j - j), T - 1)
                    floor_on = jn != jn_time
                j_clock, is_rsi = env._rise_ref_clock(ref)
                tgt = q_rad_to_action(ref["q"][jn]).astype(np.float32)
                # what the anchor pull actually asks vs what policy does
                mse = float(np.mean((np.asarray(act, dtype=np.float32)
                                     - tgt) ** 2))
                # commanded height intent of the target vs current height
                h_rel = float(env.data.xpos[env._chassis_bid, 2]) - env._z0
                contacts = [bool(env.data.sensordata[a] > CONTACT_N)
                            if a >= 0 else False for a in env._touch_adr]
                goal = env._current_goal()
                rows.append({
                    "t": t, "h_mm": round(h_rel * 1e3, 1),
                    "h_ref_goal_mm": round(float(goal.height_ref) * 1e3, 1)
                        if goal is not None else None,
                    "j": j, "jn": jn, "jn_time": jn_time,
                    "floor_on": floor_on, "j_clock": int(j_clock),
                    "d_rms_deg": round(float(np.sqrt(d2[j])) * RAD2DEG, 2),
                    "ref_h_j_mm": round(float(ref_h[j]) * 1e3, 1)
                        if ref_h is not None else None,
                    "ref_h_jn_mm": round(float(ref_h[jn]) * 1e3, 1)
                        if ref_h is not None else None,
                    "mse_act_tgt": round(mse, 5),
                    "n_contact": int(sum(contacts)),
                })
                if term or trunc:
                    break
            h_err_end = (rows[-1]["h_mm"] - rows[-1]["h_ref_goal_mm"]
                         if rows and rows[-1]["h_ref_goal_mm"] else None)
            episodes.append({"det": det, "start": start,
                             "terminated": bool(term),
                             "h_err_end_mm": h_err_end, "rows": rows})
            # ---- readable stall summary
            tail = rows[-int(3.0 / env.dt):]
            js = [r["j"] for r in tail]
            print(f"\n== {'det' if det else 'sto'}/{start}: end h_err "
                  f"{h_err_end}mm term={term}")
            print(f"   last-3s j_state: min {min(js)} max {max(js)} "
                  f"(advance {max(js)-min(js)} ticks); "
                  f"j_clock end {tail[-1]['j_clock']}")
            print(f"   last-3s median: d_rms "
                  f"{np.median([r['d_rms_deg'] for r in tail]):.2f}deg, "
                  f"h {np.median([r['h_mm'] for r in tail]):.1f}mm, "
                  f"ref_h[j] {np.median([r['ref_h_j_mm'] for r in tail]):.1f}mm, "
                  f"ref_h[jn] {np.median([r['ref_h_jn_mm'] for r in tail]):.1f}mm, "
                  f"mse(act,tgt) {np.median([r['mse_act_tgt'] for r in tail]):.4f}, "
                  f"contacts {np.median([r['n_contact'] for r in tail]):.0f}/6, "
                  f"floor_on {np.mean([r['floor_on'] for r in tail]):.2f}")
            for r in rows[:: max(1, int(1.0 / env.dt))]:
                print(f"   t={r['t']:3d} h={r['h_mm']:7.1f} goal="
                      f"{r['h_ref_goal_mm']:6.1f} j={r['j']:3d} "
                      f"jn={r['jn']:3d} jc={r['j_clock']:3d} "
                      f"d={r['d_rms_deg']:5.2f} refh_j={r['ref_h_j_mm']:6.1f} "
                      f"refh_jn={r['ref_h_jn_mm']:6.1f} "
                      f"mse={r['mse_act_tgt']:7.4f} c={r['n_contact']}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(
        {"checkpoint": str(args.checkpoint), "lookahead_s": look_s,
         "ahead_ticks": ahead, "episodes": episodes}, indent=1))
    print(f"\n[probe_anchor_align] wrote {args.out}")


if __name__ == "__main__":
    main()
