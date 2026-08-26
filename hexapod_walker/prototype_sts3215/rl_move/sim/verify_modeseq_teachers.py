"""In-env sequence-competence check for a teacher pair (goal.mode_seq).

The trans-dagger2 kill (08-14) found the two switch implementations
contradicting each other on the same checkpoints: footlow2_hard1 +
walk_longdist_r2 fell 99/225 demo sequences inside the goal.mode_seq
TRAINING env (lower 73) while scoring 11/12 zero-fall on the
eval_modeseq instrument, whose frames come from the composition-proven
reanchor_to(). Root cause: the v1 in-env switch carried the
episode-reset q_nom across segments (obs joints are q - q_nom, so a
rise-start sequence fed every later plant-family segment a belly frame
~79 deg off at the knees). The fix installs the canonical per-family
settle-probe frames at every switch (sim_env._seq_capture_frames /
_seq_maybe_switch).

This driver re-measures that number: it drives a teacher pair through
the EXACT distill_gru --transitions collection context (same cfg
overlay, same env construction, same collect_transitions loop, DR 0.5,
stochastic_frac 0.3, first-mix stance-heavy) and reports falls by
segment mode. Pass condition per the directive's ARM 1 RESULT note:
footlow2_hard1 composes in-env at ~ its instrument rate (~1/12 det
falls), not 44%.

    uv run python -m rl_move.sim.verify_modeseq_teachers \
        --stance-teacher rl_move/sim/policies/ppo_goal_cw_stand_footlow2_hard1.zip \
        --walk-teacher rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip \
        --episodes 225
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rl_move.sim.sim_env import SimServoParams  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stance-teacher", type=Path, required=True)
    ap.add_argument("--walk-teacher", type=Path, required=True)
    ap.add_argument("--episodes", type=int, default=225)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dr-scale", type=float, default=0.5)
    ap.add_argument("--stochastic-frac", type=float, default=0.3)
    ap.add_argument("--episode-seconds", type=float, default=30.0)
    ap.add_argument("--seq-segment-s", type=str, default="6,8")
    ap.add_argument("--seq-first-mix", type=str,
                    default="rise=0.40,walk=0.30,lower=0.15,hold=0.15")
    ap.add_argument("--out", type=Path, default=None,
                    help="write the stats record as JSON")
    # 08-26 (standwalk stage-2 design): the original R3_CFG overlay is
    # a stale r3/r4c-era recipe snapshot -- it does NOT match any real
    # walk teacher's actual training cfg (goal.walk_phase_obs,
    # walk_obs_body_vel etc change obs WIDTH; a mismatch crashes with
    # "checkpoint obs width does not fit env"). This lets a caller pass
    # a teacher's own launch --cfg-set list verbatim (env.model_source
    # / control.hz included) so the composed env matches what BOTH
    # teachers actually trained under. Parsed with the same
    # _parse_cfg_set used by every other harness -- one dotted-key
    # convention repo-wide.
    ap.add_argument("--extra-cfg-set", action="append", default=[],
                    help="k=v env/goal/reward/safety overrides layered "
                         "on top of the default sequence-probe overlay "
                         "(e.g. a walk teacher's own training cfg, or "
                         "env.model_source=mesh control.hz=100).")
    args = ap.parse_args(argv)

    from stable_baselines3 import PPO

    from .distill_gru import (
        DIET, _build_cfg, _make_env, collect_transitions,
    )
    from .train_ppo_sim import _parse_cfg_set

    seg_lo, seg_hi = (float(x) for x in args.seq_segment_s.split(","))
    overlay = {"obs.mode_onehot": 1.0,
               "goal.mode_seq": 1.0,
               "goal.mode_seq_segment_s_min": seg_lo,
               "goal.mode_seq_segment_s_max": seg_hi}
    overlay.update(_parse_cfg_set(args.extra_cfg_set))
    cfg = _build_cfg(overlay)
    params = SimServoParams.load()
    env = _make_env(args, cfg, params)
    first_mix = {k: float(v) for k, v in
                 (kv.split("=") for kv in args.seq_first_mix.split(","))}
    env.set_goal_mix({m: first_mix.get(m, 0.0) for m in DIET})

    walk = PPO.load(args.walk_teacher, device="cpu")
    stance = PPO.load(args.stance_teacher, device="cpu")
    teachers = {"walk": (walk, int(walk.observation_space.shape[0])),
                "stance": (stance, int(stance.observation_space.shape[0]))}
    print(f"[verify-modeseq] {args.episodes} sequences, DR "
          f"{args.dr_scale}, teachers "
          f"{args.walk_teacher.name} + {args.stance_teacher.name}")

    rng = np.random.default_rng(args.seed)
    t0 = time.monotonic()
    # verify_n=0: this IS the verification — report everything, never
    # abort early on the falls-only cap.
    _eps, stats = collect_transitions(env, teachers, args.episodes,
                                      args.stochastic_frac, rng,
                                      verify_n=0)
    n = args.episodes
    det_eps = [e for e in stats["eps"] if e["det"]]
    det_falls = sum(1 for e in det_eps if e["fall"] is not None)
    rets = [e["ret"] for e in stats["eps"]]
    record = {
        "stance_teacher": str(args.stance_teacher),
        "walk_teacher": str(args.walk_teacher),
        "episodes": n, "dr_scale": args.dr_scale,
        "stochastic_frac": args.stochastic_frac, "seed": args.seed,
        "falls": stats["falls"], "fall_rate": stats["falls"] / n,
        "fall_modes": stats["fall_modes"],
        "det_eps": len(det_eps), "det_falls": det_falls,
        "teacher_return_med": float(np.median(rets)),
        "teacher_return_min": float(min(rets)),
        "elapsed_s": round(time.monotonic() - t0, 1),
    }
    print(f"[verify-modeseq] RESULT: {stats['falls']}/{n} sequences "
          f"fell ({100.0 * stats['falls'] / n:.1f}%), by mode "
          f"{stats['fall_modes']}; det {det_falls}/{len(det_eps)}; "
          f"teacher return med {record['teacher_return_med']:.0f} "
          f"min {record['teacher_return_min']:.0f}")
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(record, indent=1))
        print(f"[verify-modeseq] wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
