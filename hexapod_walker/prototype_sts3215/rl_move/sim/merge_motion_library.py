"""merge_motion_library.py — splice named clip families from one AMP
motion-library .npz into another, tick-range-exact, everything else
byte-identical.

Motivation (08-23, amp yaw fork q_20260823T1750Z/adopted answer on
q_20260823T0130Z): six reward/demo/discriminator-obs mechanism classes
were refuted for the turn-tracking miss, and a follow-up eval-only
probe (`logs/ckpt_eval/cpg_turnauthority_wz030/`) confirmed 0.30 rad/s
body yaw IS plant-reachable (the CPG-search controller achieves it),
so the miss is a genuine demo/policy capability gap, not a physically
unreachable bar. The two demo-side fixes tried so far were both
ALL-OR-NOTHING swaps: `teacher_v3` re-scaled ONLY the tripod's own
turn-clip stride/period (a modest +30% ceiling, 0.144->0.174 rad/s,
`-turnlib3` FAIL, tips unmoved) and `cpgdemo1` swapped the ENTIRE
library to the CPG source (turn clips genuinely faster, but every
OTHER family changed too and slip regressed 3.67->4.37, `-cpgdemo1`
FAIL). Neither isolates "does the AMP discriminator learn to tolerate
a faster turn-in-place demo" from "does swapping the whole gait
generator change something else." This tool builds the missing
surgical arm: keep every non-turn family exactly as the base library
(teacher_v2, whose non-turn slip is the campaign's working baseline)
and splice in ONLY the named turn families from an override library
(cpg_v1, whose turn_ccw/turn_cw clips are grounded in the SAME
controller just measured to achieve ~0.29 rad/s at commanded 0.30,
nearly 2x teacher_v3's ceiling).

Alignment requirement (checked, not assumed): the two source files
must have IDENTICAL clip_names/clip_seeds ordering (same command
suite, same seed grid — true of every teacher_v1/v2/cpg_v1 sibling
built by build_motion_library.py's shared `--seeds`/family list) and
the same obs_style width + dt, so a per-clip-index splice is
well-defined and the merged file's per-clip neutral-pose invariant
(MotionLibrary's own hard-fail check, q_20260822T0900Z) still holds.
Fails loudly (no silent partial merge) on any mismatch.

Usage:
  python3 -m rl_move.sim.merge_motion_library \
      --base rl_move/sim/motion_library/teacher_v2.npz \
      --override rl_move/sim/motion_library/cpg_v1.npz \
      --families turn_ccw,turn_cw \
      --out rl_move/sim/motion_library/teacher_v4.npz
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

# Per-tick arrays that must be spliced clip-range-by-clip-range (every
# other key -- clip_starts/clip_lens/clip_names/clip_seeds/dt -- is
# either recomputed or copied from the base file's own metadata).
TICK_KEYS = ("joint_position", "joint_position_rel_neutral", "joint_velocity",
             "base_orientation", "base_angular_velocity", "projected_gravity",
             "foot_positions", "phase_label", "command", "obs_style")


def merge(base_path: Path, override_path: Path,
          families: list[str]) -> dict[str, np.ndarray]:
    b = np.load(base_path, allow_pickle=True)
    o = np.load(override_path, allow_pickle=True)

    b_names = [str(n) for n in b["clip_names"]]
    o_names = [str(n) for n in o["clip_names"]]
    if b_names != o_names:
        raise ValueError(
            f"{base_path.name} and {override_path.name} have different "
            "clip_names orderings -- per-clip-index splice is not "
            "well-defined; rebuild both from the same --seeds/family list")
    b_seeds = list(b["clip_seeds"])
    o_seeds = list(o["clip_seeds"])
    if b_seeds != o_seeds:
        raise ValueError(
            f"{base_path.name} and {override_path.name} have different "
            "clip_seeds orderings for the same clip_names -- refusing "
            "to splice mismatched seed draws")
    if float(b["dt"]) != float(o["dt"]):
        raise ValueError(f"dt mismatch: {base_path.name}={float(b['dt'])} "
                         f"vs {override_path.name}={float(o['dt'])}")
    if b["obs_style"].shape[1] != o["obs_style"].shape[1]:
        raise ValueError(
            f"obs_style width mismatch: {base_path.name}="
            f"{b['obs_style'].shape[1]} vs {override_path.name}="
            f"{o['obs_style'].shape[1]}")

    fam_set = set(families)
    unknown = fam_set - set(b_names)
    if unknown:
        raise ValueError(f"--families {sorted(unknown)} not present in "
                         f"{base_path.name}'s clip_names")

    n_clips = len(b_names)
    chunks: dict[str, list[np.ndarray]] = {k: [] for k in TICK_KEYS}
    new_starts, new_lens = [], []
    cursor = 0
    n_from_override = 0
    for i in range(n_clips):
        src, tag = (o, "override") if b_names[i] in fam_set else (b, "base")
        if tag == "override":
            n_from_override += 1
        s0 = int(src["clip_starts"][i])
        ln = int(src["clip_lens"][i])
        for k in TICK_KEYS:
            chunks[k].append(np.asarray(src[k])[s0:s0 + ln])
        new_starts.append(cursor)
        new_lens.append(ln)
        cursor += ln

    out: dict[str, np.ndarray] = {
        k: np.concatenate(chunks[k], axis=0) for k in TICK_KEYS}
    out["clip_starts"] = np.asarray(new_starts, dtype=np.int64)
    out["clip_lens"] = np.asarray(new_lens, dtype=np.int64)
    out["clip_names"] = np.asarray(b_names)
    out["clip_seeds"] = np.asarray(b_seeds, dtype=np.int64)
    out["dt"] = np.asarray(float(b["dt"]))

    # Per-clip neutral-pose invariant (mirrors MotionLibrary's own load-
    # time check, q_20260822T0900Z): every clip -- base or spliced --
    # must resolve to the SAME neutral, or the merged file is unusable
    # by the live trainer. Fail loudly here (cheap, offline) rather
    # than at training-launch time.
    jp = out["joint_position"]
    rel = out["joint_position_rel_neutral"]
    neutrals = np.stack([jp[s] - rel[s] for s in out["clip_starts"]])
    spread = float(np.abs(neutrals - neutrals.mean(axis=0)).max())
    if spread > 1e-6:
        raise ValueError(
            f"merged library has per-clip neutrals that differ (max "
            f"spread {spread:.3e}) -- the spliced clips do not share "
            "the base library's neutral pose convention")

    print(f"merged {n_from_override}/{n_clips} clips from "
          f"{override_path.name} (families {sorted(fam_set)}), "
          f"{n_clips - n_from_override} kept from {base_path.name}; "
          f"{cursor} total ticks, neutral spread {spread:.2e}")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", type=Path, required=True)
    ap.add_argument("--override", type=Path, required=True)
    ap.add_argument("--families", required=True,
                    help="comma-separated clip_names to take from "
                         "--override instead of --base")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    families = [f.strip() for f in args.families.split(",") if f.strip()]
    out = merge(args.base, args.override, families)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, **out)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
