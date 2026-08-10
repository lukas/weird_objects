# cw-walk-lpband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-09T23:28:17+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: grxddsed

**hardware_ready**: False

**hypothesis**: WISHLIST 8b/19 (operator-tunable speed, learning-progress curriculum): cw-walk-fast FAILED to learn 0.08-0.12 m/s because a uniform-wide command band gave no gradient off the champion's 0.065 m/s paddle optimum. goal.walk_lp_curriculum=1 (existing code, never launched) buckets speed 0.02-0.12 m/s and reweights sampling toward buckets where tracking is IMPROVING rather than a flat wide draw. If-true: own-cfg per-bucket panel shows prog_ratio>=0.75 (or a wider trained sub-band than fast's flat 0.065) with gait_valid, 0 term -- speed becomes a runtime command over more than the champion's narrow slice. If-false: paddling optimum is a hard sim-pricing attractor regardless of sampling curriculum -- report the actual learned band, don't retry a 3rd uniform-band arm.

**gate**: Own-cfg harness, DR0, det+sto 6/6 per speed bucket (low 0.02-0.04, mid 0.05-0.07, high 0.08-0.12): gait_valid 12/12 per bucket tested, 0 term, prog_ratio med >=0.75 in at least the low+mid buckets (vs fast's uniform failure at ALL buckets incl. mid); frames watched det for one episode per bucket.

**verdict**: FAIL (hypothesis REFUTED). Learning-progress speed curriculum does NOT fix command-insensitive speed: at 20M steps, low-band (0.02-0.04 m/s target) det prog med 1.55 (overshoots), mid-band (0.05-0.07, near champion default) prog med 0.89 (correct), high-band (0.08-0.12) prog med 0.55 (undershoots) -- but absolute distance covered is FLAT at ~1.4-1.6m/30s across ALL THREE bands (same as champion default pace), meaning the policy walks its one learned gait speed regardless of command, exactly like cw-walk-fast (uniform-band) FAILED the same way. Confirms this is NOT a sampling/curriculum problem -- the walk speed ceiling is a gait-pricing wall (root: sim contact/current pricing, operator-calibration class per RL_PLAN), same conclusion via a second independent method. Gait stays valid (12/12 gv, 0 term) at all bands, no pathology -- just speed-invariant. Closes wishlist 8b/19 pending operator contact-pricing calibration; do not retry a 3rd sampling-scheme variant of this idea.

