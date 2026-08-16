# cw-recover-any11-rsi-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-16T12:22:07+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-recover-any6-microbuckets-scratch1

**wandb_id**: pphzizzw

**hypothesis**: Teach the fallen robot to get up from every starting position, from scratch, with the new on-path practice trick turned on from step 0 instead of bolted on as a late rescue. If RECOVER RSI keeps a policy from ever entrenching the flat-belly (zero) stall in the first place -- rather than trying to cure it after the fact -- a from-scratch run with recover_rsi_frac=0.5 (recover_rsi_kinds=zero, default) should climb the ladder as cleanly as any6/any7's plain curriculum (no RSI) did, reaching zero and then the tangle wall (B12-15) within the 40M budget, giving a second, RSI-protected data point on the tangle wall. Exact any6 recipe/config otherwise (16-episode certs and 3 retention buckets like any7/8/9/10, for a low-noise read). Prediction-if-true: frontier passes bucket 11 with CERT success_fraction >=0.8 within a few certs of first reaching it (no multi-cert stall), matching any6/7's clean per-bucket promotion pace, and goes on to reach at least B12 (tangle_mild unlocked). Prediction-if-false: even a fresh, never-poisoned policy stalls at bucket 11 the same way any8 did -- which would mean RSI's fix only works when applied surgically to a targeted policy state, not as a blanket exposure change, and single out something else (dose, timing, reward) as the real lever.

**gate**: Read at 40M or earlier plateau: bucket 11 (zero) CERT success_fraction must reach >=0.8 within 3M steps of first becoming the frontier (i.e. no multi-cert stall like any8/9/10), AND the frontier must legitimately (>=0.8 gate_fraction, 16-ep denominator) reach at least B12 (tangle_mild) by 40M with buckets 0-10 retention >=0.8 at the final cert. FAIL (zero stalls >3M steps at frontier, or never reaches B12) = RSI does not generalize to from-scratch protection either; close the RSI-for-zero avenue entirely and treat the zero-bucket wall as needing a genuinely new mechanism (reward-side or BC-teacher-side), not curriculum/exposure of any kind. video-verify the earned frontier (no flag/stilt/park).

