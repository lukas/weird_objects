# cw-amp-m2-bcinit-sec5-style05-speedrange-fastphase

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T22:24:04+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-speedrange

**wandb_id**: n79getfy

**hypothesis**: Plain English: the speedrange arm just showed the walker's achieved speed barely tracks the commanded 0.05-0.25 m/s range, clustering 0.08-0.14 m/s instead -- is that a fixed STEP-RATE cap (the phase-obs clock ticks at a constant 1.333 Hz no matter what speed is commanded, so the actor never learned to step faster for faster commands)? Continues from the speedrange checkpoint (same sec5 reward + amp 0.5/0.5, fresh disc per stage protocol), single lever: goal.walk_phase_hz 1.333333 -> 2.0 (50% faster reference clock, same speed range). Prediction-if-true: gait stays valid (>=5/6 det+sto) and speed_mean's realized range widens meaningfully above the parent's ~0.08-0.14m/15s band (evidence the actor now steps faster on faster commands). Prediction-if-false-A: gait destabilizes (terminations/sacrificed legs) -- the actor cannot resync to a faster clock this late in the curriculum. Prediction-if-false-B: gait stays clean but speed_mean stays pinned in the same ~0.08-0.14m/15s band regardless -- the cap is not the observation clock rate at all (candidates: motion-library/style reward anchored to the ORIGINAL teacher cadence, or actor capacity), which the paired -nostyle arm (same lever, style weight zeroed) is designed to disambiguate. Strongest alternative: partial -- low commands unaffected, only the high end widens a little.

**gate**: Discovery continuation (2M, DR-0, judged jointly with the -nostyle sibling). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, height_err stays in the 14-31mm walking band, no new sacrificed legs/terminations, AND speed_mean's realized spread widens meaningfully past the parent's ~0.08-0.14 m/s band (real evidence the cadence cap moved). NO-CHANGE = gait stays clean but speed_mean stays pinned in the same ~0.08-0.14 band -- phase_hz is not the lever, redirect to the style/motion-library cadence anchor (see -nostyle sibling) or accept the current envelope as M2's real ceiling. FAIL-collapse = terminations/sacrificed legs/statue return from the faster clock.

