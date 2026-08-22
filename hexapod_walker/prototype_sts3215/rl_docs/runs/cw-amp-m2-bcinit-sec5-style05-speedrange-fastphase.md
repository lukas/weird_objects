# cw-amp-m2-bcinit-sec5-style05-speedrange-fastphase

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T22:24:04+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-speedrange

**wandb_id**: n79getfy

**hypothesis**: Plain English: the speedrange arm just showed the walker's achieved speed barely tracks the commanded 0.05-0.25 m/s range, clustering 0.08-0.14 m/s instead -- is that a fixed STEP-RATE cap (the phase-obs clock ticks at a constant 1.333 Hz no matter what speed is commanded, so the actor never learned to step faster for faster commands)? Continues from the speedrange checkpoint (same sec5 reward + amp 0.5/0.5, fresh disc per stage protocol), single lever: goal.walk_phase_hz 1.333333 -> 2.0 (50% faster reference clock, same speed range). Prediction-if-true: gait stays valid (>=5/6 det+sto) and speed_mean's realized range widens meaningfully above the parent's ~0.08-0.14m/15s band (evidence the actor now steps faster on faster commands). Prediction-if-false-A: gait destabilizes (terminations/sacrificed legs) -- the actor cannot resync to a faster clock this late in the curriculum. Prediction-if-false-B: gait stays clean but speed_mean stays pinned in the same ~0.08-0.14m/15s band regardless -- the cap is not the observation clock rate at all (candidates: motion-library/style reward anchored to the ORIGINAL teacher cadence, or actor capacity), which the paired -nostyle arm (same lever, style weight zeroed) is designed to disambiguate. Strongest alternative: partial -- low commands unaffected, only the high end widens a little.

**gate**: Discovery continuation (2M, DR-0, judged jointly with the -nostyle sibling). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, height_err stays in the 14-31mm walking band, no new sacrificed legs/terminations, AND speed_mean's realized spread widens meaningfully past the parent's ~0.08-0.14 m/s band (real evidence the cadence cap moved). NO-CHANGE = gait stays clean but speed_mean stays pinned in the same ~0.08-0.14 band -- phase_hz is not the lever, redirect to the style/motion-library cadence anchor (see -nostyle sibling) or accept the current envelope as M2's real ceiling. FAIL-collapse = terminations/sacrificed legs/statue return from the faster clock.

**verdict**: A faster gait clock alone does not let the robot follow different speed commands - the realized speed stays pinned near 0.10 m/s no matter what is asked. Joint read with the -nostyle twin (pre-registered): walk_phase_hz 1.333->2.0 kept the gait perfectly healthy (DR-0 gate gait_valid 6/6 det + 6/6 sto, zero terminations/sacrificed legs, clean six-leg cycling on the det strip) but realized speed_mean spans only 0.075-0.112 det / 0.094-0.106 sto against commands of 0.053-0.171 in-sample - NO widening vs the speedrange parent's 0.084-0.136, and the band center did not even shift up despite a 1.5x faster reference clock. Why: the clock is still speed-INDEPENDENT (walk_task._augment_obs advances it at fixed hz whenever s_ref>1e-3), so no clock rate constant can create speed MODULATION - the actor has no cadence signal correlated with the command. Next: code fix - couple the phase-clock rate to commanded speed (new cfg key, default OFF), then continue this lineage with coupling ON.

