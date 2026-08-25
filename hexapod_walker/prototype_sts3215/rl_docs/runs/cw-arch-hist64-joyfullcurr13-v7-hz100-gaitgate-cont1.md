# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T01:58:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1

**hypothesis**: Plain English: does turning on the already-built, already-bank-proven anti-leg-sacrifice reward gate (reward.walk_gait_gate, unused in this recipe) let a checkpoint that is CURRENTLY stuck sacrificing legs [0,2,5] (this exact run's FAIL verdict: sac=[0,2,5] in 5/6 det + 5/6 sto own-DR episodes, 40M steps already spent) recover into a real six-leg gait with 10M more steps? Mechanism: walk_gait_gate multiplicatively collapses walk/progress/tracking income toward zero whenever ANY support leg has not completed a real swing within a 2s window (min across legs, not mean) -- it was purpose-built+bank-tested for exactly this 'sacrifice any subset of legs' cheat class (test_walk_gait_gate_collapses_flag_leg_income, quadwalk mid-pin sibling test) but was never included in the V7/100Hz joyfullcurr13 recipe that has now shown this fingerprint on 3 independent lineages (OPERATOR_QUESTIONS.md 08-24/08-25 notes). This is a single-lever CONTINUATION test: does the gate's income collapse retrain the existing bad optimum, or is the basin too deep once converged (matching walkcurr's own repeated finding that reward-side fixes on an already-parked policy usually fail to dislodge it)? Prediction-if-true: legs [0,2,5] duty rises out of the ~0.0-0.09 band, walk_gait_min tail rises off 0, held-out gait_valid improves from 0/6. Prediction-if-false: identical [0,2,5] lock persists despite the new charge (reward may fall hard without behavior change) -- an aligned FAIL per the 08-21 ruling, and the decisive test becomes the sibling from-scratch arm (gaitgate-scratch1) instead. Strongest alternative: escaping requires never forming the bad optimum in the first place, which the from-scratch sibling tests directly.

**gate**: PASS: legs [0,2,5] (or any subset) duty moves into the ~0.3-0.8 healthy band on the held-out joygate/owncfg re-eval, gait_valid clears >=4/6 det, no new leg sacrificed. PARTIAL: duty moves off the 0.0-0.09 floor and walk_gait_min tail rises but gait_valid still <4/6 -- genuine partial recovery, continuation candidate. FAIL: identical sac=[0,2,5] lock (or an equally degenerate new sacrifice set) with reward flat/falling and no duty movement -- aligned FAIL, escalate to the from-scratch sibling as the decisive read.

**verdict**: INFRA FAIL (pre-training): landed on a legacy-64M-shm pod (train-0), _check_shm_budget correctly refused before any 24x-SIGBUS silent death (99.5MB needed vs 67.1MB total shm). 0 GPU/wandb budget spent (crashed before wandb init). Retrying on train-8 (verified 4.0G tmpfs).

