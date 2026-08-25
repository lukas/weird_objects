# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T02:04:29+00:00

**pod**: hexapod-mjx-train-8

**steps**: 10000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1

**wandb_id**: 1t8jxzqo

**hypothesis**: retry of the same-name attempt (see verdict FAIL note): identical hypothesis, only the target pod changes (train-0 -> train-8, verified 4.0G shm) to fix the legacy-shm pre-training crash. Full hypothesis unchanged: does turning on the already-built, already-bank-proven anti-leg-sacrifice reward gate (reward.walk_gait_gate) let a checkpoint CURRENTLY stuck sacrificing legs [0,2,5] recover into a real six-leg gait with 10M more steps, single lever vs the FAIL-verdicted parent.

**gate**: PASS: legs [0,2,5] (or any subset) duty moves into the ~0.3-0.8 healthy band on the held-out joygate/owncfg re-eval, gait_valid clears >=4/6 det, no new leg sacrificed. PARTIAL: duty moves off the 0.0-0.09 floor and walk_gait_min tail rises but gait_valid still <4/6. FAIL: identical sac=[0,2,5] lock (or an equally degenerate set) with reward flat/falling and no duty movement -- escalate to the from-scratch sibling as the decisive read.

