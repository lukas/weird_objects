# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T02:04:29+00:00

**pod**: hexapod-mjx-train-8

**steps**: 10000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1

**wandb_id**: 1t8jxzqo

**hypothesis**: retry of the same-name attempt (see verdict FAIL note): identical hypothesis, only the target pod changes (train-0 -> train-8, verified 4.0G shm) to fix the legacy-shm pre-training crash. Full hypothesis unchanged: does turning on the already-built, already-bank-proven anti-leg-sacrifice reward gate (reward.walk_gait_gate) let a checkpoint CURRENTLY stuck sacrificing legs [0,2,5] recover into a real six-leg gait with 10M more steps, single lever vs the FAIL-verdicted parent.

**gate**: PASS: legs [0,2,5] (or any subset) duty moves into the ~0.3-0.8 healthy band on the held-out joygate/owncfg re-eval, gait_valid clears >=4/6 det, no new leg sacrificed. PARTIAL: duty moves off the 0.0-0.09 floor and walk_gait_min tail rises but gait_valid still <4/6. FAIL: identical sac=[0,2,5] lock (or an equally degenerate set) with reward flat/falling and no duty movement -- escalate to the from-scratch sibling as the decisive read.

**verdict**: Result: FAIL — the walk_gait_gate continuation partially fixes the DUTY-DISTRIBUTION symptom (sac set narrows from {0,2,5} to just leg2, other 5 legs' duty moves into the healthy 0.3-0.98 band, joygate per-leg sacrificed_frac drops to 0.15-0.33 on 5/6 legs) but the checkpoint is WORSE on every safety/gait axis that actually matters: held-out joygate falls 12/48->39/48 (up, not down), gait_valid_frac stays 0.0 (identical to parent), and DR-0 gate now terminates over_current or tilt_pitch in 24/24 episodes (100% term rate vs parent's partial survival) with progress collapsing on several (e.g. sto/4 prog -0.56 slip 10.4). In-training held-out eval/walk/survived_frac DECLINED over the run (0.5 at 1M -> 0.5 at 5M -> 0.0 at 9M) while ep_rew_mean kept rising (quarters 336/318/402/522) -- classic misalignment-not-undertrained per the 08-21 ruling, but here the misalignment got WORSE with more steps, not better: the gate charge trades the 3-leg-sacrifice cheat for a single-stuck-leg drag that trips over_current/tilt_pitch almost immediately (video: walk_det_0 strip shows a hard nose-dive/pitch topple within the first few frames, not a recovering gait). This matches the run's own pre-registered decision tree ('escalate to the from-scratch sibling as the decisive read') more than either PASS or clean PARTIAL -- the reward-side gate alone cannot dislodge this converged bad optimum via continuation. Next: gaitgate-scratch1 (from-scratch with the gate baked in from step 0, already running) is now the sole decisive read on whether walk_gait_gate prevents the sacrifice basin from forming at all; do not fund further continuation-only gaitgate arms off this checkpoint lineage.

