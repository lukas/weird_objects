# cw-amp-m2-freeprog-term400-fixedcmd-seed11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T16:37:13+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-noamp

**wandb_id**: i9iets9t

**hypothesis**: Plain English: test whether a from-scratch policy can learn to walk forward AT ALL once the task is the exact simplest case the SLIPWALK semantics bank itself already validates -- one fixed slow forward speed (0.05 m/s) held the whole episode, no direction changes, no stops, no turns, no mid-episode resampling. Single lever vs cw-amp-m2-freeprog-term400-noamp: replace the goal.* command config with the literal SLIPWALK_OVERRIDES fixed command; byte-identical reward stack, no AMP. See --notes on the launched run for the full writeup.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg -- eval auto-matches this run's own fixed-command training cfg). PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six-leg cycling with net forward displacement, read against the SLIPWALK bank's own gait(+417,0.22m)/creep(+108,0.16m)/stall(-143)/park(-244) reference. Prediction-if-true: task complexity (not reward shape) was the barrier -- revisit stress_mix/stagecurric with a longer held stage-0. Prediction-if-false: statue persists at the bank's own simplest case -- genuine PPO exploration/basin-barrier problem; next lever is RSI or a stance-load curriculum, not another task/dose tweak.

