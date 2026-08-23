# cw-walkcurr-pf-fwd6-sde

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:22:50+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: ub6h73yx

**hypothesis**: Plain English: fwd1-fwd5 (8 arms total) all froze in an identical static crouch no matter how the reward charges were priced/ramped, and a cheap W&B-history read (no GPU) confirmed WHY across 3 of those arms: train/clip_fraction collapses to EXACTLY 0 within the first third of every 2M-step run and never recovers -- PPO's per-step i.i.d. Gaussian action noise apparently never produces a rollout with real net forward displacement to reinforce (fresh init/entropy-coef arms already ruled out noise MAGNITUDE; this tests noise STRUCTURE instead). SB3's gSDE (--use-sde) samples ONE noise matrix per rollout collection instead of fresh per-tick noise, so a single draw biases the action mean in a FIXED direction for the whole n-steps window -- the joint dynamics can actually integrate that into a real displacement excursion, unlike i.i.d. noise which tends to cancel out step-to-step. Single lever on the exact fwd3-chargeramp recipe (charge ramp kept, bank-proven safe); everything else byte-identical, fresh init, 2M steps (same budget as every prior arm for a clean comparison). Prediction-if-true: env/walk_freeprog_score leaves its flat [-0.10,-0.05] band and trends toward/above 0 within 2M, gate eval shows nonzero prog_ratio. Prediction-if-false: identical frozen video/flat score under correlated noise too -- noise STRUCTURE is refuted alongside noise MAGNITUDE/entropy, and the fork moves to rung-0 (simpler sub-goal) or a curiosity/RND state-novelty bonus (both still untested, next in STATUS.md's own priority order).

**gate**: Rung-1 gate (same as fwd1-fwd5): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health (08-21 ruling): env/walk_freeprog_score > -0.02 and rising at 2M = continue; still flat in [-0.10,-0.05] = gSDE/noise-structure hypothesis refuted, no same-recipe continuation -- escalate to rung-0/RND.

