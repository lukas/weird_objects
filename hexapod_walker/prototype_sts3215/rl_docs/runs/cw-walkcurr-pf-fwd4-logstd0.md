# cw-walkcurr-pf-fwd4-logstd0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T18:34:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: 4uh8job8

**hypothesis**: Plain English: fwd1/fwd2/fwd3 all froze in an identical static crouch with walk_freeprog_score flat-negative (~-0.07) for 2M steps regardless of which reward charge got softened -- three reward-shape fixes refuted with the same signature points at an EXPLORATION bottleneck, not a pricing bottleneck: random-init PPO at std=0.37 (log-std-init -1.0, unchanged since fwd1) may simply never sample a forward-progress trajectory to reinforce. This arm keeps the exact fwd3 recipe (charge ramp intact, bank-proven safe) and only widens the initial action-noise std to 1.0 (log-std-init 0.0, ~2.7x wider). Prediction-if-true: walk_freeprog_score leaves its flat ~-0.07 band and trends toward/above 0 within 2M steps, gate eval shows nonzero prog_ratio and lower slip. Prediction-if-false: identical frozen video/flat score at wider noise too -- exploration-noise-scale is refuted, next fork is the untried loadslip-bootstrap lever (k_loadslip_excess excluded from the current ramp) or ent-coef (its sibling arm, launched alongside).

**gate**: Rung-1 gate (same as fwd1/fwd3): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health (08-21 ruling, CORRECTED metric per fwd3 triage -- reward_walk_prog is dead under freeprog): env/walk_freeprog_score > -0.02 and rising at 2M with panel short = continue; walk_freeprog_score still flat in [-0.10,-0.05] at 2M = exploration-noise-scale hypothesis refuted, no same-recipe continuation.

**verdict**: Widening initial action noise (log-std-init -1.0 -> 0.0, std 0.37 -> 1.0, ~2.7x) does not unfreeze rung-1 discovery -- env/walk_freeprog_score stays flat in [-0.10,-0.08] the whole 2M run (never trends toward 0), gate eval det 0/6 static frozen crouch identical to fwd1-fwd3 (prog_ratio 0.00, speed 0.01m/s vs 0.05-0.06 cmd, slip/m 8.07). The wider noise did destabilize sto-mode rollouts into actual falls (5/6 term tilt_roll/tilt_pitch, one sacrificed leg) rather than finding a gait -- a new, worse failure mode (falling) layered on the same underlying freeze, not progress. Exploration-noise-scale hypothesis refuted.

