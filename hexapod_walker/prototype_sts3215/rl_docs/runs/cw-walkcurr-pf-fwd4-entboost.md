# cw-walkcurr-pf-fwd4-entboost

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T18:38:44+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: ff2ejyc5

**hypothesis**: Plain English: sibling arm to fwd4-logstd0, same 'exploration bottleneck not reward pricing' hypothesis (fwd1/fwd2/fwd3 all froze identically under three different reward fixes, walk_freeprog_score flat -0.07 the whole 2M). Instead of widening the INITIAL action std, this arm raises PPO's entropy coefficient 10x (0.001 -> 0.01) so the policy is pushed to keep sampling widely THROUGHOUT training rather than collapsing early onto the frozen/park optimum discovered near init. Exact fwd3 recipe otherwise (charge ramp intact). Prediction-if-true: walk_freeprog_score leaves its flat ~-0.07 band and trends toward/above 0 within 2M steps. Prediction-if-false: identical frozen video/flat score -- entropy-coefficient magnitude is refuted (distinct from logstd0's initial-noise-scale hypothesis: this tests whether noise DECAYS too fast rather than starting too narrow), next fork is the untried loadslip-bootstrap lever.

**gate**: Rung-1 gate (same as fwd1/fwd3): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health (08-21 ruling, CORRECTED metric per fwd3 triage -- reward_walk_prog is dead under freeprog): env/walk_freeprog_score > -0.02 and rising at 2M with panel short = continue; walk_freeprog_score still flat in [-0.10,-0.05] at 2M = entropy-coefficient hypothesis refuted, no same-recipe continuation.

