# cw-walkcurr-pf-fwd6-rscale50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:41:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: sms76sd6

**hypothesis**: Plain English: dose sibling of fwd6-rscale10 -- shrink every reward gain 50x (not 10x) so PPO's value-loss gradients (the thing crushing all policy updates via SB3's single global grad-norm clip; see rscale10's hypothesis for the full 8/8-run evidence chain) become unambiguously small: expected returns |~50| instead of |~2500|, value_loss O(1-10) instead of 400-2000+. Same single lever, same exact fwd3-chargeramp recipe, all 15 active gains scaled together; ranking bank green at x0.02 (test_walkcurr_pf_scaled_*, snapshot exp/walkcurr-fwd6-rscale-batch). If-true: clip_fraction healthy past 50% of run, std off init, walk_freeprog_score leaves the [-0.10,-0.05] band. If-false while rscale10 succeeds: 10x was already sufficient (adopt it). If BOTH freeze with healthy clip_fraction: optimizer-crush refuted, escalate to RND/rung-0. Strongest alternative: at x0.02 per-step reward differences may sink toward the value net's noise floor -- if rscale50 shows healthy clip_fraction but NO freeprog trend while rscale10 trends, the dose overshot.

**gate**: Rung-1 gate (same as fwd1-fwd5): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Mechanism-health at 2M: clip_fraction still >0.02 in the last quarter AND std off init = optimizer fix worked mechanically (continue per 08-21 ruling if freeprog_score also rising); clip_fraction collapsed to 0 again = optimizer-crush hypothesis refuted at this dose.

