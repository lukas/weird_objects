# cw-walkcurr-pf-fwd6-rscale50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T20:41:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: sms76sd6

**hypothesis**: Plain English: dose sibling of fwd6-rscale10 -- shrink every reward gain 50x (not 10x) so PPO's value-loss gradients (the thing crushing all policy updates via SB3's single global grad-norm clip; see rscale10's hypothesis for the full 8/8-run evidence chain) become unambiguously small: expected returns |~50| instead of |~2500|, value_loss O(1-10) instead of 400-2000+. Same single lever, same exact fwd3-chargeramp recipe, all 15 active gains scaled together; ranking bank green at x0.02 (test_walkcurr_pf_scaled_*, snapshot exp/walkcurr-fwd6-rscale-batch). If-true: clip_fraction healthy past 50% of run, std off init, walk_freeprog_score leaves the [-0.10,-0.05] band. If-false while rscale10 succeeds: 10x was already sufficient (adopt it). If BOTH freeze with healthy clip_fraction: optimizer-crush refuted, escalate to RND/rung-0. Strongest alternative: at x0.02 per-step reward differences may sink toward the value net's noise floor -- if rscale50 shows healthy clip_fraction but NO freeprog trend while rscale10 trends, the dose overshot.

**gate**: Rung-1 gate (same as fwd1-fwd5): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Mechanism-health at 2M: clip_fraction still >0.02 in the last quarter AND std off init = optimizer fix worked mechanically (continue per 08-21 ruling if freeprog_score also rising); clip_fraction collapsed to 0 again = optimizer-crush hypothesis refuted at this dose.

**verdict**: PARTIAL -- strongest mechanical result of the fwd6 wave: x0.02 global reward scale keeps PPO's optimizer alive from step ~200k AND walk_freeprog_score rises the whole run, but behavior hasn't crossed into stepping at 2M. Evidence (cached W&B + gate eval): clip_fraction healthy every quarter (Q1-Q4 means 0.024/0.043/0.050/0.049, last 0.083 -- never the exactly-0 collapse of the 9 frozen rung-1 arms), std creeping off init (0.3678->0.3729), walk_freeprog_score monotonic by quarter -0.074/-0.045/-0.023/-0.015 (start -0.103) -- the FIRST rung-1 arm to leave the [-0.10,-0.05] dead band and trend toward 0. Gate eval still 0/6 det (fwd 0.01m, slip 6.5, static splayed crouch on video, all six legs non-cycling; sto slip 16-26, 1 tilt_pitch term). Dose read vs sibling: rscale10 hovered flat at -0.055 with a Q2 clip near-collapse (0.003) -- the 'x0.02 overshoots into the value-net noise floor' alternative in this run's own hypothesis is REFUTED; the stronger scale is the one trending. Why continue: this is the gate's own pre-registered branch (clip_fraction >0.02 last quarter + freeprog rising = 08-21 continue), and the score is approaching the zero crossing where forward progress starts paying. Next: cw-walkcurr-pf-fwd6-rscale50-cont1, +4M byte-identical warm-start (this cycle); if freeprog plateaus below 0 across the continuation with clip healthy, crush-fix was necessary-but-insufficient -> pre-registered RND/rung-0 escalation.

