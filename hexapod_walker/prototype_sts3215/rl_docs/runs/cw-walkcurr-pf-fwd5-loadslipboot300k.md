# cw-walkcurr-pf-fwd5-loadslipboot300k

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T19:47:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**hypothesis**: Plain English: does briefly going easy on the anti-skate/slip charge (not just the park/idle/heading charges fwd3's ramp already softens) let a from-scratch policy discover forward walking? fwd1/fwd2/fwd3/fwd4x2 (5 straight rung-1 arms: plain, swing-income x2, charge-ramp, wider-init-noise x2, higher-entropy) all froze in an identical static crouch with env/walk_freeprog_score flat/negative ~[-0.10,-0.05] the whole 2M budget -- closing both the income-side and the walk-charge-ramp's three discovery-friction charges as fixes. The one charge the ramp deliberately excludes is k_loadslip_excess (bank finding: loosening it together with the others at 0.15 let skate/shuffle beat every honest behavior). This arm adds a SEPARATE bootstrap (new reward.walk_loadslip_bootstrap_steps mechanism, smoke-tested this cycle: wires end-to-end, no crash) that softens ONLY k_loadslip_excess to 0.65x for the first 300k steps (15% of budget), annealing back to full dose -- bank-proven at its own floor this cycle (test_walkcurr_loadslip_bootstrap_min_ranking_holds: park/stall stay strictly above every wrong-way gait with margin ~37, skate charge cut ~30%). Keeps fwd3's charge ramp (harmless, bank-proven) and fresh init. If-true: walk_freeprog_score leaves its flat band and trends toward/above 0 within the bootstrap window or shortly after, gate eval shows nonzero prog_ratio and six-leg stepping on video. If-false (freezes again): the loadslip charge specifically was not the binding constraint either -- BOTH init/noise AND loadslip-bootstrap are now refuted, matching the track's own escalation rule (STATUS.md 'Next'): the prior-free MLP recipe is refuted at this budget, escalate to a dig-in (rung-0 curriculum / BC-kickstart / architecture) before any further reward-magnitude variant.

**gate**: Rung-1 gate (same as fwd1/fwd3/fwd4): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health: env/walk_freeprog_score > -0.02 and rising at 2M = continue/harden; still flat in [-0.10,-0.05] at 2M = loadslip-bootstrap hypothesis refuted (joint with fwd4's noise-scale refutation) -> escalate to dig-in (rung-0/BC-kickstart/architecture), no further same-class reward-magnitude arm.

**verdict**: Softening ONLY k_loadslip_excess (to 0.65x, bank-proven floor) for the first 300k of 2M steps does NOT unfreeze rung-1 discovery either -- env/walk_freeprog_score stays flat in the same [-0.10,-0.06] FAIL band the whole run (never approaches 0), gate eval is 0/6 det + 0/6 sto, prog_ratio 0.00, det speed 0.004 m/s vs 0.05-0.06 cmd, slip/m 8-41 (cap 3.0), contact sheet shows the same static splayed crouch repeated across all 10 frames as fwd1-fwd4. This was the pre-registered 'untried lever' (STATUS.md Next); its refutation means the loadslip charge specifically was not the binding constraint. Reward fell monotonically (-89/-481/-1122/-1948 by quarter) as the ramped/annealed charges grew against a frozen policy -- not undertraining, matches the other 4 FAILs' signature exactly.

