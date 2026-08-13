# cw-quadwalk3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-13T14:15:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-quadwalk2

**wandb_id**: ar9ez7f7

**hardware_ready**: no

**hypothesis**: Make walking on six legs actually COST money so the robot finally lifts its front pair and walks on four. Two arms (cw-quadwalk1, cw-quadwalk2) proved paying MORE for lifted fronts is not enough: even 3x lift income only cut front-leg ground contact from ~100% to 62%/32% of the episode — the warm-started six-leg walk pays too well to abandon for a bonus. This arm keeps everything from quadwalk2 (same base cw-quad-hold2, mix quadwalk=0.7/quad=0.3, 3x income, 2M discovery) and adds ONE new lever, the new code term reward.k_quad_lift_contact=3.0: a per-tick charge on any commanded lift leg touching the ground after the settle window. Semantics bank proves the separation: six-leg walk charged -547/ep, fronts-down drag -882/ep, honest tucked-fronts gait exactly 0. Prediction-if-true: det video by 2M shows fronts genuinely held up (tail lift duty <0.15) with rear-four stepping attempts, even if translation is clumsy. Prediction-if-false: fronts stay down eating the charge, or the policy collapses to a freeze/park (charge avoidable by stillness — freeze is already a pre-registered one-line STOP), meaning exploration/entropy is the real blocker, not pricing structure.

**gate**: Harness quadwalk det 6 eps @2M: >=4/6 eps net forward displacement >= +0.05 m AND fronts lifted (post-grace tail lift duty <0.15 both lift legs) AND no episode net backward < -0.02 m AND 0 falls; det video shows all four support legs cycling contact/swing. Retention: quad-hold survived_frac 1.0, fronts lifted, planar creep <=0.10 m/15s. Any known cheat dominating video (freeze/park, fronts-down gait, backward shuffle) = STOP, no continuation. Passing discovery does NOT make it the bank reference (needs full QUADWALK_REF_GATE.md).

**verdict**: STOP — pre-registered cheat (b) fronts-down six-leg gait, THIRD occurrence, this time with the charge live: env/reward_quad_lift_contact fired at -1.54/tick (~-575/ep, matches the bank's -547) yet tail lift duty stayed 0.58/0.36 med (bar <0.15; quadwalk2 without the charge: 0.62/0.32), fronts_lifted 0/12 det+sto, fwd med 0.60m, 0 falls, roll tail 0.7-0.9deg clean, slip/m ~1.96, drag ~1.05m. The policy eats a charge worth ~40% of episode return rather than leave the warm-started six-leg walk basin -> PRICING (income AND charge) exhausted as a lever class for quadwalk; per this arm's own prediction-if-false the blocker is EXPLORATION from the six-foot plant spawn (quad-hold proves lifting is learnable from plant under a hold command; under a walk command the six-leg basin dominates). Secondary: quad-hold retention survived 12/12 but creep 0.41-0.43m/15s vs the 0.10m gate bar — k_quad_still=1.0 did not fix stance creep either. Next lever: quad-stance episode spawn (RSI-style start in the fronts-tucked stance), CODE.

