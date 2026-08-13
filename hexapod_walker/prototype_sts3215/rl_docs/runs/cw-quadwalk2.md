# cw-quadwalk2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-13T13:27:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-quadwalk1

**hypothesis**: Teach the four-leg-stance robot to actually WALK on its rear four legs with the front pair lifted as hands, instead of just walking normally on all six. cw-quadwalk1 (same recipe) hit the pre-registered failure class (b): it never even attempted to lift the fronts (fronts_lifted 0/6 det+sto, gait_valid 0/6) because ordinary six-leg walking already pays well and lifting earns only a small side bonus. This arm tests the hypothesis's own prescribed fix for class (b): reprice — triple the lift-leg income (k_quad_clear 1.5->4.5, k_quad_plant 1.0->3.0, same ratio) so that actually lifting and balancing on four is worth the risk relative to the safe six-leg default. Same base (cw-quad-hold2), same mix (quadwalk=0.7/quad=0.3), same 2M discovery budget, same grace/thresholds -- ONE lever: the size of the reward for doing the quad thing. Prediction-if-true: det video by 2M shows at least some episodes with fronts genuinely lifted and rear-four stepping attempts (fronts_lifted improves off 0/6, even if imperfect). Prediction-if-false: fronts_lifted stays 0/6 -- income scaling isn't the lever (the policy isn't exploring lifting at all under this entropy/warm-start), and the next lever must be an explicit CODE penalty for front-leg ground contact during quadwalk (or higher exploration), not further coefficient scaling. This is the SECOND arm on this failure class; per research-rules 'two misses in a row = change the hypothesis, not the step count', a second 0/6 here closes the pure-reprice lever.

**gate**: Harness quadwalk det 6 eps @2M: >=4/6 eps net forward displacement >= +0.05 m AND fronts lifted (post-grace tail lift duty <0.15 both lift legs) AND no episode net backward < -0.02 m AND 0 falls; det video shows all four support legs cycling contact/swing (no pinned/dragged mid leg, no outrigger). Retention: quad-hold mode survived_frac 1.0, fronts lifted, planar creep <=0.10 m/15s. Any known cheat dominating video (freeze, fronts-down gait, backward shuffle) = STOP, no continuation. Note: passing this discovery gate does not make it the bank reference -- that needs the full rl_docs/tracks/quad/QUADWALK_REF_GATE.md.

**refused_reason**: hexapod-mjx-train-0 code marker 30ca4804fc53a39780a51b96d45d6b041350b8fe-dirty != local HEAD 30ca4804fc53a39780a51b96d45d6b041350b8fe. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

