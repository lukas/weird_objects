# cw-stance-posture

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T22:07:54+00:00

**pod**: hexapod-sweep-s4

**steps**: 4000000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**wandb_id**: q7797l60

**hypothesis**: Stance champion's flag-leg endings (posture-strict baseline: lower 0/12, rise 5/12, hold 12/12) exist because load concentration and support-polygon shrinkage are unpriced (static-hold pricing measured CORRECT cycle 12; flagged leg 0.24A < supporting leg 0.28-0.44A; linear charge distribution-blind). Pricing the physics (k_support_margin=0.3 + k_load_even=1.5, GLOBAL in no-walk joint_goal) gives a dense local gradient that plants hoisted legs. If-true: lower end_posture >=5/6 det+sto and rise crouch/bridge endings plant legs 2/4, heights retained. If-false: heights retained but end-posture unchanged (pricing insufficient at audited scale -> exploration question, NOT coefficient iteration) or canary regression (terms fight rise kernel). Strongest alternative: flag-leg ending is a local optimum unreachable at inherited std ~0.29 (deliberate warm refinement - posture is a dense local gradient, not a basin escape). Probe probe-posture-price2 PASSED (150k, canaries protected, parts in audited band).

**gate**: posture-strict harness @ DR 1.0, 6 eps/mode det+sto: lower end_posture >=5/6 both passes AND rise posture-strict sto >=4/6 AND hold 6/6 AND crown-jewel heights intact (rise/lower height-only >=5/6 both passes)

**verdict**: FAIL headline (cycle 13): lower end-posture 0/6 det + 0/6 sto (gate >=5/6 both) with heights 12/12 <=6.2mm; legs 0/2 pulled to 58-91mm (parent >100-200) but leg 4 vertical flag UNMOVED 229-264mm. rise sto 4/6 (=gate), det crouch 0/6 both draws. hold det 5/6 (24mm marginal), sto 6/6. raise height-only 11/12 best-ever, posture 0/12. Root cause: load_even/support_margin have zero gradient on an UNLOADED airborne leg; stance_clearance excludes rise/lower/raise; std 0.195 cannot bridge. Pre-registered if-false branch: exploration, not coefficients. Champion unchanged. ckpt md5 7c2ab2f5.

