# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings-curric1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY PASS (own scope) - JOINT DIVERGENCE

**created**: 2026-08-27T06:26:31+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1-s1

**wandb_id**: 93z0cisc

**hypothesis**: Seed-1 twin of anchor2-headings-curric1 (see that entry for the full hypothesis): does ramping the heading cone 0->0.7854 over the first 1.2M of a 2M continuation (instead of opening it fully at step 0) let the walk core learn to steer, tested on the second seed off anchor2-s1's own leak-fixed checkpoint.

**gate**: Same panel and WALK-SURVIVES / DIRECTION-LEARNS-FULL clauses as anchor2-headings-curric1; JOINT call with that seed-0 twin per that entry's promote/close branches.

**verdict**: CANARY PASS (own scope) - JOINT DIVERGENCE: seed1 half of the anchor2-headings-curric1 joint 2-seed call. UNLIKE seed0 (verdicted CANARY FAIL - MECHANISM: total anchor4-class 2-leg-sacrifice freeze, det gait_valid 0/6, sac=[4,5]), this seed's walk is HEALTHY -- det gait_valid 6/6 both DR-0 and own-DR, 0 sacrificed legs, prog med 0.29-0.31 (clears the >=0.2 bar), slip med 4.7-5.6 (in the healthy band), video confirms all six legs cycling normally (walk_det_0_sheet.png), no freeze. Direction-learning is weak (dir_err mean 52-55deg, wrong_direction_frac 0.2-0.29, direction_valid_frac ~0.89) but that is the SAME fragile-partial pattern already logged for headings1 (Next -1.85), not a new catastrophe. Per this arm's pre-registered joint-close decision rule (STATUS Next -1.95): seed1 healthy-like-headings1 + seed0 anchor4-class catastrophe = SEED-LEVEL NOISE in an already-fragile coef=3.0 dual-core recipe, not evidence the ramp mechanism itself destabilizes walk. Joint call: do not fund a ramp-vs-instant heading-exposure mechanism arm on this recipe; the earlier 'gentler ramp is worse' read on seed0 alone does not replicate. No further ramp ramp-vs-instant work; campaign should target the shared critic/trunk per anchor6b's own separate closure.

