# cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-26T00:15:50+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock1-s1

**wandb_id**: 77l3258u

**hypothesis**: Seed twin (seed 1) of stdreopen: does the proven flat-start rise recipe transfer into the full mix once exploration noise is restored, cross-seed? Same single lever as the seed-0 twin (log-std-init 0, was pinned -4.0), same warm-start parent, judged jointly as a pass-rate pair.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (2M, joint 2-seed pair with seed 0): PASS if flat-pinned probe (rise_flat_frac=1.0, det+sto 6+6, DR-0) shows genuine non-freeze tuck motion in both seeds (duty>0 AND swing_count>0, no 2.64A press-up pin signature in the MAJORITY of episodes) AND hold det+sto >=5/6+5/6 zero-term AND lower >=5/6 honest (<=10mm herr) -> fund an 8M mix acquisition pair (mirroring the isolated recipe's own 2M canary -> 8M acquisition arc). FAIL if flat probe still shows the freeze/press-up pin in both seeds -> reopening std at 2M is not enough on its own; the next lever is budget (fund 8M directly on this exact recipe, matching the isolated arm's own required budget) not another config change.

**verdict**: CANARY FAIL - MECHANISM — Seed-1 twin of stdreopen (joint pair). Flat probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, DR-0, det+sto 6+6): a CLEAN total freeze, matching seed 0 -- 12/12 over_current-terminated, cur_max_a pinned 2.64A every episode, swing_count=0 on EVERY leg in EVERY one of the 12 episodes (zero exceptions), height_err growing 8.6-25.1mm. Worse than the pinned-std parent (-s1) whose own flat probe was also a total freeze, so this seed shows the SAME direction as seed 0: reopening std did not create tuck motion, it just kept (rather than reduced) the complete freeze. Standard mixed-start DR-0 gate: hold 6/6+6/6 zero-term (unaffected), lower 6/6+5/6 zero-term (unaffected), rise/det 5/6 (1 term, better letter than parent's 5/6-with-1-term -- comparable) + rise/sto 2/6 (4 term, worse than parent's 2/6-with-3-term). Own-DR(0.2): rise/det 4/6 (2 term), rise/sto 4/6 (2 term), lower 4/6 (1 term) -- similar softening to seed 0. JOINT CALL (with seed 0): both seeds still show the flat-start freeze/press-up pin -- CANARY FAIL - MECHANISM per the pre-registered gate. Cross-seed the direction is CONSISTENT: reopening std at 2M in this warm-started mix does not produce the hoped-for exploration effect; if anything it removes the small amount of asymmetric leg motion the pinned-std parent had. DIG-IN flagged jointly with seed 0's cycle rather than auto-funding the registered '8M directly' next step -- the isolated recipe that solved flat-start rise was from-scratch at every budget, not warm-started, so committing 8M x 2 seeds on this exact warm-started lineage needs a considered call, not mechanical follow-through.

