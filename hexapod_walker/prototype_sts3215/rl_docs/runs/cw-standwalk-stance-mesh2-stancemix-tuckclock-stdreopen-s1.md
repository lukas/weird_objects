# cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T00:15:50+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock1-s1

**wandb_id**: 77l3258u

**hypothesis**: Seed twin (seed 1) of stdreopen: does the proven flat-start rise recipe transfer into the full mix once exploration noise is restored, cross-seed? Same single lever as the seed-0 twin (log-std-init 0, was pinned -4.0), same warm-start parent, judged jointly as a pass-rate pair.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (2M, joint 2-seed pair with seed 0): PASS if flat-pinned probe (rise_flat_frac=1.0, det+sto 6+6, DR-0) shows genuine non-freeze tuck motion in both seeds (duty>0 AND swing_count>0, no 2.64A press-up pin signature in the MAJORITY of episodes) AND hold det+sto >=5/6+5/6 zero-term AND lower >=5/6 honest (<=10mm herr) -> fund an 8M mix acquisition pair (mirroring the isolated recipe's own 2M canary -> 8M acquisition arc). FAIL if flat probe still shows the freeze/press-up pin in both seeds -> reopening std at 2M is not enough on its own; the next lever is budget (fund 8M directly on this exact recipe, matching the isolated arm's own required budget) not another config change.

