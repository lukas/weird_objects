# cw-standwalk-stance-mesh2-stancemix-tuckclock1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T23:12:08+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock1

**wandb_id**: ycgendqa

**hypothesis**: Seed twin (seed 1) of cw-standwalk-stance-mesh2-stancemix-tuckclock1: does the mesh-ref + flat-time-indexed-clock port into the full stancemix stay healthy cross-seed? Same warm-start, same two-key delta vs slowchain, only the seed differs. Judged jointly with seed 0 as a 2-seed pass-rate pair.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (2M, joint 2-seed pair with seed 0): PASS if flat-pinned probe (rise_flat_frac=1.0, det+sto 6+6, DR-0) shows genuine non-freeze tuck motion in both seeds (duty>0 AND swing_count>0, no 2.64A press-up pin signature) AND hold det+sto >=5/6+5/6 zero-term AND lower >=5/6 honest (<=10mm herr) -> fund the 8M acquisition pair. FAIL if flat probe shows the tuckfloor/tuckexempt freeze or slowchain press-up pin in both seeds, or hold/lower regress below the bars -> the flat clock does not transfer into the mix warm at pinned std; next single lever is re-opened std, not more budget.

