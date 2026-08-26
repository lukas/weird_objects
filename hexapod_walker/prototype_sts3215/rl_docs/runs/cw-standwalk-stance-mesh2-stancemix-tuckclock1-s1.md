# cw-standwalk-stance-mesh2-stancemix-tuckclock1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-25T23:12:08+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock1

**wandb_id**: ycgendqa

**hypothesis**: Seed twin (seed 1) of cw-standwalk-stance-mesh2-stancemix-tuckclock1: does the mesh-ref + flat-time-indexed-clock port into the full stancemix stay healthy cross-seed? Same warm-start, same two-key delta vs slowchain, only the seed differs. Judged jointly with seed 0 as a 2-seed pass-rate pair.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (2M, joint 2-seed pair with seed 0): PASS if flat-pinned probe (rise_flat_frac=1.0, det+sto 6+6, DR-0) shows genuine non-freeze tuck motion in both seeds (duty>0 AND swing_count>0, no 2.64A press-up pin signature) AND hold det+sto >=5/6+5/6 zero-term AND lower >=5/6 honest (<=10mm herr) -> fund the 8M acquisition pair. FAIL if flat probe shows the tuckfloor/tuckexempt freeze or slowchain press-up pin in both seeds, or hold/lower regress below the bars -> the flat clock does not transfer into the mix warm at pinned std; next single lever is re-opened std, not more budget.

**verdict**: CANARY FAIL - MECHANISM — Seed-1 twin of stancemix-tuckclock1 (joint pair, same recipe/pinned std, seed 1 only difference). Flat-pinned pod probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, DR-0, det+sto 6+6) is a CLEAN total freeze: 12/12 episodes over_current-terminated, near-zero swing_count on every leg in the overwhelming majority (all six legs at 0 swings in 4/6 det + 4/6 sto), height_err GROWING not settling (7.0-22.9mm det, 8.9-20.1mm sto) rather than the seed-0 twin's partial tuck -- cur_max_a pinned at exactly 2.64A in all 12 episodes, the joint pair's own named 'press-up pin' FAIL signature verbatim, worse than seed 0's mixed picture. Standard mixed-start DR-0 gate: hold 6/6+6/6 zero-term (clean), lower 5/6+5/6 zero-term (one each det/sto miss ~17-20mm height_err, no term), rise/det 5/6 (1 bridge over_current term) + rise/sto 2/6 (3 term: flat x2 + bridge x1) -- flat/bridge deep starts are exactly where it pins, matching seed 0 and the predecessor slowchain FAIL pattern. Own-DR(0.2): rise collapses further (det 2/6, sto 2/6, term on flat/rsi/bridge) and lower shows two more misses -- DR hardens the same weakness. Joint call (with seed 0): CANARY FAIL - MECHANISM, both seeds show the flat-start freeze/pin the gate pre-registered as the FAIL route. Next: re-open std (log-std-init 0, was pinned -4.0), same mesh-ref+flat-clock recipe and parent otherwise -- stdreopen pair launched (registered by this canary's own FAIL route, not a budget increase).

