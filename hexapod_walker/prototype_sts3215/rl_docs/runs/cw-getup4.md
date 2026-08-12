# cw-getup4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T12:10:26+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-getup3-c2

**wandb_id**: v6gt7wfl

**hardware_ready**: False

**hypothesis**: Deepen the stand basin so the anchored policy has a reason to finish the climb. Root-cause chain (required for a pricing change): BEHAVIOR — cw-getup3-c2 plateaued at a partial-load 4-leg stand for 8M steps (env/getup_S 0.17-0.21, feet_loaded stuck ~2.8/6, front legs held up) while height/footprint kept improving. INCENTIVE — at that plateau the S-cubed hold income pays ~0.009/tick, and the last pipeline step (lower + load the final two legs, briefly risking a tilt) leads to income the policy never samples far enough to see. PRICING — getup_k_hold=0.8 was sized to make cheats earn scraps, not to make the summit worth a risky climb; the gap between the 4-leg plateau (~0.03/tick at k=2.5) and a true six-foot stand (~0.9/tick at k=2.5, S~0.7) becomes ~30x with this change. ONE variable vs cw-getup3-c2: reward.getup_k_hold 0.8 -> 2.5, warm-started from the c2 plateau checkpoint so the test is exactly whether deeper stand income moves a policy already parked at the plateau. Cheat ordering is unchanged by construction (all hold pay scales by the same factor; the S-cubed shape and every fade stay; MDP_PREFLIGHT GETUP bank runs on defaults and stays green). Prediction-if-true: feet_loaded climbs off 2.8 toward 5+ and env/getup_S breaks 0.3 within 2M, hold income becomes a visible line. Prediction-if-false: S stays in the 0.21 band — pricing depth is NOT the binding constraint and the next lever is exploration/anchor-side (per-start-kind BC anchoring or start-mix reweighting), i.e. CODE, not another coefficient. Strongest alternative: higher hold pay entrenches the 4-leg pose harder (it also scales that pose by 3x) — visible as S flat with reward_getup_hold slightly up.

**gate**: PASS if by 2M steps env/getup_S ends >0.30 with feet_loaded trending above 4/6 and video shows a six-foot supported stand (front legs down and loaded) with no flag-leg/stilt signature; FAIL if S remains in the 0.17-0.23 plateau band or feet_loaded stays pinned below 3.5/6.

**verdict**: FAIL per pre-registered scalar branch: getup_k_hold 0.8->2.5 (30x summit/plateau income gap) moved nothing in 2M — env/getup_S ends 0.178 (plateau band 0.17-0.23), feet_loaded 2.63-2.67/6 (pinned <3.5). Pricing depth is NOT the binding constraint on the 4-leg getup plateau; per prediction-if-false the next lever is anchor/exploration-side CODE (per-start-kind BC anchoring / start-mix reweighting), not another coefficient. Getup sub-line stays deprioritized vs the passing footlow2 stance lineage.

