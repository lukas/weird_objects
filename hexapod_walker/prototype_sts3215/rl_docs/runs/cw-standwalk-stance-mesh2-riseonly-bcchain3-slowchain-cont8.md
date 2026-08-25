# cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain-cont8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T15:27:35+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: 5bx2x0n3

**hypothesis**: Given a pace that already helped (slowchain's half-pace, 1/2 lookahead), does more budget finish the job -- mirroring cont8's test, but on the ONE lever that actually moved the deep-start metric? cont8 (more budget, UNCHANGED full pace) FAILED to unpin deep-start current (median stayed 2.64A, anchor loss already at the plateau). This arm continues 8M more steps from the slowchain checkpoint itself (already at cur median 1.85A, det 3/6), std pinned at the annealed -4 (no re-anneal, pure low-noise refinement), same half-pace anchor cfg unchanged. Prediction-if-true: deep-start median keeps falling below 1.85A and/or valid_plant counts climb past 3/6+2/6 -- budget helps once the structural lever (pace) is right. Prediction-if-false: median/valid counts plateau near slowchain's own numbers -- slowchain's 8M was already sufficient for this pace, and the remaining heat needs a further pace dose (quarterchain/eighthchain) or a new mechanism, not more of the same.

**gate**: DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness). PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms. PARTIAL: deep-start median falls below slowchain's 1.85A and/or valid counts climb past 3/6 det + 2/6 sto, even short of the full bar. FAIL: median/valid counts plateau at or worse than slowchain's own numbers -- budget doesn't help at this pace either; the pace-dose arms (quarterchain/eighthchain) are the live lever, not budget.

**verdict**: FAIL - budget does not help even on the ONE working pace lever (mirrors cont8's null result, now confirmed on both pace settings). 8M more steps from the slowchain checkpoint itself (same half-pace anchor cfg, std pinned -4, no re-anneal) makes the gate NET WORSE, not better: DR-0 det 2/6 (down from slowchain's 3/6) + sto 2/6 (same as slowchain's 2/6) = 4/12 valid vs slowchain's own 5/12. over_current terms rise back to 7/12 (vs slowchain's 3/12) -- notably the flat start, which was slowchain's coolest deep-start (0.35A, no term), now terminates over_current (16.9A->2.64A, 16.9mm herr) after more training: continued optimization pushed the policy toward MORE aggressive/hot attempts on some deep starts at the cost of others, not toward a uniformly cooler solution. Combined with cont8's identical null result on the unchanged-pace recipe, this settles the budget question for this whole rung: NEITHER pace setting benefits from more low-noise refinement once the anchor has converged (bc_anchor_loss_rise plateaus ~0.05 regardless) -- budget is not a lever here in either direction. slowchain's own 8M checkpoint remains the best asset from this campaign; further budget on it should not be funded again without a structural change first.

