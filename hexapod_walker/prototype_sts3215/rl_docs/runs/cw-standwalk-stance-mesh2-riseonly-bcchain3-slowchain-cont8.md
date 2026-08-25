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

**verdict**: More budget on top of the pace-halved slowchain checkpoint does NOT extend the pace win -- deep-start current stays pinned near the 2.64A ceiling and valid-plant count REGRESSES vs the parent. Evidence: DR-0 gate rise det 2/6 valid_plant (bridge1+crouch1; parent slowchain was 3/6 -- lost the bridge5 pass, now over_current at height_err 70mm mid-rise) with deep-start (flat/bridge/rsi) cur_p95 pinned 2.23-2.64A on 4/5 deep starts vs parent's spread 0.29-2.09A; sto 2/6 unchanged (crouch-only). own-DR(0.2) det 2/6, sto REGRESSED 2/6->0/6, Imax 2.64-2.66A. Video: crouch/bridge starts that do plant look level and clean, but flat/rsi deep starts terminate over_current mid-rise before reaching height. Reward flat (~-46 both ends, matches eighthchain sibling, no net training signal). This is the pre-registered FAIL branch (plateau/worse than slowchain's own numbers) -- same conclusion as siblings stdanneal-cont8/stdanneal-reanneal: budget and noise-re-injection do not move this metric, only the pace-halving lever (slowchain itself) did. Next: read jointly with eighthchain (1/8 pace) and quarterchain (1/4 pace, another cycle's) as the 3-point dose curve; if eighthchain also fails to beat quarterchain, pace dosing floors between 1/2 and 1/4 and stage-1 rise should escalate past pace-dosing (mesh-native IK ref, rung-9) rather than fund more of this exact lever.

