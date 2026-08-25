# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T17:01:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: 08k9lmkm

**hypothesis**: Can the robot finish learning to stand up from every start posture now that it copies a demonstration its motors can actually afford? The 2M meshref canary pair (s0+s1, joint det 10/12) proved the mesh-native scripted rise ref unpins the deep starts that every old-ref arm ground into the 2.64A ceiling; the residual failures (flat-start det pinned both seeds, 2 sto rsi pins, a few valid episodes at 1.6-2.25A) look like unfinished acquisition, not a structural wall. The prior 'budget is not a lever' finding was measured against the INFEASIBLE old ref (anchor loss plateaued at a posture the mesh model cannot hold); with the feasible ref the anchor has real tracking headroom left at 2M. Exact meshref recipe (mesh-native ref, half-pace chain, anchor 3.0, stdanneal), 8M from scratch, seed 0 — the pre-registered promotion from the canary's PARTIAL branch. Prediction-if-true: rung PASS bar met, incl. flat start. Prediction-if-false: flat start still pins at 8M with anchor loss converged — the flat/belly segment of even the scripted ref needs its own treatment (tuck-phase content), a targeted next arm.

**gate**: Rung-8 rise PASS bar, DR-0 det+sto n=6+6 (same harness as slowchain/meshref): PASS = det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms AND no freeze episodes (always cross-check height_err_end_mm; current/terms alone are gameable). PARTIAL = beats the meshref canary pair (det>5/6 joint-rate or oc terms <3/12 or valid-episode current band drops below 1.5A) without the full bar — fund one targeted arm at the named residual subclass (e.g. flat-start) instead of generic budget. FAIL = no improvement over the 2M canary (det<=5/6 with same flat/rsi pins and same current band) — 8M budget refuted on the new ref too; the residual flat-start pin is structural, attack the ref's tuck segment content next.

