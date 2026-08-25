# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-acq8m-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-25T17:26:46+00:00

**pod**: hexapod-mjx-train-4

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: i19z5h68

**hypothesis**: Can the robot finish learning to stand up from every start posture now that it copies a demonstration its motors can actually afford? The 2M meshref canary pair (s0+s1, joint det 10/12) proved the mesh-native scripted rise ref unpins the deep starts that every old-ref arm ground into the 2.64A ceiling; the residual failures (flat-start det pinned both seeds, 2 sto rsi pins, a few valid episodes at 1.6-2.25A) look like unfinished acquisition, not a structural wall. The prior 'budget is not a lever' finding was measured against the INFEASIBLE old ref; with the feasible ref the anchor has real tracking headroom left at 2M. Exact meshref recipe (mesh-native ref, half-pace chain, anchor 3.0, stdanneal), 8M from scratch, seed 0 — the pre-registered promotion from the canary's PARTIAL branch, forming a 3-seed 8M cohort with the parallel cycle's -8m-s1/-8m-s2. Prediction-if-true: rung PASS bar met incl. flat start. Prediction-if-false: flat start still pins at 8M with anchor loss converged — the ref's tuck/belly segment needs its own treatment next.

**gate**: Rung-8 rise PASS bar, DR-0 det+sto n=6+6 (same harness as slowchain/meshref): PASS = det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms AND no freeze episodes (always cross-check height_err_end_mm; current/terms alone are gameable). PARTIAL = beats the meshref canary pair (det>5/6 joint-rate or oc terms <3/12 or valid-episode current band drops below 1.5A) without the full bar — fund one targeted arm at the named residual subclass (e.g. flat-start) instead of generic budget. FAIL = no improvement over the 2M canary (det<=5/6, same flat/rsi pins, same current band) — 8M budget refuted on the new ref too; the residual flat-start pin is structural, attack the ref's tuck segment content next. Read jointly with -8m-s1/-8m-s2 as a seed pass-rate.

**verdict**: KILLED - DUPLICATE (this cycle's own error, ~5 min in, no information lost): launched as a DEAD-retry of -acq8m based on a misread — acq8m had not died, it had FINISHED its full 8M budget in ~8 min (8,060,928 steps, clean W&B sync 17:15, 08k9lmkm) before -fullpace2 legitimately took the freed train-0. Identical spec+seed to the completed acq8m, so this run answers nothing. acq8m is the 3-seed 8M grid's seed-0; triage it normally.

