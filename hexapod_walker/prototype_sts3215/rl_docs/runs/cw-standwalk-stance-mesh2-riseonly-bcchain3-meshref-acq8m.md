# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T17:01:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: 08k9lmkm

**hypothesis**: Can the robot finish learning to stand up from every start posture now that it copies a demonstration its motors can actually afford? The 2M meshref canary pair (s0+s1, joint det 10/12) proved the mesh-native scripted rise ref unpins the deep starts that every old-ref arm ground into the 2.64A ceiling; the residual failures (flat-start det pinned both seeds, 2 sto rsi pins, a few valid episodes at 1.6-2.25A) look like unfinished acquisition, not a structural wall. The prior 'budget is not a lever' finding was measured against the INFEASIBLE old ref (anchor loss plateaued at a posture the mesh model cannot hold); with the feasible ref the anchor has real tracking headroom left at 2M. Exact meshref recipe (mesh-native ref, half-pace chain, anchor 3.0, stdanneal), 8M from scratch, seed 0 — the pre-registered promotion from the canary's PARTIAL branch. Prediction-if-true: rung PASS bar met, incl. flat start. Prediction-if-false: flat start still pins at 8M with anchor loss converged — the flat/belly segment of even the scripted ref needs its own treatment (tuck-phase content), a targeted next arm.

**gate**: Rung-8 rise PASS bar, DR-0 det+sto n=6+6 (same harness as slowchain/meshref): PASS = det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms AND no freeze episodes (always cross-check height_err_end_mm; current/terms alone are gameable). PARTIAL = beats the meshref canary pair (det>5/6 joint-rate or oc terms <3/12 or valid-episode current band drops below 1.5A) without the full bar — fund one targeted arm at the named residual subclass (e.g. flat-start) instead of generic budget. FAIL = no improvement over the 2M canary (det<=5/6 with same flat/rsi pins and same current band) — 8M budget refuted on the new ref too; the residual flat-start pin is structural, attack the ref's tuck segment content next.

**verdict**: PARTIAL by the gate letter, but the same-seed twin says budget is likely NOT the lever — treat as at-canary-within-noise pending s1/s2. Evidence (DR-0 gate, det+sto n=6+6): det 5/6 + sto 5/6 valid_plant, over_current 2/12, valid-episode cur_p95 median 0.81A — nominally beats the canary pair (5/6+4/6, 3/12) and trips the PARTIAL clause (oc<3/12). HOWEVER the killed duplicate meshref-8m ALSO completed the full 8M (max global_step 8,060,928 before its 17:19 kill took effect; W&B history confirms) with the exact same seed/config and scored 5/6+4/6 @ 3/12 oc = EXACTLY canary level. Same seed, same budget, different pod => the +1 sto conversion / -1 oc term separating the twins is run-to-run (GPU-nondeterminism) noise, so acq8m's edge over the 2M canary is inside that noise band. Robust findings across both 8M replicates: (1) flat det/0 is the SAME splayed press-up at 2M/8M/8M — front legs never tuck, 2.64A pin, h_err_end 24-28mm (video-confirmed); (2) successes are clean (bridge: sprawl->level six-foot plant by ~3s, tilt 0.3deg, h_err 0.2-3.6mm); (3) valid bridge/rsi episodes still press 1.58-2.15A over the 1.5A clause. Next: s1/s2 decide the grid, but with two seed-0 8M replicates at/within-noise-of canary levels, expect the pre-registered FAIL route — targeted tuck mechanism (start-mix weighting toward flat, or tuck-phase anchor dose), never more undirected budget.

