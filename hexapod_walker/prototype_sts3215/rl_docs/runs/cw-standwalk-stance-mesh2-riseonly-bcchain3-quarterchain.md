# cw-standwalk-stance-mesh2-riseonly-bcchain3-quarterchain

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T15:11:26+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**hypothesis**: Can halving the BC-anchor chain pace AGAIN unpin the deep-start (flat/bridge/rsi) rise press-up further than slowchain did? slowchain (lookahead 0.5s->0.25s, min_h 15->8mm) moved the deep-start cur_p95 median from 2.64A (pinned) to 1.85A and cut over_current terms 8/12->3/12, but stayed short of the 4/6+4/6 valid_plant PASS bar and the plants that do land are still hot (1.85-2.09A). This arm halves the pace again (lookahead 0.25->0.125s, min_h 8->4mm), same from-scratch stdanneal recipe (log-std 0->-4 anneal-frac 0.5, coef 3.0, 8M). Prediction-if-true: deep-start median keeps falling below 1.85A and/or more valid_plant deep-start successes land under 1.5A. Prediction-if-false: median plateaus near 1.85A or valid counts don't improve -- pace has a floor, the remaining heat is intrinsic to the flat-segment posture, not pursuit speed.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness as slowchain). PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode (deep-start or crouch) AND zero over_current terms. PARTIAL: deep-start cur_p95 median continues falling below slowchain's 1.85A and/or over_current terms fall below slowchain's 3/12, even without crossing the valid-count bar -- pace still has room, dose again. FAIL/FLOOR: deep-start median >=1.85A (no further improvement) or over_current terms >=3/12 -- pace has hit its floor; escalate to rung-9 (mint a mesh-native rise ref from scripted IK) or attack the flat-segment posture directly (splayed lever arms) instead of pursuit speed.

**refused_reason**: canary runs cap at 2000000 steps (asked 8000000): the question is 'is the training mechanism healthy?' - continue as --phase acquisition with --evidence.

