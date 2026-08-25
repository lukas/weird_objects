# cw-standwalk-stance-mesh2-riseonly-bcchain3-quarterchain

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T15:13:05+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: 75d9j4tg

**hypothesis**: Can halving the BC-anchor chain pace AGAIN unpin the deep-start (flat/bridge/rsi) rise press-up further than slowchain did? slowchain (lookahead 0.5s->0.25s, min_h 15->8mm) moved the deep-start cur_p95 median from 2.64A (pinned) to 1.85A and cut over_current terms 8/12->3/12, but stayed short of the 4/6+4/6 valid_plant PASS bar and the plants that do land are still hot (1.85-2.09A). This arm halves the pace again (lookahead 0.25->0.125s, min_h 8->4mm), same from-scratch stdanneal recipe (log-std 0->-4 anneal-frac 0.5, coef 3.0, 8M). Prediction-if-true: deep-start median keeps falling below 1.85A and/or more valid_plant deep-start successes land under 1.5A. Prediction-if-false: median plateaus near 1.85A or valid counts don't improve -- pace has a floor, the remaining heat is intrinsic to the flat-segment posture, not pursuit speed.

**gate**: DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness as slowchain). PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode (deep-start or crouch) AND zero over_current terms. PARTIAL: deep-start cur_p95 median continues falling below slowchain's 1.85A and/or over_current terms fall below slowchain's 3/12, even without crossing the valid-count bar -- pace still has room, dose again. FAIL/FLOOR: deep-start median >=1.85A (no further improvement) or over_current terms >=3/12 -- pace has hit its floor; escalate to rung-9 (mint a mesh-native rise ref from scripted IK) or attack the flat-segment posture directly (splayed lever arms) instead of pursuit speed.

**verdict**: FAIL - pace overshot, new degenerate FREEZE failure mode (registered prediction-if-false). Halving the anchor pace AGAIN (lookahead 0.25s->0.125s, min_h 8->4mm) does NOT continue the dose-response: DR-0 gate det 2/6 (WORSE than slowchain's 3/6) + sto 2/6 (same as slowchain's 2/6). The naive proxy metrics look great in isolation -- deep-start cur_p95 MEDIAN falls to 0.2A (vs slowchain's 1.85A) and over_current terms fall to 1/12 (vs slowchain's 3/12) -- but reading height_err_end_mm exposes why: most flat/rsi deep-start episodes now draw near-zero current (0.18-0.34A) while sitting at height_err 76-86mm, i.e. UNCHANGED from the belly start -- the policy FROZE and never attempted to rise, rather than pressing up hot. This is a cheap way to look 'cool' on the current/term metrics while doing worse on the actual goal (rising) -- exactly the new too-close-target failure mode this arm's own gate text flagged as a possible FAIL branch. One deep-start episode (det bridge) still hit the old over_current failure (2.64A, herr 66mm, partial climb then trip), so the policy is bimodal: freeze or press-up-and-trip, no third option that reaches a valid plant. Verdict: the pace dose curve is NOT monotonic -- slowchain's 1/2-pace is better than this 1/4-pace on every count that matters (det valid, and genuine progress). Re-recorded (08-25 ~16:2x): a concurrent cycle's ledger write raced and clobbered this verdict the first time it was recorded (status reverted to RUNNING, verdict cleared) -- same text, reapplied.

