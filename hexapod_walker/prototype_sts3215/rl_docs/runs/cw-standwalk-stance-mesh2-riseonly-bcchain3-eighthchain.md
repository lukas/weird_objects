# cw-standwalk-stance-mesh2-riseonly-bcchain3-eighthchain

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T15:17:28+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: l9tcutsx

**hypothesis**: Bracket the pace dose-response beyond quarterchain: does pace keep paying off at 1/8 of the original (lookahead 0.5s->0.0625s, min_h 15->2mm)? slowchain (1/2 pace) moved the deep-start cur_p95 median 2.64A->1.85A and cut over_current terms 8/12->3/12; quarterchain (1/4 pace) tests the next step. This arm is the third point on the same dose curve, same from-scratch stdanneal recipe (log-std 0->-4 anneal-frac 0.5, coef 3.0, 8M), so the three together (1/2, 1/4, 1/8) read whether the benefit keeps climbing, saturates, or overshoots (too-close a lookahead could start starving the policy of a forward target entirely, a new failure mode neither sibling tested). Prediction-if-true: median keeps falling / valid counts keep climbing monotonically with pace. Prediction-if-false: eighthchain is flat vs or worse than quarterchain -- diminishing returns or a new too-close-target failure mode, pace dosing stops at 1/4-1/2.

**gate**: DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness as slowchain/quarterchain). Read together with quarterchain as one dose-response curve (1/2, 1/4, 1/8 pace) against deep-start cur_p95 median and over_current term count. PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms. PARTIAL: monotonic improvement continues past quarterchain on either metric. FAIL: flat or worse than quarterchain, or a NEW failure mode (e.g. stalling/refusing to lift because the target is too close) -- pace dosing has a floor between 1/4 and 1/8; stop dosing pace, escalate to rung-9 (mesh-native IK ref) or attack flat-segment posture directly.

**verdict**: FAIL - clear FLOOR, worse than quarterchain (bracket confirmed). Halving pace a third time (lookahead 0.125s->0.0625s, min_h 4->2mm) COLLAPSES the rung: DR-0 gate det 0/6 + sto 0/6 valid_plant -- even the crouch starts, which had been robust 6/6 across every other dose in this whole rung (parent, cont8, reanneal, slowchain, quarterchain), now fail (valid_plant=False despite low height_err 3.2-7.0mm -- fails on a DIFFERENT plant criterion, likely footprint/posture, not height). Mix of the two known bad modes: 3 over_current terms (rsi/bridge x3, still hitting 2.64A) AND 3 stalled/frozen episodes (flat + 2 rsi, cur_p95 0.18-0.31A, herr 79-86mm, i.e. never left the belly) -- the freeze failure mode quarterchain introduced gets WORSE, not better, and doesn't even trade off cleanly against the over_current mode anymore. This is decisive bracketing evidence with quarterchain: the pace dose-response is an inverted-U peaking near slowchain's 1/2-pace value, not monotonic in either direction. Pace dosing is DONE -- do not dose further in either direction.

