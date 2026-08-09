# cw-walk-comshift30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T17:12:41+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**hypothesis**: OPERATOR WISHLIST 11, second half (payload 'mass pushed ASYMMETRICALLY'): payload50 (in triage) tests symmetric extra mass; real payloads (battery pack, camera, claw cargo) sit OFF-CENTER. ISOLATED axis via dr overrides: dr-scale 0.0 with ONLY dr.com_offset_m=0.03 randomized (30mm chassis CoM shift each of x/y, 2.5x the standard 12mm envelope), one variable off the no-DR champion. Plain: walk straight while carrying weight on one side. Prediction-if-true: gait absorbs CoM shift (own-cfg gv 12/12, 0 term, det med fwd >=1.2m @30s, heading holds) and DR0 nominal retention holds - asymmetric payload becomes a keeper rung and a demo capability. Prediction-if-false: offset mass induces veer/tilt terminations or per-leg loading collapses gait_valid on the heavy side - asymmetric load needs explicit adaptation, not exposure. Strongest alternative: passes by crabbing/slowing (prog low, track_err high) - frames + heading error will show it.

**gate**: own-cfg harness at --dr-scale 0.0 + dr.com_offset_m=0.03, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**refused_reason**: hexapod-mjx-train-11 code marker a05d6a76a1bfe550fa37ca0b21381658579d85b3 != local HEAD dfa6fb996a3be84ea60f9edca249e6b6867aefb9. Sync first: snapshot.sh --sync hexapod-mjx-train-11 (and snapshot/commit before that if the tree is dirty).

