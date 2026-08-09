# cw-walk-stiffvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T17:12:55+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**hypothesis**: OPERATOR WISHLIST 13b via c49 dr.<field> overrides, next isolated axis after mass/latency/torque/friction: CONTACT COMPLIANCE (solref timeconst). The real robot's rubber feet on unknown floors are the least-calibrated contact quantity we have, and contact is the ruled root of the skating defect - exposure to varied stiffness is the cheapest transfer hedge while the operator's hardware calibration waits. ISOLATED: dr-scale 0.0 with ONLY dr.contact_stiff_scale=0.5,2.5 randomized (wider than the 0.7-2.0 standard envelope), one variable off the no-DR champion. Prediction-if-true: gait absorbs compliance variation (own-cfg gv 12/12, 0 term, det med fwd >=1.2m @30s) and DR0 nominal retention holds - compliance becomes a keeper robustness rung. Prediction-if-false: soft/stiff contact breaks the paddle gait (terminations, prog craters, or slip explodes at the soft end) - the paddle transport is compliance-fragile, strengthening the case that corrected contact physics will need a retrain (readiness review P0). Strongest alternative: policy passes by slowing/parking on soft contact - frames + prog_ratio will show it.

**gate**: own-cfg harness at --dr-scale 0.0 + dr.contact_stiff_scale=0.5,2.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**refused_reason**: hexapod-mjx-train-7 code marker a05d6a76a1bfe550fa37ca0b21381658579d85b3 != local HEAD dfa6fb996a3be84ea60f9edca249e6b6867aefb9. Sync first: snapshot.sh --sync hexapod-mjx-train-7 (and snapshot/commit before that if the tree is dirty).

