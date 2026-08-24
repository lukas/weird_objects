# cw-walkcurr-pf-fwd6-hgt2-pdw05-pdx15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-24T00:08:24+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-hgt2

**hypothesis**: Plain English: dose sibling of hgt2-pdw05 -- same confound fix (goal.park_duty_window_s 2.0->0.5, park-duty charge now fires before the 2.0s termination grace expires) PLUS 1.5x the per-leg contact-duty charge (reward.k_park_duty 0.08->0.12) to test whether the de-confounded charge merely needs to fire vs. needs to fire HARDER to out-price the frozen tripod/belly-sit stances every RND and height-gate arm has converged to so far. Bank-proven this cycle at exactly this dose (test_walkcurr_pf_hgt_ tight_pdw05_pdx1p5, 16/16 green -- note 3x (0.24) was tried first and REFUTED at the bank stage: it over-taxes the honest 'park' stand-still behavior itself (all 6 legs near-duty-1.0 pays the >0.9 branch on every leg), pushing park_gated BELOW belly_sit_gated and inverting the ranking the mechanism is supposed to protect; 1.5x is the largest dose that stays bank-legal). Prediction-if-true: freeprog crosses toward/past 0 faster or further than the plain pdw05 sibling. Prediction-if-tied: dose doesn't matter once the confound is fixed, the plain pdw05 recipe becomes the operating point. Prediction-if-both-fail identically: park_duty-class charges (any bank-legal dose) are insufficient regardless of the confound, escalate to a direct minimum-total-foot-contact charge (new mechanism) before BC-kickstart.

**gate**: Same rung-1 gate as hgt2-pdw05; read the pair jointly -- whichever dose (or neither) crosses walk_freeprog_score past 0 with real six-leg stepping on video decides the park_duty-class operating point before any rung-2 respec or foot-contact-charge escalation.

**refused_reason**: hexapod-mjx-train-2 code marker 562f830841d42facf9f52d3a3b222197c9a24aec-dirty != local HEAD 562f830841d42facf9f52d3a3b222197c9a24aec and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-2 (and snapshot/commit before that if the tree is dirty).

