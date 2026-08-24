# cw-walkcurr-pf-fwd6-hgt2-pdw05-pdx15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T00:10:02+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-hgt2

**wandb_id**: jqccc0zq

**hypothesis**: Retry (prior attempt PARKED on a stale-code-marker sync race, unrelated to this arm's own logic; see cw-walkcurr-pf-fwd6-hgt2-pdw05-pdx15's original entry). Plain English: dose sibling of hgt2-pdw05 -- same confound fix (goal.park_duty_window_s 2.0->0.5, park-duty charge now fires before the 2.0s termination grace expires) PLUS 1.5x the per-leg contact-duty charge (reward.k_park_duty 0.08->0.12) to test whether the de-confounded charge merely needs to fire vs. needs to fire HARDER to out-price the frozen tripod/belly-sit stances every RND and height-gate arm has converged to so far. Bank-proven at exactly this dose (test_walkcurr_pf_hgt_ tight_pdw05_pdx1p5, 16/16 green -- 3x (0.24) was tried first and REFUTED at the bank stage: it over-taxes the honest 'park' stand-still behavior itself, pushing park_gated BELOW belly_sit_gated; 1.5x is the largest dose that stays bank-legal). Prediction-if-true: freeprog crosses toward/past 0 faster or further than the plain pdw05 sibling. Prediction-if-tied: dose doesn't matter once the confound is fixed. Prediction-if-both-fail identically: park_duty-class charges are insufficient regardless of confound/dose, escalate to a direct minimum-total-foot-contact charge before BC-kickstart.

**gate**: Same rung-1 gate as hgt2-pdw05; read the pair jointly -- whichever dose (or neither) crosses walk_freeprog_score past 0 with real six-leg stepping on video decides the park_duty-class operating point before any rung-2 respec or foot-contact-charge escalation.

**verdict**: Dose sibling of hgt2-pdw05 (park_duty_window fix + 1.5x k_park_duty, 0.08->0.12) FAILS identically -- the extra dose changes nothing qualitatively. env/reward_park_duty nonzero throughout (-0.005 -> -0.016, proportionally larger than the plain-dose sibling's -0.003/-0.011 as expected from the 1.5x multiplier -- the dose IS landing), but env/walk_freeprog_score still flat in [-0.11,-0.10] the whole run (ends -0.101, no better than the plain dose), env/walk_speed decays 0.10->0.054 m/s (same curve, same numbers as the plain dose to 2 decimal places), env/height_err_mm climbs 11->51mm identically, clip_fraction crashes to EXACTLY 0 mid-run (step 161, worse collapse than the plain dose's 2.6e-5) before a partial recovery to 0.011. Own-cfg gate: 6/6 det + 6/6 sto terminate walk_low_height, gait_valid TRUE (duty 0.24-0.78, no sacrificed legs), forward_dist 0.005-0.03m/25s, slip/m 5.8-7.8, height_err_end 65-94mm -- video (walk_det_0.png) is visually indistinguishable from the plain-dose sibling: slow splay-and-sink into a low wide crouch, zero net translation, cut off by the safety cutoff. Read jointly with hgt2-pdw05: dose does NOT matter once the confound is fixed (matches the pre-registered 'Prediction-if-tied' branch exactly) -- the park_duty-class charge (any bank-legal dose, 0.08 or 0.12, confound-fixed or not) is now CLOSED as a mechanism for unlocking rung-1 discovery. This is the last cheap dose-lever on the existing charge stack; every other item on the track's own escalation list (RND dose+budget, rung-0 swing-income, GRU, gSDE, rscale dose+continuations, height-gate loose/tight dose, park_duty confound+dose) is now closed. DIG-IN flagged (same as hgt2-pdw05) for the next design decision: build a direct minimum-total-foot-contact charge (new mechanism + scripted twin + bank proof) vs. finally escalate BC-kickstart to the operator per the track's own item (d).

