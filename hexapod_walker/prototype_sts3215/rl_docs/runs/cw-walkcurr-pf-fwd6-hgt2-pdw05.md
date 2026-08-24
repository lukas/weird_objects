# cw-walkcurr-pf-fwd6-hgt2-pdw05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-24T00:39:47+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-hgt2

**hypothesis**: Plain English: hgt2 (tight height-gate dose) FAILed, and its own wandb read found a mechanical confound -- the safety.walk_height_grace_s=2.0s termination fires before the goal.park_duty_window_s=2.0s trailing contact-duty history buffer ever fills, so env/reward_park_duty stayed EXACTLY 0 for the entire run: the one existing charge that already prices a permanently-unloaded leg (k_park_duty's duty<0.1 branch) never got a chance to act on the belly-sit collapse. Single lever vs hgt2: shorten goal.park_duty_window_s 2.0->0.5s (1/4 of the grace period, fills with room to spare) -- same sigma=11/drop=25/grace=2.0, same fresh 2M discovery init, not warm-started. Bank-checked this cycle (test_walkcurr_pf_hgt_gate_bites_belly_sit/gait_beats_belly_sit/light_tax/ranking_still_holds, new 'tight_pdw05' dose, 4/4 green, gate still fires fast and gait still clearly beats belly_sit/park/stall). Prediction-if-true: walk_freeprog_score leaves [-0.12,-0.08] and trends toward/past 0, env/reward_park_duty becomes nonzero, gait_valid improves on the det panel. Prediction-if-false (freeprog stays flat/negative, reward_park_duty stays near 0 or the same collapse-and-terminate loop recurs): the confound was real but not consequential -- de-confounding alone does not unlock walking, and the track's next escalation is a genuinely new mechanism (direct minimum-total-foot-contact charge) or BC-kickstart, not further height-gate calibration.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- prog_ratio>0 and gait_valid on >=4/6 det episodes with visible forward travel on video, env/walk_freeprog_score leaves [-0.12,-0.08] and trends toward/past 0 by 2M, clip_fraction stays healthy. Additionally check env/reward_park_duty is nonzero by the end (confirms the confound fix actually engaged) and read jointly with hgt2-pdw05-pdx15 (the dose-boosted sibling) before any further height-gate calibration.

**refused_reason**: hexapod-mjx-train-1 already runs cw-walkcurr-pf-fwd6-actbias1 — GPU pods host exactly one run; pick a free GPU pod.

