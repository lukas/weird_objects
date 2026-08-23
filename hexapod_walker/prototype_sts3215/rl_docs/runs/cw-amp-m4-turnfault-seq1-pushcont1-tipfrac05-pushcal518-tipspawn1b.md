# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn1b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T13:34:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: ytbfsr7y

**hypothesis**: Plain English: the robot still cannot turn in place at the commanded rate, and we have now measured that neither paying more for turning (k_yaw_prog 1-3x grid), nor faster turn demos (teacher_v3, +30% wz), nor removing the AMP style term moves it — so this arm tests the last named cause: the policy never STARTS an episode already turning, so PPO never gets gradient inside fast-turn states. Mechanism (new, snapshotted exp/amp-gait-spawn-wz, default-off key, test_sim_env 45/45 + semantics bank green): goal.walk_gait_spawn_wz=1.0 passes the episode's own commanded yaw rate into the mid-stride gait spawn pose and makes the yaw command live from the 0.3s fast ramp; goal.walk_gait_start_frac=0.5 enables the spawn machinery (coupled unit: the passthrough is meaningless without gait spawns). ~25% of episodes become mid-TURN spawns (tip_frac 0.5 x gait 0.5), ~25% mid-stride walk spawns (benign tall-walk RSI). Prediction-if-true: m5 yaw tips drop >=0.03 toward the 0.20 bar vs parent 0.2157/0.2351. Prediction-if-false: tips unmoved +-0.02 — reset densification joins pricing/demos/style as refuted, leaving stance-geometry authority or the 0.20 bar ruling (q_20260823T0130Z amendment) as the only remaining levers. Strongest alternative: the mid-walk RSI half of the dose (not turn spawns) moves walk slip instead — read both sections.

**gate**: eval_amp_m5 yaw+walk on own cfg. PASS = 0/12 raw falls AND both tip errs <=0.20 AND walk det slip med <=3.8. PARTIAL = both tips improve >=0.03 vs 0.2157/0.2351 but one misses 0.20 (densification works — raise dose: gait_frac 0.75 or +budget continuation). FAIL = tips unmoved (+-0.02) or slip >3.8 or any fall — reset densification refuted; remaining levers are stance-geometry/turn-authority mechanisms or the 0.20 bar amendment ruling; do NOT re-dose this key.

