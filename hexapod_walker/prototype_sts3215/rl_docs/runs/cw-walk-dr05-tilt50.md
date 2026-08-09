# cw-walk-dr05-tilt50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T12:41:13+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_dr05_r1.zip

**wandb_id**: 4blvzgkw

**hypothesis**: 0-c(i) STABILITY, tilt-price half (split one-variable arm off cw-walk-dr05-r1, md5 13a668ea). Plain: under randomized physics + action noise the walker occasionally lets its body tip far (to ~11 deg) and the gait degenerates (flag leg, slip blowouts) — tipping is nearly free today (quad k_roll/k_pitch=10 => ~0.3/tick at 10 deg vs ~2-3/tick walk income; kernel flat past ~3 deg). One variable vs parent: reward.k_roll=50, reward.k_pitch=50 (1.5/tick at 10 deg, 0.03/tick at 1.5 deg — normal gait unaffected per scale audit). Prediction-if-true: DR0.5 sto blowout episodes disappear (gv 12/12, no ep slip/m>3; parent had 3 such eps + gv 5/6 sto) with det gait/income intact. Prediction-if-false: blowouts persist (tilt is a downstream symptom of the flag-leg failure, not the driver) OR speed/prog collapses (pricing suppresses leg swings). Strongest alternative: walks flatter but still slides — slip unchanged, gv unchanged. (Retry 2: pod race on t4, stale SHA on t9.)

**gate**: own-cfg DR0.5 6+6: 0 term, gv 12/12, no ep slip/m>3 (parent 3 eps), det agg slip/m <=1.3 (parent 1.56); DR0 det retention along>=0.55 gv 6/6; frames watched det+sto

