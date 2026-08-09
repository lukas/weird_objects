# cw-walk-dr05-fall300

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T12:42:26+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_dr05_r1.zip

**wandb_id**: 0wcr51t3

**hypothesis**: 0-c(i) STABILITY, fall-charge half (split one-variable arm off cw-walk-dr05-r1, md5 13a668ea; twin of cw-walk-dr05-tilt50). Plain: a fall (safety tilt trip) currently costs 10 on a ~900 episode return — the policy is near-indifferent to toppling, especially late in an episode; steer-fdiag showed a tilt_pitch termination at DR1.0 sto. One variable vs parent: reward.safety_termination_penalty 10 -> 300 (~1/3 of episode income; scale audit in RL_LOG c39). Prediction-if-true: terminations go to 0 at DR1.0 sto eval (champ-lineage falls there today) with DR0.5 gait/income unchanged (charge binds only on the rare fall trajectories). Prediction-if-false: no behavior change (falls too rare in DR0.5 TRAINING for the charge to generate gradient — result then says stability pricing must target tilt, not termination) OR overcaution (speed/prog drops as policy hedges away from the trip boundary). Strongest alternative: fewer falls but flag-leg slip blowouts unchanged (falls and blowouts are separate failure modes). (Retry: stale SHA on first attempt.)

**gate**: own-cfg DR0.5 6+6: 0 term, gv 12/12, det agg slip/m <=1.6 (parent 1.56, no worse); PLUS DR1.0 sto 6 eps 0 term (fdiag lineage fell here); DR0 det retention along>=0.55 gv 6/6; frames watched det+sto

