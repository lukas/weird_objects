# cw-steer-fdiag

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T11:30:01+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: kl87ee0r

**hardware_ready**: no

**hypothesis**: Steering-competence arm (operator parallel line; A/B scope partner to cw-walk-fwdband). Plain: the robot mostly practiced 'go straight ahead'; to drive it around a room it must follow diagonal commands too. This run trains the champion on commands drawn uniformly across the ruled forward-diagonal scope (+/-45deg, walk_heading_max_rad=pi/4) instead of the legacy 60% straight / 20% diag / 20% anywhere mix - one variable off champion 35234ddc. Prediction-if-true: diagonal-draw progress_ratio (harness, own cfg, DR1.0) improves vs the champion fdiag baseline (recorded this cycle in logs/ckpt_eval/champ_anchorgate_fdiag_dr10) beyond eval noise, with forward retention (DR0 det fwd>=0.55, gv 12/12) intact. Prediction-if-false: diag progress_ratio stays at baseline while forward holds - diagonal transport is limited by the paddle gait itself, not the training distribution; steering waits on the contact-pricing fix and scope-only arms stop. Strongest alternative: uniform +/-45deg merely DILUTES forward training - that shows as forward-retention FAIL with no diag gain, distinguishable from both predictions. Exploratory/non-promotion per rulings section 4; cannot displace the champion.

**gate**: own-cfg DR1.0 harness 15s 6+6: diag-draw (|heading|>=15deg) mean prog_ratio improves vs champ fdiag baseline beyond +/-1-2ep noise; DR0 det fwd>=0.55; gv 12/12; 0 term; frames watched det+sto. Non-promotion arm.

**verdict**: FAIL (hypothesis REFUTED per pre-registration): DR1.0 fdiag panel det diag-draw prog_ratio mean 1.47 vs champ baseline 1.64 — right direction but within the 1-2ep noise band; sto DEGRADED vs baseline: 1 tilt_pitch termination + 2 sacrificed-leg eps (gv 4/6, slip/m to 13.4) vs baseline gv 5/6 no falls. DR0 forward retention PASS (along 0.58-0.71, gv 6/6) — NOT dilution. Paddle gait itself, not the command distribution, blocks steering; scope-only steering arms STOP pending contact/current pricing. Frames watched (DR1.0 det): body tilts to ~11 deg on diag draws, feet cycling but sliding. Not hardware-ready.

