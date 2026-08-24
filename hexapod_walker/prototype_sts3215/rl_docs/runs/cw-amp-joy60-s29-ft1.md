# cw-amp-joy60-s29-ft1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T17:54:30+00:00

**pod**: hexapod-mjx-train-4

**steps**: 8000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz11-s29

**wandb_id**: mpj70aqg

**hypothesis**: Fine-tuning the AMP M5 champion on full-length 60-second joystick command sessions should close its last gap to the joystick DONE gate: on the corrected manual gate it already never falls and keeps a valid six-leg gait (n=48, falls=0, gait_valid=1.0, no sacrificed legs) but slips a bit too much (3.679 vs cap 2.9) and misses direction under command churn (48.15 vs 40 deg); deterministic is near-gate, stochastic/churn is the miss. Operator-ordered run (MCP 20260824T175033Z): warm-start ppo_goal_..._phasehz11_s29.zip, identical recipe/obs dialect (25 Hz, phase_obs 1.1 Hz, obs_body_vel=2, yaw_cmd 0.3 rad/s, phase_run_on_yaw, fault_health, fast motor contract write_speed=1500/acc=80/max_delta_q 5deg, stress_mix resample 4.0 s jitter 0.5, AMP style 0.5 to preserve the gait), changing ONLY episode-seconds 15->60 so training sees the same command-churn horizon the gate scores. Prediction-if-true: joystick gate goes green - slip/m med <=2.9 and direction_err med <=40 with zero falls, DR-0+own-DR, det+sto. Prediction-if-false: slip/dir stay above gate while AMP reward rises - the reward (no explicit slip pricing) is misaligned with the joystick gate; next arm adds slip pricing after a semantics-bank check, per the 08-21 ruling. Strongest alternative: 60 s episodes dilute the style/task balance and the gait itself degrades - catch on video/gait_valid.

**gate**: Canonical eval_joystick_gate.py randomized 60 s session, held-out seed 90000, DR-0 + own-DR, det+sto (watcher joygate artifact logs/ckpt_eval/<run>_joygate/gate_verdict.json). PASS: zero falls, gait_valid all episodes, slip/m med <=2.9 AND direction_err med <=40 deg in every section. PARTIAL: 0 falls + gait valid with slip <=3.3 or dir <=45 (clear improvement vs parent 3.679/48.15). FAIL: any fall, invalid gait, or slip/dir not improved beyond eval noise vs parent.

