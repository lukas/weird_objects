# cw-standwalk-unified1-joyfix-velobs3-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T16:13:48+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: ve4ahvl1

**hypothesis**: Plain English: the unified walking policy undershoots commanded speed by ~50% and its 'velocity' observation is just a copy of the command (walk_obs_body_vel=2 ref-copy contract — 'carries ZERO body-velocity information by construction'), so it cannot close the loop on its own achieved speed; this canary switches those two obs channels to the deployable leg-odometry velocity estimator (mode 3, same obs width, built 08-20 exactly for this defect) on the otherwise identical recipe, warm from long-s0's 16M PASS checkpoint. Predict-if-true: after an initial adaptation dip the policy re-stabilizes by 2M (terminations back at parent band) and forward speed_ratio in the response probe is at or above the parent's 0.475. Predict-if-false: the semantic swap of channels 70/71 destabilizes the multi-mode policy (rise/hold terminations explode and don't recover by 2M) — would say mode 3 needs a fresh-lineage or annealed introduction, not a warm swap. Strongest alternative: the undershoot is priced in by the reward stack, not obs-limited (the cmdtrack arm decides).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only): at 2M — training reward recovers to within noise of the long-s0 continuation trend after the obs-swap dip; DR-0 det walk gait_valid >=5/6 no new sacrificed leg; session terminations <=6/90; rise success (owndr) not collapsed (>=4/6 det); probe fwd speed_ratio >=0.40. PASS -> fold into the acquisition bundle; FAIL by instability -> retry as annealed/staged obs swap or fresh lineage only.

