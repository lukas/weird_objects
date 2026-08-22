# cw-amp-m2-bcinit-sec5-style05-speedrange-cadcouple-nostyle

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T23:08:14+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-speedrange-fastphase-nostyle

**wandb_id**: sn6cnls0

**hypothesis**: Plain English: same speed-coupled step-timing clock as the -cadcouple-r1 twin, but with the AMP style reward switched off, to see whether the imitation critic (trained only on fixed-pace demo clips) punishes the robot for stepping at the new command-matched pace. Mechanism: goal.walk_phase_speed_scale=1.0 (hz_eff = 1.333*(s_ref/0.08), cap 3.0 Hz) on the fastphase-nostyle continuation (amp-task-weight=1.0, style=0.0). The constant-clock probes proved style is not the cap for a FIXED cadence; a VARYING cadence is a new question the discriminator has never priced. Prediction-if-true (coupling works, style irrelevant): realized speed spread widens here AND in the styled twin. Prediction-if-false: both twins stay pinned => timing was not the residual cap (stride/actuation or budget is). Divergence branch: only THIS arm widens => the fixed-cadence motion library is vetoing cadence variation => build cadence/speed augmentation for teacher_v2 (STATUS Next item, already scoped as cheap). Bank: test_phase_speed_coupling.py 7/7 PASS; snapshot exp/cadcouple-phase-clock.

**gate**: Discovery (2M, DR-0, judged jointly with -cadcouple-r1). INFORMATIVE-PASS = gait_valid >=5/6 det+sto, no new sacrificed legs/terminations, AND realized det speed_mean range widens meaningfully past the parent band (spread ratio max/min >= 2.0 across sampled 0.05-0.25 commands, vs ~1.5 pinned today) with per-episode speed correlating with command. FAIL-collapse = terminations/statue/drag or paddle-creep on video even if speed widens (video-overrides-scalar). Joint read decides whether the clock coupling unlocks speed tracking and whether AMP style fights a varying cadence.

