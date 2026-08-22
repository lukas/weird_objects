# cw-amp-m2-bcinit-sec5-style05-speedrange-cadcouple-nostyle

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T23:08:14+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-speedrange-fastphase-nostyle

**wandb_id**: sn6cnls0

**hypothesis**: Plain English: same speed-coupled step-timing clock as the -cadcouple-r1 twin, but with the AMP style reward switched off, to see whether the imitation critic (trained only on fixed-pace demo clips) punishes the robot for stepping at the new command-matched pace. Mechanism: goal.walk_phase_speed_scale=1.0 (hz_eff = 1.333*(s_ref/0.08), cap 3.0 Hz) on the fastphase-nostyle continuation (amp-task-weight=1.0, style=0.0). The constant-clock probes proved style is not the cap for a FIXED cadence; a VARYING cadence is a new question the discriminator has never priced. Prediction-if-true (coupling works, style irrelevant): realized speed spread widens here AND in the styled twin. Prediction-if-false: both twins stay pinned => timing was not the residual cap (stride/actuation or budget is). Divergence branch: only THIS arm widens => the fixed-cadence motion library is vetoing cadence variation => build cadence/speed augmentation for teacher_v2 (STATUS Next item, already scoped as cheap). Bank: test_phase_speed_coupling.py 7/7 PASS; snapshot exp/cadcouple-phase-clock.

**gate**: Discovery (2M, DR-0, judged jointly with -cadcouple-r1). INFORMATIVE-PASS = gait_valid >=5/6 det+sto, no new sacrificed legs/terminations, AND realized det speed_mean range widens meaningfully past the parent band (spread ratio max/min >= 2.0 across sampled 0.05-0.25 commands, vs ~1.5 pinned today) with per-episode speed correlating with command. FAIL-collapse = terminations/statue/drag or paddle-creep on video even if speed widens (video-overrides-scalar). Joint read decides whether the clock coupling unlocks speed tracking and whether AMP style fights a varying cadence.

**verdict**: Task-only twin of cadcouple-r1 -- lands in the SAME partial regime, confirming AMP style neither vetoes nor rescues the cadence-coupling mechanism. DR-0 gate: gait_valid 6/6 det+sto, zero sacrificed legs; speed_mean spans 0.057-0.118 (styled sibling: 0.051-0.114, essentially identical spread and center); slip det med 5.11 (sibling 4.40), sto med 6.54 (sibling 6.59) -- same order of degradation, same high-frequency-shuffle signature on the worst episodes (walk_sto_4 frame-strip: legs cycling fast, body barely advancing across the checkerboard). This CLOSES the 'style-veto' branch this arm's own gate pre-registered: style is not fighting the faster/variable clock (nostyle shows the identical pathology), and style is not protecting against it either. Consistent with every other axis tested this cycle (heading 0/25/90/full, speed range, now cadence-coupling): AMP style has shown ZERO measured functional necessity anywhere in the M2 command envelope built so far on this reward/init. hardware-ready: no (2M discovery, DR-0). Next real lever (shared with the styled sibling): couple stride length/workspace amplitude to commanded speed, not just clock rate -- or price wasted-motion slip more directly during high-cadence segments.

