# cw-dep-vref1-r1-gainvar-legmass

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T18:15:57+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-dep-vref1-r1-gainvar

**wandb_id**: u5eg0jka

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if two properties of THIS SPECIFIC ROBOT UNIT combine -- wider per-joint servo gain spread (kp/kv) AND per-leg build tolerance (mass jitter + per-leg link-length error)? Both PASSed alone on vref1-r1 (gainvar: kp 0.20->0.40/kv 0.25->0.50; legmass: leg_mass_jitter 0.10->0.20/link_len_leg 0.012->0.025) but never together -- unlike axes that model the ENVIRONMENT (floor, IMU mount), these two are both fixed manufacturing/assembly properties of the one physical unit being tested, so they co-occur by construction on real hardware. Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- unit-level manufacturing variation composes free like every other axis pairing tonight. If-false: combined actuator+structural variation degrades the gait beyond either alone -- flag as a real per-unit calibration risk before the operator's hardware attempt.

**gate**: own-cfg (DR0.35 + dr.kp_scale_pct=0.40 + dr.kv_scale_pct=0.50 + dr.leg_mass_jitter_pct=0.20 + dr.link_len_leg_pct=0.025) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

