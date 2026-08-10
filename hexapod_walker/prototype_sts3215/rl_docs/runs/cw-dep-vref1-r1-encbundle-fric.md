# cw-dep-vref1-r1-encbundle-fric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T18:49:19+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-dep-vref1-r1-encbundle

**wandb_id**: dowclv10

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if imperfect joint sensing (encoder noise + zero-bias, already PASSed as a bundle) happens on a floor with uncertain grip (friction 0.4-1.6x, already PASSed alone)? Both individually benign but never combined -- a noisy/biased joint-position estimate matters most exactly when foot placement is already grip-limited, unlike axes that don't share a failure pathway. Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- joint-sensing error composes free with floor-grip uncertainty like every other pairing tonight. If-false: the combination degrades tracking/slip beyond either alone -- flag as a real pre-attempt-#2 risk (sensing error is least forgiving on a low-grip floor).

**gate**: own-cfg (DR0.35 + dr.encoder_noise_deg=0.5 + dr.joint_zero_bias_deg=3.0 + dr.friction_scale=0.4,1.6) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv; video frames watched det+sto

