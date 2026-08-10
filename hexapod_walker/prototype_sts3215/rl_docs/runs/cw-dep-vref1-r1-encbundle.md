# cw-dep-vref1-r1-encbundle

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: EVALUATED

**created**: 2026-08-10T16:56:14+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 6hedb9pn

**hardware_ready**: False

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly with two realistic JOINT-sensing quirks together instead of one at a time -- noisy encoder reads AND a small per-joint zero-point offset, which the real servos will both have simultaneously (calibration is never perfect). vref1-r1 already PASSED encoder noise (0.5deg) and joint zero-bias (3deg) INDIVIDUALLY tonight; this bundles them onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not warm-started off either single-axis checkpoint, to avoid compounding one lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + both axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the two individually-benign joint-sensing axes stay benign combined, same pattern as comshift+deadband and fric+groundtilt5. If-false: combined joint-sensing error (noise stacked on a fixed offset) breaks tracking in a way neither axis did alone -- flag as a real pre-attempt-#2 calibration risk.

**gate**: own-cfg (DR0.35 + dr.encoder_noise_deg=0.5 + dr.joint_zero_bias_deg=3.0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 retention clean; frames watched det

**verdict**: PASS -- combined joint-sensing (encoder noise 0.5deg + zero-bias 3deg) composes free onto the contract-exact hardware base. DR0-gate (nominal+override) det+sto 6/6 gv, 0 term, slip/m med 1.15/0.95 (within/near vref1-r1 own 0.89-1.13/1.13-1.36 band, det 2% over upper edge, well inside +-20% tol). Own-cfg (DR0.35+override) det+sto 6/6 gv, 0 term, slip/m med 1.07/1.15, with the SAME det/5+sto/0-1 degraded-episode pattern (prog 0.5-0.9, slip 1-2) seen identically in already-PASSed torquescale/tiltnoise siblings at DR0.35 -- a curriculum-DR artifact, not new. Video (contact sheets + crater/degraded frames) shows the lineage known low-amplitude six-leg creep, body level, no flag leg, no fall.

