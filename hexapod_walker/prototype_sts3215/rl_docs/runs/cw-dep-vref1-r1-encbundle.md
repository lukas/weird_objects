# cw-dep-vref1-r1-encbundle

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T16:54:58+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly with two realistic JOINT-sensing quirks together instead of one at a time -- noisy encoder reads AND a small per-joint zero-point offset, which the real servos will both have simultaneously (calibration is never perfect). vref1-r1 already PASSED encoder noise (0.5deg) and joint zero-bias (3deg) INDIVIDUALLY tonight; this bundles them onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not warm-started off either single-axis checkpoint, to avoid compounding one lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + both axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the two individually-benign joint-sensing axes stay benign combined, same pattern as comshift+deadband and fric+groundtilt5. If-false: combined joint-sensing error (noise stacked on a fixed offset) breaks tracking in a way neither axis did alone -- flag as a real pre-attempt-#2 calibration risk.

**gate**: own-cfg (DR0.35 + dr.encoder_noise_deg=0.5 + dr.joint_zero_bias_deg=3.0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 retention clean; frames watched det

**refused_reason**: hexapod-mjx-train-9 code marker 6721ad9d59ecb4653040a7557b7bf7fe4857b60e != local HEAD c05f11a1f111b6ed02e8e7b2ad5d576a45d4830a. Sync first: snapshot.sh --sync hexapod-mjx-train-9 (and snapshot/commit before that if the tree is dirty).

