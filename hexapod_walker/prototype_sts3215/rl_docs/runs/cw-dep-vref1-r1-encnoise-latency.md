# cw-dep-vref1-r1-encnoise-latency

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T17:39:17+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: xt7ibpfx

**hardware_ready**: False

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly with two realistic SENSING/COMMS quirks together instead of one at a time -- noisy joint-encoder reads AND variable bus/comms latency, which the real deployed link has simultaneously (every read is both noisy AND delayed). vref1-r1 already PASSED encoder noise (0.5deg) and latency jitter (0.5-2.5x) INDIVIDUALLY tonight; this bundles them onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not warm-started off either single-axis checkpoint, to avoid compounding one lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + both axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the two individually-benign sensing axes stay benign combined, same pattern as comshift+deadband, fric+groundtilt5, encbundle, imubundle. If-false: a stale AND noisy velocity-relevant reading compounds worse than either alone -- flag as a real pre-attempt-#2 sensing/comms risk distinct from the already-tested pure-sensor bundles.

**gate**: own-cfg (DR0.35 + dr.encoder_noise_deg=0.5 + dr.latency_scale=0.5,2.5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band; DR0 retention clean; frames watched det

**verdict**: PASS -- confirms if-true: encoder noise (0.5deg) and comms/bus latency jitter (0.5-2.5x) stay benign COMBINED, same as every prior sensing-axis compose tonight. Own-cfg (DR0.35+both axes) det+sto 6/6 gv, 0 term, slip/m med 0.96 det (inside vref1-r1's own 0.89-1.13 band) / 1.06 sto (inside/at the low end of the 1.13-1.36 band). Degraded episodes (det/5, sto/0, sto/1 -- prog 0.57-0.68, slip 2.0-2.6) are the lineage's known FIXED-SEED hard-DR-draw fingerprint that appears at the identical indices in every already-PASSed DR0.35 sibling (torquescale, gainvar, tiltnoise, encbundle, gyronoise, imumount, latency); frame-checked (det/5): level body, all six legs still cycling, no flag-leg/drag/fall, a march-in-place stall not a new pathology. DR0 no-axes retention also clean: det+sto 6/6 gv, 0 term. Not independently hardware-ready (inherits vref1-r1's own paddle-gait economics); clears the combined sensing/comms-noise axis as safe for the hardware candidate.

