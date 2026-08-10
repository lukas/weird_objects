# cw-dep-vref1-r1-encnoise-latency

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T17:39:17+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: xt7ibpfx

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly with two realistic SENSING/COMMS quirks together instead of one at a time -- noisy joint-encoder reads AND variable bus/comms latency, which the real deployed link has simultaneously (every read is both noisy AND delayed). vref1-r1 already PASSED encoder noise (0.5deg) and latency jitter (0.5-2.5x) INDIVIDUALLY tonight; this bundles them onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not warm-started off either single-axis checkpoint, to avoid compounding one lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + both axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the two individually-benign sensing axes stay benign combined, same pattern as comshift+deadband, fric+groundtilt5, encbundle, imubundle. If-false: a stale AND noisy velocity-relevant reading compounds worse than either alone -- flag as a real pre-attempt-#2 sensing/comms risk distinct from the already-tested pure-sensor bundles.

**gate**: own-cfg (DR0.35 + dr.encoder_noise_deg=0.5 + dr.latency_scale=0.5,2.5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band; DR0 retention clean; frames watched det

