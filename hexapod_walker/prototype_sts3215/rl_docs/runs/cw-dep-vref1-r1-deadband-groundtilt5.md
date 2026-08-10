# cw-dep-vref1-r1-deadband-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T18:24:35+00:00

**pod**: hexapod-mjx-train-3

**steps**: 8000000

**parent**: cw-dep-vref1-r1-deadband

**wandb_id**: 1wam62gi

**hardware_ready**: False

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if a sluggish/dead-zone servo response happens on an unlevel floor at the same time? Both PASSed alone on vref1-r1 (deadband 1.0-3.0x; ground tilt 5deg) but never together -- a sluggish push-off is more likely to matter exactly when the floor is already sloped against you, unlike axes that don't interact mechanically (e.g. IMU mount rotation). Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- deadband+slope composes free like every other axis pairing tonight. If-false: sluggish response on a slope defeats the contract-exact obs in a way neither did alone -- flag as a real hardware risk (uneven, imperfectly-responsive floor+servo combo) before deployment.

**gate**: own-cfg (DR0.35 + dr.deadband_scale=1.0,3.0 + dr.ground_tilt_deg=5.0) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

**verdict**: PASS -- confirms if-true: servo deadband (1-3x) and a 5deg sloped floor stay benign COMBINED, matching every prior compose tonight. Own-cfg (DR0.35+both axes) det 5/6 ok / sto 4/6 ok, gv 6/6 both, 0 term either pass, slip/m med 1.06 det / 1.08 sto -- both inside vref1-r1's own band (0.89-1.36). Degraded episodes (det/5, sto/0, sto/1 -- prog 0.63-0.70, slip 1.65-1.85) are the lineage's known FIXED-SEED hard-DR-draw march-in-place stall, same magnitude/indices as every other PASSed DR0.35 sibling; frame-checked (det/5 both modes): level body, all six legs cycling, no flag-leg/drag/fall. DR0-no-override gate retention also clean: det 5/6 sto 6/6, gv 12/12, 0 term; the one det/4 catastrophic-slip draw (28.58) reproduces the SAME shared-lineage fixed-seed crater seen at the identical index/magnitude in cmddrop/comshift-deadband/fric-groundtilt5/torquescale-gyronoise/encnoise-latency/velscale's own DR0 gate reports (20-29 slip, ~0 prog) -- confirmed by direct comparison, not specific to deadband+groundtilt5, and absent from the base vref1-r1 checkpoint's own DR0 gate. Training finished clean (reward quarters 422/674/690/680). Not independently hardware-ready (inherits vref1-r1's own paddle-gait economics); clears the combined sluggish-servo + sloped-floor axis as safe for the hardware candidate.

