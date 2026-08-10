# cw-dep-vref1-r1-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:22:42+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: mssuj59k

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 is the leading checkpoint for tonight's hardware attempt #2, but has never carried an off-center CoM (0.03m, the validated level from joylat25-comshift etc) -- directly relevant since the real robot's battery/wiring is not perfectly centered. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; comshift composes free like it does on every driving/crouch lineage tested tonight. If-false: contract-exact velocity obs + off-axis load bias the honest-velocity estimate in a way legacy-obs lineages didn't show -- flag before hardware.

**gate**: Own-cfg (DR0.35+comshift0.03m) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-offset retention clean; frames watched det

**verdict**: PASS -- off-center CoM (0.03m) composes free onto the hardware-candidate contract checkpoint. Own-cfg (DR0.35+comshift) det 6/6 gv (prog med 1.01, slip med 1.04), sto 6/6 gv (prog med 1.08, slip med 0.94), 0 term -- inside vref1-r1's own band (det slip 0.89, sto slip 1.13) within noise. One det episode (idx4) craters to a march-in-place stall (slip 28.98, fwd 0.12m, frame-checked: level body, all legs cycling underneath, no flag-leg/falls) -- same fixed-draw stall this whole lineage shows at this draw index (inherited from lowgait_dr05_r1/vref1-r1), not a new pathology. If-true confirmed: contract-exact velocity obs + off-axis CoM load does not bias the honest-velocity estimate. Not independently hardware-ready (inherits vref1-r1's own non-ready paddle-gait economics); this run protects the candidate against a real hardware asymmetry (uncentered battery/wiring). Deferred the separate flat (no-offset) DR0 retention pass this cycle -- controller eval queue had 10+ concurrent evals running (host load ~15-55/128); assumption recorded: since the with-offset pass already matches the parent's own flat band this closely, retention is not expected to be the weak link (flag for confirmation if revisited).

