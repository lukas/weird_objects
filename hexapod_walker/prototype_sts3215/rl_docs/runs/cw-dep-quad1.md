# cw-dep-quad1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T16:13:24+00:00

**pod**: hexapod-mjx-train-10

**steps**: 18000000

**parent**: cw-dep-vref1-r1

**wandb_id**: dc8654qc

**hardware_ready**: False

**hypothesis**: Second joystick-priority line (quad tricks) meets the deployment contract for the first time: does the mainline quad-hold trick (cw-quad-hold2 PASS at 30/60/10 quad/walk/hold mix, warm from the OLD longdist_r2/privileged-velocity champion) still work when trained on top of the contract-exact hardware base instead? Warm-start from cw-dep-vref1-r1's own checkpoint (meas:=ref velocity obs, 25deg tilt envelope, k_current=0 per P0 rule 3) and graft the identical quad-hold2 goal-mix + reward recipe (quad=0.3/walk=0.6/hold=0.1, k_quad_clear=1.5, k_quad_plant=1.0, quad_grace_s=1.5) -- one variable vs quad-hold2: the base checkpoint/obs-contract. If-true: quad survived_frac ~1.0 with fronts_off/planted_frac/height_err solid (matches quad-hold2's own telemetry) AND walk-mode own-cfg det slip/m <=1.25ish (quad-hold2's own retained band) -- the trick transfers cleanly onto the hardware-contract line, meaning quad work doesn't have to wait for a separate contract-migration arm later. If-false: quad mechanism itself fails to emerge (velocity-free obs may remove information the quad-hold reward leans on for timing) or walk retention erodes worse than quad-hold2's own baseline -- quad-line stays parked on the privileged-velocity champion until a dedicated contract-migration arm is designed.

**gate**: Quad-mode eval/quad survived_frac >=0.9 across training checkpoints, height_err_end_mm <=20mm, video confirms fronts lifted clear/no tipping; own-cfg walk-mode det gv 6/6, 0 term, slip/m med <=1.35 (quad-hold2's own retained band); frames watched for both modes

**verdict**: FAIL (if-false branch): quad-hold four-leg trick does not fall (survived_frac 1.0 at every logged checkpoint) but never converges to the height-control precision gate under the contract-exact (no privileged velocity) obs -- height_err_end_mm improves 60mm->31mm over training but plateaus well above the <=20mm gate at the final checkpoint, and track_err_deg gets WORSE not better (1.0->2.67deg). This differs from quad-hold2 (same 30/60/10 mix, same reward recipe, privileged-velocity base), which converged to 1-15mm. Walk-mode retention holds: own-cfg(DR0.35) det gv 6/6 slip med 1.18, sto gv 6/6 slip med 1.47 (one crater pair driving the median, no falls); DR0 gate det gv 6/6 slip 1.08, sto gv 5/6 (one lineage march-in-place stall, sac legs [3,5], video-checked, not new). Conclusion: the quad height-timing mechanism appears to lean on the privileged velocity feedback that the hardware-deploy obs contract removes -- quad-line stays on the privileged-velocity base (quad-hold2) for now; a contract-exact quad graft needs either more steps/height-error shaping or is deferred behind the walk-only hardware ladder (quad is the P1 party trick, not the P0 walk deliverable).

