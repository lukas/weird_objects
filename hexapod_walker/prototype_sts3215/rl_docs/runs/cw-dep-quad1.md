# cw-dep-quad1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T16:13:24+00:00

**pod**: hexapod-mjx-train-10

**steps**: 18000000

**parent**: cw-dep-vref1-r1

**wandb_id**: dc8654qc

**hypothesis**: Second joystick-priority line (quad tricks) meets the deployment contract for the first time: does the mainline quad-hold trick (cw-quad-hold2 PASS at 30/60/10 quad/walk/hold mix, warm from the OLD longdist_r2/privileged-velocity champion) still work when trained on top of the contract-exact hardware base instead? Warm-start from cw-dep-vref1-r1's own checkpoint (meas:=ref velocity obs, 25deg tilt envelope, k_current=0 per P0 rule 3) and graft the identical quad-hold2 goal-mix + reward recipe (quad=0.3/walk=0.6/hold=0.1, k_quad_clear=1.5, k_quad_plant=1.0, quad_grace_s=1.5) -- one variable vs quad-hold2: the base checkpoint/obs-contract. If-true: quad survived_frac ~1.0 with fronts_off/planted_frac/height_err solid (matches quad-hold2's own telemetry) AND walk-mode own-cfg det slip/m <=1.25ish (quad-hold2's own retained band) -- the trick transfers cleanly onto the hardware-contract line, meaning quad work doesn't have to wait for a separate contract-migration arm later. If-false: quad mechanism itself fails to emerge (velocity-free obs may remove information the quad-hold reward leans on for timing) or walk retention erodes worse than quad-hold2's own baseline -- quad-line stays parked on the privileged-velocity champion until a dedicated contract-migration arm is designed.

**gate**: Quad-mode eval/quad survived_frac >=0.9 across training checkpoints, height_err_end_mm <=20mm, video confirms fronts lifted clear/no tipping; own-cfg walk-mode det gv 6/6, 0 term, slip/m med <=1.35 (quad-hold2's own retained band); frames watched for both modes

