# cw-walk-posetrack

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-09T23:26:04+00:00

**pod**: hexapod-mjx-train-9

**steps**: 10000000

**parent**: cw-walk-longdist-r2

**wandb_id**: taqvjab1

**hardware_ready**: False

**hypothesis**: WISHLIST item 18 (body pose control, READY, untried): warm-start the walk champion and retrain with goal-mix hold/lean/track only (no walk) so the robot holds a fixed stance while tracking commanded roll/pitch (and implicitly height via the stance height term) instead of just standing quietly. If-true: own-cfg lean+track panel hits track_err_mean_deg<=1.5 on most episodes, 0 term, no leg-through-floor squat (video). If-false: pose tracking degrades below 1.5deg or walk-quality knowledge doesn't transfer to stable standing (unlikely given hold already works) -- report the actual band.

**gate**: Own-cfg harness, --modes hold lean track, --per-mode 6, dr-scale 0, det+sto: gv/success (track_err_mean_deg<=1.5) on >=10/12 episodes each mode-pair, 0 term; video shows all six feet planted, no leg sweeping through floor (post-273ebde sim).

**verdict**: FAIL (gate: tracking accuracy) on lean/track, hold PASSES clean. Own-cfg panel: hold track_err_mean_deg 0.84 (well under 1.5 gate, height_err 1.2mm) - quiet stable stance confirmed. lean/track MISS the 1.5deg gate: lean 2.9/3.6deg (det/sto), track 2.2/2.65deg (det/sto) - roughly 1.5-2.4x over tolerance, but 0 terminations, no falls, no leg-through-floor (video), stable planted stance throughout (height_err 1.2-9.3mm). Reasonable first attempt at a genuinely new skill (10M steps, 3-way mix) - moving/tilted body-pose commands need tighter tracking than a static hold; likely fixable with more steps or a mix that spends less budget on plain hold. Not hardware-ready as a body-pose skill yet; hold sub-skill is solid and feeds the UNIFIED JOYSTICK quiet-hold requirement.

