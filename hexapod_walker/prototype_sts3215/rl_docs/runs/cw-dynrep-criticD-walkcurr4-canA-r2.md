# cw-dynrep-criticD-walkcurr4-canA-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-18T10:00:15+00:00

**pod**: hexapod-mjx-train-7

**steps**: 4000000

**parent**: cw-dynrep-criticD-walkcurr3

**wandb_id**: xf6htku3

**hardware_ready**: False

**hypothesis**: Can the walking robot START walking upright if we only pay for upright progress and update the brain more aggressively? walkcurr3 is numerically healthy but stuck in a crouched shuffle (-65mm, height_factor 0.07, slip 4.6/m, zero B0 promotions) because the calibrated height/progress income gates were left OFF and its actor update (1e-4 x 3 epochs) is far below the proven acquisition recipe. Arm A of the operator's same-seed A/B/C tournament (fb_20260818T085648_2a0a60): gates ON (sigma 11mm, 25mm safety cutoff, kernel progress gate) + actor LR 2e-4 x 5 epochs held until B0 promotes, then the default-off frontier-gated handover to 3 epochs / 1e-4->1e-5. Prediction-if-true: B0 certs upright (height_factor>=0.8) with cmd_prog_frac>=0.5 by 4M; if-false: still crouch-terminates or KL blowups force rollbacks. Strongest alternative: acquisition needs BC/gait-entry help, not LR. (-r2: attempt 1 hit a missing encoder on train-6, -r1 hit the 64M /dev/shm SIGBUS on old-spec train-7; train-7 recreated with 4Gi dshm + full artifact set.)

**gate**: Behavioral admission at 4M, judged on B0 cert telemetry (NOT crash-free): cmd_prog_frac>=0.50, height_factor>=0.8, falls==0 in final cert round, slip_per_m<=3.0, positive recent B0 progress slope; B0 promotion preferred. Rank passing arms by retention-clean frontier, then cmd_prog_frac, slip, roll. Winner contributes its RECIPE (fresh actor) to 40M cw-dynrep-criticD-walkcurr4; if all three fail, one evidence-based correction then re-canary — never launch the 40M on a failing recipe.

**verdict**: FAIL — 4M canary refutes the LR/gates-only hypothesis: with the calibrated walk_height_gate/walk_kernel_prog_gate income gates ON and a stronger actor update (2e-4 x 5 epochs, held pre-promotion) this arm reproduces walkcurr3's exact crouch-shuffle stall — walkcurr/frontier and walkcurr/promotions stayed at 0 through the full 4M budget (cert_round 8, b0_ignition/pass=0), cmd_prog_frac 0.024 (bar >=0.50), height_factor 0.55-0.70 (bar >=0.8), slip_per_m 5.4 (bar <=3.0). Gate-eval confirms: DR-0 det gait_valid 0/6, median prog 0.03, fwd 0.06m over 15s, slip_per_m med 6.83; DR-0.3 det gait_valid 0/6, prog med 0.02, slip med 7.31; every sto rollout drifts into walk_low_height terminations. Frame strip (walk_det_0.png) shows the robot settle into a static low crouch and barely translate across the whole episode — same failure shape as walkcurr3, not a new one. Turning the gates on and raising the actor LR alone does NOT fix acquisition. Session-pairing eval correctly refused this checkpoint (hist16-wide obs 1152 vs the deployed stance partner's 72 — expected incompatible-candidate exit, not a bug). No follow-up launched from this arm alone — one leg of the operator's 3-arm tournament (fb_20260818T085648_2a0a60 + addendum fb_20260818T085834_588d9a); canB-r1/canC-r1 (actor-only transplant from the proven BC-gait recipe) are running under a concurrent cycle and will decide the correction.

