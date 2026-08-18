# cw-dynrep-criticD-walkcurr4-canB-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-18T10:12:22+00:00

**pod**: hexapod-mjx-train-9

**steps**: 4000000

**parent**: cw-dynrep-criticD-walkcurr3

**hypothesis**: Can the walking robot START walking upright if we only pay for upright progress and update the brain more aggressively? walkcurr3 is numerically healthy but stuck in a crouched shuffle (-65mm, height_factor 0.07, slip 4.6/m, zero B0 promotions) because the calibrated height/progress income gates were left OFF and its actor update (1e-4 x 3 epochs) is far below the proven acquisition recipe. Arm B of the operator's same-seed A/B/C tournament (fb_20260818T085648_2a0a60): gates ON (sigma 11mm, 25mm safety cutoff, kernel progress gate) + actor LR 3e-4 x 5 epochs (the known 40m1 acquisition strength) held until B0 promotes, then the default-off frontier-gated handover to 3 epochs / 1e-4->1e-5. Prediction-if-true: B0 certs upright (height_factor>=0.8) with cmd_prog_frac>=0.5 by 4M; if-false: crouch-terminates or KL blowups force rollbacks. Strongest alternative: acquisition needs BC/gait-entry help, not LR. (-r1: first attempt REFUSED on a stale code marker after pod recreation; re-synced.)

**gate**: Behavioral admission at 4M, judged on B0 cert telemetry (NOT crash-free): cmd_prog_frac>=0.50, height_factor>=0.8, falls==0 in final cert round, slip_per_m<=3.0, positive recent B0 progress slope; B0 promotion preferred. Rank passing arms by retention-clean frontier, then cmd_prog_frac, slip, roll. Winner contributes its RECIPE (fresh actor) to 40M cw-dynrep-criticD-walkcurr4; if all three fail, one evidence-based correction then re-canary — never launch the 40M on a failing recipe.

