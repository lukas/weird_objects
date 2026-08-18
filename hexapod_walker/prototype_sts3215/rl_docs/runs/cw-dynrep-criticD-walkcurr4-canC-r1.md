# cw-dynrep-criticD-walkcurr4-canC-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-18T10:17:16+00:00

**pod**: hexapod-mjx-train-5

**steps**: 4000000

**parent**: cw-dynrep-criticD-walkcurr3

**hypothesis**: Can the walking robot START walking upright if we only pay for upright progress and update the brain more aggressively? walkcurr3 is numerically healthy but stuck in a crouched shuffle (-65mm, height_factor 0.07, slip 4.6/m, zero B0 promotions) because the calibrated height/progress income gates were left OFF and its actor update (1e-4 x 3 epochs) is far below the proven acquisition recipe. Arm C of the operator's same-seed A/B/C tournament (fb_20260818T085648_2a0a60): gates ON (sigma 11mm, 25mm safety cutoff, kernel progress gate) + the strongest update tested, actor LR 3e-4 x 7 epochs, held until B0 promotes, then the default-off frontier-gated handover to 3 epochs / 1e-4->1e-5. Prediction-if-true: fastest B0 ignition of the three arms without KL rollbacks; if-false: KL blowups/rollbacks or crouch-termination. Strongest alternative: acquisition needs BC/gait-entry help, not LR. (-r1: first attempt raced a concurrent cycle onto train-11; relaunched on freed train-5.)

**gate**: Behavioral admission at 4M, judged on B0 cert telemetry (NOT crash-free): cmd_prog_frac>=0.50, height_factor>=0.8, falls==0 in final cert round, slip_per_m<=3.0, positive recent B0 progress slope; B0 promotion preferred. Rank passing arms by retention-clean frontier, then cmd_prog_frac, slip, roll. Winner contributes its RECIPE (fresh actor) to 40M cw-dynrep-criticD-walkcurr4; if all three fail, one evidence-based correction then re-canary — never launch the 40M on a failing recipe.

