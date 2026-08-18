# cw-dynrep-criticD-walkcurr4-canA-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-18T09:42:03+00:00

**pod**: hexapod-mjx-train-7

**steps**: 4000000

**parent**: cw-dynrep-criticD-walkcurr3

**hardware_ready**: False

**hypothesis**: Can the walking robot START walking upright if we only pay for upright progress and update the brain more aggressively? walkcurr3 is numerically healthy but stuck in a crouched shuffle (-65mm, height_factor 0.07, slip 4.6/m, zero B0 promotions at 7.5M) because the calibrated height/progress income gates were left OFF and its actor update (1e-4 x 3 epochs) is far below the proven acquisition recipe. Arm A of the operator's same-seed A/B/C tournament (fb_20260818T085648_2a0a60): gates ON (sigma 11mm, 25mm safety cutoff, kernel progress gate) + actor LR 2e-4 x 5 epochs held until B0 promotes, then the default-off frontier-gated handover to 3 epochs / 1e-4->1e-5. Prediction-if-true: B0 certs upright (height_factor>=0.8) with cmd_prog_frac>=0.5 by 4M; if-false: still crouch-terminates or KL blowups force rollbacks. Strongest alternative: acquisition needs BC/gait-entry help, not LR. (-r1: attempt 1 died at boot on train-6, frozen encoder absent there; encoder md5-verified on train-7 before this relaunch.)

**gate**: Behavioral admission at 4M, judged on B0 cert telemetry (NOT crash-free): cmd_prog_frac>=0.50, height_factor>=0.8, falls==0 in final cert round, slip_per_m<=3.0, positive recent B0 progress slope; B0 promotion preferred. Rank passing arms by retention-clean frontier, then cmd_prog_frac, slip, roll. Winner contributes its RECIPE (fresh actor) to 40M cw-dynrep-criticD-walkcurr4; if all three fail, one evidence-based correction then re-canary — never launch the 40M on a failing recipe.

**verdict**: No training verdict - launch infra failure: MjxShardedVecEnv workers SIGBUS-killed by the old 64M /dev/shm on pre-dshm-spec train-7 (CAPACITY.md caveat). Pod recreated with 4Gi dshm; superseded by canA-r2.

**failed_reason**: run never appeared as 'running' in W&B within 240s

