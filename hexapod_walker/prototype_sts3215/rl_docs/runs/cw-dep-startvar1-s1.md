# cw-dep-startvar1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T06:04:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 18000000

**parent**: cw-dep-vref1-r1

**wandb_id**: elr18ype

**hardware_ready**: False

**hypothesis**: Seed twin of cw-dep-startvar1-r1 (P0 hardware candidate: contract-exact obs + 25deg envelope + placement-noise6 + bad-start0.4 + zero-drift-frame3deg + k_current=0, warm-started from vref1-r1). Given the closing hardware window, confirm the start-variation compose is a seed-robust recipe (not a lucky draw) before it becomes the hardware-attempt-#2 candidate. If-true: matches startvar1-r1's own-cfg gv/prog/slip band once both finish (no seed-driven pathology). If-false: seed-sensitive -- needs a wider varied-start eval panel before hardware promotion.

**gate**: Same gate as cw-dep-startvar1-r1 (own-cfg varied-start panel det+sto 6/6, DR0 contract retention, JOYSTICK-style flip/fall check); frames watched det

**verdict**: FAIL -- seed twin confirms cw-dep-startvar1-r1's breakdown is a real defect, not seed luck. Own-cfg det 0/6 pass (fwd 0.13-0.5m range; all degraded), gait_valid 6/6 this seed (no flag leg this draw, but slip/m up to 6.66 and prog_ratio med 0.52 -- systemic skate/paddle, not a clean gait). sto 3/6, prog 0.78, slip 1.88 -- same degraded band as r1's sto. Training reward quarters also DECLINE (438.9->486.7->416.0->347.1). Same conclusion as r1: the start-variation compose (placement6+badstart0.4+zerodrift-frame3+k_current=0) on the deployment contract breaks the gait; do not use for hardware. See cw-dep-startvar1-r1's verdict for the full analysis and root-cause hypothesis (zero_drift_cmd_frame prime suspect) -- isolation ablation cw-dep-startvar1-noZD1 running.

