# cw-walk-deadband-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T18:37:38+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-deadband30

**wandb_id**: f1ra5y09

**hardware_ready**: no

**hypothesis**: Composition rung off the fresh deadband30 PASS: deadband tolerance was proven at dr-scale 0.0 only. One variable off deadband30: dr-scale 0.0 -> 0.5 while keeping dr.deadband_scale=1.0,3.0. Plain: tolerating sloppy servos must survive real-world physics spread too. If-true: own-cfg DR0.5+deadband harness gv 12/12, 0 term, det median fwd >=1.1m @30s and DR0 retention stays in champion band - deadband becomes a deployable robustness rung. If-false: DR draws plus wide deadband break the fine-correction paddle (terminations, or jerky overdrive appears in frames that nominal deadband30 avoided) - deadband stays a nominal-sim skill; note the compose ceiling. Strongest alternative: passes scalars with slip creep past champion band - caveat is slip, not gait.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.deadband_scale=1.0,3.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1 m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det for lurching

**verdict**: PASS with caveat. DR0.5 + deadband 1-3x compose holds: own-cfg gv 12/12, 0 term, det med fwd 1.38m (gate 1.1), no jerky overdrive or lurch in frames; DR0 nominal retention gv 6/6, prog 0.97, slip/m 1.22 — inside the 1.24 cap but at its edge, and fwd 1.42m vs champion 1.57m (mild nominal shading). Deadband tolerance is now a DR-robust rung. Paddle lineage, not hardware-ready.

