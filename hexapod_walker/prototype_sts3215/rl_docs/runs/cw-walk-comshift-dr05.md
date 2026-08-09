# cw-walk-comshift-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T18:38:57+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-comshift30

**wandb_id**: o6rhyyvg

**hardware_ready**: no

**hypothesis**: Composition rung off the fresh comshift30 PASS (same move class as payload-dr05/lowgait-dr05): off-center-payload competence was proven at dr-scale 0.0 only. One variable off comshift30: dr-scale 0.0 -> 0.5 while keeping the absolute dr.com_offset_m=0.03 override. Plain: carrying weight on one side must survive real-world physics spread, not just nominal sim. If-true: own-cfg DR0.5+offset harness gv 12/12, 0 term, det median fwd >=1.1m @30s and DR0 nominal retention stays in champion band - asymmetric payload becomes a deployable robustness rung. If-false: DR draws plus offset mass tip the gait (terminations or the half-speed shuffle spreads from worst draw to median) - comshift stays a nominal-sim skill; note the compose ceiling. Strongest alternative: holds but slip creeps past champion band under DR like other composes - then the caveat is slip, not gait.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.com_offset_m=0.03, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1 m; plus DR0 no-offset retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: PASS. DR0.5 + com_offset 30mm compose holds: own-cfg gv 12/12, 0 term, det med fwd 1.44m (gate 1.1); DR0 no-offset retention CLEAN (gv 6/6, prog 0.95, slip/m 0.98, fwd 1.49m = champion band — this compose did NOT charge nominal, unlike payload-dr05 c61). Frames: six legs cycling, level, no lurch. Paddle-slip lineage, not hardware-ready. Seed twin queued (comshift-dr05-s1).

