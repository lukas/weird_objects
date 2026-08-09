# cw-walk-comshift-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T20:41:26+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-comshift30

**wandb_id**: xbuuygna

**hardware_ready**: no

**hypothesis**: Ruling-7 seed twin of today's comshift-dr05 PASS (c62). The dr05-compose class is split — comshift/deadband kept DR0 nominal retention, payload-dr05 (c61) charged it — so confirm the comshift compose pass is seed-robust before treating off-center payload as a deployable DR rung. ONE variable vs cw-walk-comshift-dr05: seed 0 -> 1, identical config/parent. Plain: same recipe, different dice - does it land in the same place? Prediction-if-true: own-cfg DR0.5+offset panel gv 12/12, 0 term, det med fwd >=1.1m AND DR0 retention in champion band (slip <=1.24, prog ~0.95) - compose pass is seed-robust. Prediction-if-false: retention erodes like payload-dr05 - the c62 pass was seed luck and the compose class verdict flips to axis-AND-seed dependent. Strongest alternative: own-cfg passes but retention lands between (1.24-1.38 slip) - widen the panel before concluding. Parent: cw-walk-comshift30.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.com_offset_m=0.03, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd >=1.1m; plus DR0 no-offset retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: PASS. Seed-1 twin confirms comshift-dr05 compose is seed-robust (ruling-7): own-cfg DR0.5+offset gv 12/12, 0 term, det med fwd 1.44m/sto med 1.44m (gate >=1.1m); DR0 no-offset retention CLEAN det gv 6/6, slip/m med 1.00, prog 0.95 (gate <=1.24). Honest caveat: det tail is seed-variable — s1 has 2/6 severe crater draws (fwd 0.75-0.79m, slip ~3.0, prog ~0.5) vs seed0's single mild dip (worst 1.23m/1.45) — same median/retention, worse worst-case this seed. Frames det: six legs cycling, level, no flag leg; crater draws show a slower shuffle, not a fall or drag. Paddle-slip lineage, not hardware-ready.

