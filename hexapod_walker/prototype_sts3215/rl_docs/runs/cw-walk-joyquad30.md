# cw-walk-joyquad30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T04:54:34+00:00

**pod**: hexapod-mjx-train-10

**steps**: 12000000

**parent**: cw-walk-joylat25

**wandb_id**: bslsafxo

**hardware_ready**: False

**hypothesis**: QUAD INTO THE MAINLINE (operator 08-10 00:4x: 'four leg trick in the main line so I can hit it with the joystick'): compose quad-hold (30% mix; 50% eroded walk in quad-hold1-r2, if-false branch) onto the DRIVING champion joylat25 (ruling-7 panel complete, DR0.5+latency+abrupt flips) instead of the plain walk champion. Command = two-hot goal one-hot, obs unchanged, warm start exact. One checkpoint that drives AND lifts fronts on a joystick button (drive_policy.py key 4 wired). If false: quad mix erodes the joystick gate -> ladder mix down to 0.2 or train quad-entry only from stops.

**gate**: JOYSTICK GATE @DR0.2 retained (0 in-envelope falls incl. flip-stress) AND own-cfg walk-mode det slip/m within joylat25 band (<=1.55) AND quad-mode: fronts_off >= 0.9, mean clear >= 20 mm, planted_frac >= 0.95 over final 10 s in >= 10/12 eps, |roll|,|pitch| <= 4 deg, 0 term

**verdict**: FAIL on the walk-retention leg of the compound gate (if-false branch: quad mix erodes walk-mode slip). JOYSTICK GATE @DR0.2 PASS (0 in-envelope falls incl. flip-stress). Quad-hold mechanism itself is solid: training's own eval/quad telemetry holds survived_frac 1.0 throughout with height_err_end 2-9mm and track_err <1.1deg at every checkpoint, and rollout video (10M-step, all-4 eval eps quad) shows a clean level stance with both front legs raised clear (~12-14mm) and the rear four planted, no tipping. But own-cfg walk-mode harness eval (DR0.5+latency compose, matching joylat25's own gate config) shows slip/m med 1.72 det / 1.72 sto vs the pre-registered cap 1.55 (parent joylat25's own band was 1.48/1.51) -- all 6 det episodes sit 1.32-1.92, a real median-level shift not a single outlier, though gait_valid 12/12 and 0 terminations (frames show no visible flag-leg/dragging -- the slip inflation is a numbers-only degradation like the lineage's known fixed-draw pattern, invisible on casual video). Even a light 30% quad mix (vs quad-hold1-r2's 50%) meaningfully erodes walk-mode foot-slip on the driving line -- second dose-response data point for the quad-mix-erosion finding (P0 ruling 7).

