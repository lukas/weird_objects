# cw-gait-terrain2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T17:53:29+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 19yjucye

**hardware_ready**: False

**hypothesis**: GAIT CLEANUP P3 lever 1 RELAUNCH at TRUE amplitude (supersedes INVALID cw-gait-terrain1, which trained on ground silently clamped to 18mm by the servo_model.py terrain_amp clip, fixed 434a6e0). Terrain-as-teacher: FROM SCRATCH on env.terrain_amp=4.0 = 72mm peak bumps, where the champion-on-terrain probe (logs/terrain_probe2, train-5, 08-11) shows the PADDLE FAILS THE WALK GATE OUTRIGHT: 0/6 success, prog_ratio 0.80, slip/m 1.46 (vs 6/6 prog 0.85 at 36mm; every historical terrain run was really 18mm). On this ground dragging cannot complete the task, so plain progress income selects lift-and-place with no reward surgery, and no paddle habit exists to break (scratch). The hfield spawn disk is flat with bumps fading in over ~0.3-0.6m — a built-in curriculum: easy travel near spawn, stepping required beyond.

**gate**: By 2M: completes walk episodes on its own 72mm terrain (success >0/6 where the champion scores 0/6 -- ANY honest success beats the paddle ceiling) AND flat-DR0 retention slip/m < 0.6 (champion band 1.1-1.5) at in-band travel. Kill signatures (pre-registered): (a) no travel at all by 2M -- 72mm too hard from scratch, single retry at amp 3.0 (54mm) allowed; (b) travels on bumps but flat retention paddles at slip >1.0 -- stepping does not transfer to flat, lever refuted, drag-charge-annealed-up (GAIT.md P3.2) is next; (c) learns to travel only inside the flat spawn disk and parks at the bump fence -- park-and-earn variant, treat as (a).

**verdict**: FAIL, worse than kill signature (a) -- from-scratch training on true 72mm terrain does NOT discover lift-and-place; it collapses into a WORSE pathology than the closed paddle: leg-sacrifice (leg 3 parked at low duty, gait_valid only 2/6 det). Own-terrain gate: det slip/m med 8.58 (range 6.6-15.3), fwd med 0.23m/15s, prog_ratio med 0.29; flat-DR0 retention is essentially identical (det slip 6.83, gait_valid 2/6, same leg[3] sacrifice) -- both 4-10x worse than the closed champion band (slip 1.1-1.5) and worse than dragstep1's already-failed 6.36. Confirms + extends the zero-training probe (probe2, train-5: champion zero-shot fails the walk gate 0/6 at 72mm+108mm with slip flat at 1.46-1.50, never dropping) -- physics does not force stepping even for an established paddler, and from-scratch it finds an even cheaper cheat (sacrifice one leg) before ever discovering the harder six-leg lift-and-place skill. Pre-registered single retry queued at env.terrain_amp=3.0 (54mm) per the run's own gate text -- cw-gait-terrain2-r1, VERIFIED RUNNING train-3.

