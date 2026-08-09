# cw-walk-joyjit-dr05-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:00:47+00:00

**pod**: hexapod-mjx-train-8

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05

**wandb_id**: y84rtaks

**hardware_ready**: no

**hypothesis**: REBALANCE continuation (not a new variable): cw-walk-joyjit-dr05 was killed at ~2M/20M because node g142d86 was host-starved (load 216/128, fps 2.1-2.9k). Same hypothesis as parent: abrupt randomized command resampling (1.5s +-60% jitter, blend 0.1-1.0s, 20% stops) hardens joystick-style flips while KEEPING DR 0.5; controlled pair with cw-walk-joystick45 (same package at DR0). If-true: JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls AND own-cfg DR0.5 harness gv 12/12, 0 term. If-false: abrupt resampling under DR degrades the base gait - hardening needs a curriculum, not joint exposure.

**gate**: JOYSTICK GATE: python3 -m rl_move.sim.eval_drive <ckpt> --dr-scale 0.2 own cfg - ZERO in-envelope falls across panel + flip stress; plus own-cfg DR0.5 harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.85; frames watched det

**verdict**: PASS. JOYSTICK GATE (eval_drive DR0.2, own cfg): 0 in-envelope falls across fwd/diag/stop-go panel + 3 flip-stress eps (trk_err 0.025-0.056; backward cmd parks at 0.026m, doesn't fall). Own-cfg DR0.5 harness 6+6: gv 12/12, 0 term, prog med 0.94 det / 0.98 sto (gate >=0.85), slip/m med 1.38/1.44 = steering-lineage band (wander-dr05 1.39-1.70). DR0 retention gv 12/12, prog 0.97/0.95. Frames det: level, all six legs cycling, no flag leg; paddle foot-slide persists. Abrupt-resample flip hardening and DR0.5 COMPOSE (controlled pair with joystick45 at DR0) - strongest driving candidate so far. Not hardware-ready (contact-pricing slip root).

