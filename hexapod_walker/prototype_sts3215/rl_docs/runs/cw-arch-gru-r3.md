# cw-arch-gru-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T16:53:52+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**wandb_id**: seq03hmp

**hardware_ready**: no

**hypothesis**: GRU rung RECIPE CHANGE after the r1/r2 leg-sacrifice/paddle cheat (both FAILED on harness forensics): r1/r2 ran the DEFAULT reward config without the anti-cheat machinery the walk lineage needed for exactly this exploit. r3 = from scratch, same mixed diet (walk 0.60 / rise 0.15 / lower 0.15 / hold 0.10), but with the full hist16-r7 anti-cheat + joystick stack (step-event income, drag/park penalties, prog+anchor gates, narrow 0.05-0.06 band, cmd resample jitter) and single-frame obs — the GRU replaces the 16-frame stack as the ONE arch variable vs hist16-r7. Discovery question: with paddle/park income shut off, does a NON-CHEAT walk attempt emerge alongside rise/lower/hold in one recurrent policy?

**gate**: PASS if the 2M harness forensics (DR0 det+sto walk, standard fingerprint check) show NO leg-sacrifice (det gait_valid > 0/6, no parked-leg collapse) AND positive det median progress ratio AND stance still emerging (>=1 rise completion, tipped >=1/2). Walk speed tracking NOT required at 2M — the question is cheat-free gait attempt, not gait quality. FAIL if the same cheat fingerprint reappears under the gates: then the lever is BPTT window / capacity, not reward, and the rung goes back to frozen.

**verdict**: FAIL (known exploit, no forensics needed): identical leg-sacrifice/paddle fingerprint as r1/r2 SURVIVES the full hist16-r7 anti-cheat reward stack (step-event income, drag/park penalties, prog+anchor gates, narrow band) -- det gait_valid 0/6, legs [0,3] parked at duty 0.02 (video-confirmed static legs, speed 0.005-0.006 m/s), sto gait_valid 6/6 but no-progress jitter-paddle (prog_ratio 0.01-0.05, slip/m 17.4-17.5). Own-DR0.5 pass matches (legs [0,1,3] parked). Gate's own pre-registered FAIL branch fires exactly: since the anti-cheat reward already works on the MLP walk lineage, the cheat's return here is a GRU/BPTT-window or capacity limitation, not a reward-pricing gap. GRU rung stays FROZEN per RL_PLAN (off the blocker list, pending the flagship); no further recipe/diet/step-count variant on this rung.

