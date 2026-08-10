# cw-arch-hist16-dep1-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T22:05:25+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-arch-hist16-dep1

**wandb_id**: 2zle309u

**hardware_ready**: False

**hypothesis**: Plain English: continue the hardware-deployment-contract 16-frame walker to close its efficiency gap. cw-arch-hist16-dep1 just proved the 16-frame temporal architecture bootstraps a clean, valid six-leg walking gait from scratch directly on the real robot's sensing contract (0 falls, 0 terminations, 12/12 gait_valid) -- but its foot-slip (1.41-1.48/m) and speed-tracking are not yet inside the current hardware candidate vref1-r1's own band (0.89-1.36). The sibling architecture line (cw-arch-hist16-r7) closed an identical gap (1.43 -> 1.13-1.16) purely through two +40M continuations, no reward change. One variable vs the parent: +40M more steps, same recipe, nothing else changed. If-true: slip/m moves into or near the vref1-r1 band and vel-tracking success improves -- the dep-contract candidate becomes a real hardware-ladder rung. If-false: slip/m plateaus outside the band despite the extra steps -- the gap is structural (contact-pricing/architecture), not a training-budget question, and the dep line needs a different lever before another blind continuation.

**gate**: own-cfg DR0.5 det+sto 6/6 gv, 0 term, prog med >=0.85; slip/m det improves or holds vs dep1's 1.41 (not regressing), sto vs dep1's 1.45; DR0 gate det+sto 6/6 gv, 0 term; JOYSTICK GATE (eval_drive DR0.2) 0 falls; video clean six-leg cycling, no flag leg

**verdict**: PASS — +40M continuation closed the dep-contract 16-frame walker's economy gap into (det) or to the edge of (sto) vref1-r1's own slip band: slip/m med 1.20-1.21 det, 1.35-1.37 sto (was 1.41-1.48 pre-continuation, band is 0.89-1.36). Both DR passes 6/6 gait_valid, 0 terminations, prog med 1.07-1.17 (>=0.85 gate). JOYSTICK GATE (eval_drive DR0.2, run live on the pod, not pre-staged) PASS: 0 falls across the full direction panel + flip stress. Video (all 12 det/sto episodes, both DR0 and DR0.5) shows ordinary six-leg swing/stance cycling, no flag-leg/drag/skate. Vel-tracking success also improved from a coin-flip to 4-5/6 per pass. Confirms the if-true branch: the dep-contract candidate is now a real hardware-ladder rung alongside vref1-r1, closing via continuation alone (same pattern as the r7 sibling line) rather than needing a reward change.

