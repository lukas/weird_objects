# cw-arch-hist16-dep1-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T22:05:25+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-arch-hist16-dep1

**wandb_id**: 2zle309u

**hypothesis**: Plain English: continue the hardware-deployment-contract 16-frame walker to close its efficiency gap. cw-arch-hist16-dep1 just proved the 16-frame temporal architecture bootstraps a clean, valid six-leg walking gait from scratch directly on the real robot's sensing contract (0 falls, 0 terminations, 12/12 gait_valid) -- but its foot-slip (1.41-1.48/m) and speed-tracking are not yet inside the current hardware candidate vref1-r1's own band (0.89-1.36). The sibling architecture line (cw-arch-hist16-r7) closed an identical gap (1.43 -> 1.13-1.16) purely through two +40M continuations, no reward change. One variable vs the parent: +40M more steps, same recipe, nothing else changed. If-true: slip/m moves into or near the vref1-r1 band and vel-tracking success improves -- the dep-contract candidate becomes a real hardware-ladder rung. If-false: slip/m plateaus outside the band despite the extra steps -- the gap is structural (contact-pricing/architecture), not a training-budget question, and the dep line needs a different lever before another blind continuation.

**gate**: own-cfg DR0.5 det+sto 6/6 gv, 0 term, prog med >=0.85; slip/m det improves or holds vs dep1's 1.41 (not regressing), sto vs dep1's 1.45; DR0 gate det+sto 6/6 gv, 0 term; JOYSTICK GATE (eval_drive DR0.2) 0 falls; video clean six-leg cycling, no flag leg

