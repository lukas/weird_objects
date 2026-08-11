# cw-gait-dragstep1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T17:34:05+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-dep-vref1-r1

**wandb_id**: pq1unuzb

**hypothesis**: GAIT CLEANUP P3 lever 2, big drag charge FROM SCRATCH + dense step income (operator 08-11, rl_docs/GAIT.md): the operator premise is that a genuinely expensive drag should force stepping if the task is learnable early. The closed anti-slip arms retrofit penalties onto FORMED paddlers (habit + starvation failures); this arm never lets the paddle form: from scratch, k_drag_loaded 10->40 (4x the stock charge, applied from step 0), PLUS reward.k_step_event=1.0 -- the one-shot per-leg credit for a COMPLETED lift->swing->touchdown (>=10mm along command) that birthed the first genuine six-leg gait (n lineage, 08-08). The step credit is the make-it-easier half: it pays a dense, unfarmable income for exactly the behavior we want while the drag charge makes the alternative expensive, so exploration has a gradient toward lifting before the paddle local-optimum is discovered.

**gate**: By 2M: travels (fwd distance in the parent joystick band, zero falls) with slip/m at DR0 < 0.6 (champion band 1.1-1.5). Kill signatures (pre-registered): (a) park-and-earn -- fwd ~0 while step-event income is farmed by in-place stepping (then the along>=10mm projection gate is leaking and needs the P2 bank, not a coef change); (b) paddle forms anyway at slip>1.0 despite the 4x charge -- pricing refuted even from scratch, terrain/physics levers are the survivors; (c) no travel and no stepping -- charge too punishing at 40, one rung down (20) is the single allowed retry.

