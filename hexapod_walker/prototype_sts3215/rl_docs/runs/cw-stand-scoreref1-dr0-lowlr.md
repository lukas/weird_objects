# cw-stand-scoreref1-dr0-lowlr

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T01:43:44+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-scoreref1-dr0

**hardware_ready**: False

**hypothesis**: scoreref1-dr0 exonerated DR as the root blocker and exposed the real one: at DR0 the warm start BEGINS with the crutch engaged (reward_rise_ref 0.65/tick, feet factor 0.87, rent ~0 at the first W&B sample) and PPO training ERODES it to 0.02/0.20/-0.46 -- a paying behavior is being destroyed by updates, not undiscovered. Prime suspect: the warm-started critic was fitted to the stance champion's old reward stack; under the new score-routed reward its early advantages are miscalibrated and the first updates stomp the aligned behavior before the critic recalibrates (classic warm-start/changed-reward erosion). Zero-code test: cut LR 3e-4 -> 5e-5 so early updates are too small to destroy the start while the critic refits. ONE change vs scoreref1-dr0.

**gate**: W&B env/reward_rise_ref must HOLD near its warm-start level (>=0.3/tick median over the run; erosion to <0.1 by mid-run = FAIL fast) and env/rise_score must climb off 0.1; harness: rise >=4/6 det valid_plant, lower retains >=5/6. If tracking holds but rise_score stalls below ~0.3, the residual gap is the score's top factors and the next knob is annealing (more steps at low LR), not another mechanism.

**verdict**: FAIL — pre-registered fail-fast trigger hit: env/reward_rise_ref crashed from its 0.66/tick warm-start value to ~0.002-0.05/tick by step 18-32 (a handful of updates in) and stayed there (run median 0.021, gate needed >=0.3 median); env/rise_score never climbed off the floor (median 0.021, max 0.096, gate needed >0.1). This REFUTES the LR-erosion hypothesis this run tested: cutting LR 3e-4->5e-5 (6x smaller updates) did NOT slow the collapse -- it happens on essentially the same few-step timescale as the un-throttled run. So the aligned start is not being destroyed by oversized gradient steps; either the score-routed reward genuinely disfavors the tracked behavior once summed with the rest of the stack, or (untested) the collapse is a rollout-stochasticity artifact (tight 6-deg sigma kernel vs sampled actions, not a true policy-mean drift). Per this run's own pre-registered fallback: the next lever is per-stream reward forensics, not another LR/coefficient variant -- and RISE.md's binding ruling already closes further income/reference-tracking variants; no follow-up launched here.

**refused_reason**: hexapod-mjx-train-6 already runs cw-stand-scoreref1-dr0-lowlr — GPU pods host exactly one run; pick a free GPU pod.

