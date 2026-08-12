# cw-dep-bcgait1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-12T00:18:16+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tip1

**hypothesis**: BC-INIT LINE, arm 1 (bc_init_gait.py landed 08-11 night, operator session): the policy is initialized by supervised cloning of the SCRIPTED TRIPOD GAIT — the controller that walks tall in sim (existence proof, probe_tall_wall +12mm) AND walked the real robot in the tape sessions — with DART noise injection (clean-label noisy-execution demos; without it the clone mean-collapses, measured 11mm/15s). The init walks TALL in closed loop (+14..+29mm mid-gait by probe, 177mm footprint, yaw 30deg off the limit - every RL-bred policy: -72..-75mm, splayed, yaw-pinned) and travels 238mm/825mm cmd on its best seed, stalling in-place on others: it sits on the travel/stall boundary, which is exactly what RL income selection fixes. Mechanism is DISJOINT from every closed lever: not an anchor during RL (gaitbc1 froze - an init cannot be satisfied by freezing), not champion distillation (arch-gru clones the paddle), not pricing (6 arms flat), not state injection (rsi1 dove back because its ACTIONS from tall states were bad - this init IS the actions). Recipe: tip1 dep stack + walk_height_gate sigma30 at ref 0 (tall = the paid behavior, so fine-tune income pulls TOWARD the init, unlike ft1/ft2 where BC skills were income-orphaned) + target-kl 0.02 (fresh value function protection).

**gate**: Primary (probe_tall_wall, binding tall-campaign metric): steady-state walking height >= -20mm AND consistent travel (all 3 probe seeds move, det travel >= 200mm/15s at 0.055 cmd). Secondary: eval walk survived 1, gait_valid det+sto, slip <= 1.8, no park. JACKPOT: height >= -10mm + speed >= 0.035 = the tall deployable walker, straight to Gate 0 export + tipped retention + bench. FAIL modes to distinguish: (a) reverts to the -72mm crouch = walk income overwhelms the init even with the gate (then retry with lr 1e-4 + tighter kl ONCE); (b) stalls tall without travel = income too weak to select travel (then raise walk kernel weight or drop target-kl).

