# cw-dep-bcgait1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-12T00:18:16+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tip1

**wandb_id**: 3wqhp82f

**hardware_ready**: False

**hypothesis**: BC-INIT LINE, arm 1 (bc_init_gait.py landed 08-11 night, operator session): the policy is initialized by supervised cloning of the SCRIPTED TRIPOD GAIT — the controller that walks tall in sim (existence proof, probe_tall_wall +12mm) AND walked the real robot in the tape sessions — with DART noise injection (clean-label noisy-execution demos; without it the clone mean-collapses, measured 11mm/15s). The init walks TALL in closed loop (+14..+29mm mid-gait by probe, 177mm footprint, yaw 30deg off the limit - every RL-bred policy: -72..-75mm, splayed, yaw-pinned) and travels 238mm/825mm cmd on its best seed, stalling in-place on others: it sits on the travel/stall boundary, which is exactly what RL income selection fixes. Mechanism is DISJOINT from every closed lever: not an anchor during RL (gaitbc1 froze - an init cannot be satisfied by freezing), not champion distillation (arch-gru clones the paddle), not pricing (6 arms flat), not state injection (rsi1 dove back because its ACTIONS from tall states were bad - this init IS the actions). Recipe: tip1 dep stack + walk_height_gate sigma30 at ref 0 (tall = the paid behavior, so fine-tune income pulls TOWARD the init, unlike ft1/ft2 where BC skills were income-orphaned) + target-kl 0.02 (fresh value function protection).

**gate**: Primary (probe_tall_wall, binding tall-campaign metric): steady-state walking height >= -20mm AND consistent travel (all 3 probe seeds move, det travel >= 200mm/15s at 0.055 cmd). Secondary: eval walk survived 1, gait_valid det+sto, slip <= 1.8, no park. JACKPOT: height >= -10mm + speed >= 0.035 = the tall deployable walker, straight to Gate 0 export + tipped retention + bench. FAIL modes to distinguish: (a) reverts to the -72mm crouch = walk income overwhelms the init even with the gate (then retry with lr 1e-4 + tighter kl ONCE); (b) stalls tall without travel = income too weak to select travel (then raise walk kernel weight or drop target-kl).

**verdict**: PASS on the binding primary metric -- the crouch-splay tall-wall is BROKEN for the first time. probe_tall_wall steady-state walking height is -10 to +6mm across 3 seeds (every prior RL-bred walker sat at -72 to -75mm), leg yaw margin is now POSITIVE 17-18deg (every prior arm was pinned negative at the 35deg splay limit) -- the policy is no longer buying stability with a crouch+splay. Harness confirms real forward travel, not a stall: det prog_ratio 0.77, speed 0.067 m/s, gait_valid 6/6 all six episodes, roll settles clean (tail 0.4-1.4deg, all recover from transient peaks up to 17.5deg, zero falls). Video (all 6 det episodes) shows a visibly taller, genuinely walking gait, not a paddle. Caveat: the run's own SECONDARY bar (slip<=1.8) is missed (det 2.12, sto 12.39 with one sacrificed leg in 1/6 sto episodes) -- this is a real existence-proof win on the top-priority tall-wall question, not yet a polished/robust deployable candidate. Mechanism confirmed as hypothesized: BC-INIT (pure action pretraining on the scripted tall gait before any RL) is disjoint from every closed pricing/anchor/RSI lever because it fixes the ACTIONS from tall states directly, where all of those fixed only prices or states.

