# cw-arch-hist16-dep1-c1-joyfullcurr10-stopfreeze-probe-stopcur6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-24T09:07:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr10-stopfreeze-probe

**hypothesis**: Plain English: the just-verdicted stopfreeze-probe proved the structural stop-hold (goal.walk_stop_freeze_s=0.4) clears the V6 b1 stop cert outright on the k=2.0 current-charge checkpoint (stopcur2). stopcur6 is the dose sibling (k=6.0 current charge) that solves over_current identically (1/48 joygate falls) but carries a KNOWN separate pathology the current-charge dose introduced: a leg-3 rigid-lock/sacrifice trade under own-DR (owncfg det gait_valid 5/6, one leg statically loaded). Since the freeze mechanism forces a hold instead of letting the policy fight to a stop, it might ALSO reduce or remove that isometric leg-3 lock (which is itself a stop-adjacent fight), not just the speed-cert creep. Same cheap (~15s, no PPO) --walkcurr-precert-only dry run as the original probe, just reading stopcur6 instead of stopcur2, before committing a full 40M training budget to a stopcur6+freeze lineage.

**gate**: If walkcurr/pre_b1_stop_speed_m_s <= 0.015 with the freeze on (matching stopcur2's result): the hold generalizes across current-charge doses -- worth a real training continuation exactly like freeze40 but from stopcur6, AND worth checking (via the existing own-DR eval, not this diagnostic) whether pre_b1's other metrics hint the leg-3 lock is also affected. If it stays above 0.015 or degrades vs stopcur2's read: the hold's benefit is dose-conditional -- do not extend the freeze training continuation to the stopcur6 lineage without digging into why.

**verdict**: Structural stop-hold (goal.walk_stop_freeze_s=0.4) generalizes across current-charge dose: precert-only dry run (~15s, no PPO) on the UNCHANGED stopcur6 checkpoint (k_walk_stop_current=6.0) reads b1 front45_20s stop_speed_m_s=0.0133 (settled 0.0044, settled_frac 0.80), under the 0.015 cap, matching stopcur2's post-freeze read to 3 decimals; b0 bridge_10s clean too (prog 1.05-1.09, falls 0, roll 2.3-3.1deg). Evidence: the residual post-freeze creep floor is a hold-mechanics/physics property, not checkpoint-specific -- two different reward-priced checkpoints (k=2.0 vs k=6.0 current charge) land on the identical number once the freeze overrides their command. Gate met per the pre-registered text (<=0.015 matching stopcur2) -> justifies a real training continuation from stopcur6 exactly like freeze40, plus checking via own-DR eval whether the freeze also reduces stopcur6's known leg-3 rigid-lock pathology (not testable by this diagnostic alone). Next: launching cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40-stopcur6 (stopcur6 base, same freeze cfg as freeze40) this cycle.

