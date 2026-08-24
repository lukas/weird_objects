# cw-arch-hist16-dep1-c1-joyfullcurr10-stopfreeze-probe-stopcur6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-24T09:07:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr10-stopfreeze-probe

**hypothesis**: Plain English: the just-verdicted stopfreeze-probe proved the structural stop-hold (goal.walk_stop_freeze_s=0.4) clears the V6 b1 stop cert outright on the k=2.0 current-charge checkpoint (stopcur2). stopcur6 is the dose sibling (k=6.0 current charge) that solves over_current identically (1/48 joygate falls) but carries a KNOWN separate pathology the current-charge dose introduced: a leg-3 rigid-lock/sacrifice trade under own-DR (owncfg det gait_valid 5/6, one leg statically loaded). Since the freeze mechanism forces a hold instead of letting the policy fight to a stop, it might ALSO reduce or remove that isometric leg-3 lock (which is itself a stop-adjacent fight), not just the speed-cert creep. Same cheap (~15s, no PPO) --walkcurr-precert-only dry run as the original probe, just reading stopcur6 instead of stopcur2, before committing a full 40M training budget to a stopcur6+freeze lineage.

**gate**: If walkcurr/pre_b1_stop_speed_m_s <= 0.015 with the freeze on (matching stopcur2's result): the hold generalizes across current-charge doses -- worth a real training continuation exactly like freeze40 but from stopcur6, AND worth checking (via the existing own-DR eval, not this diagnostic) whether pre_b1's other metrics hint the leg-3 lock is also affected. If it stays above 0.015 or degrades vs stopcur2's read: the hold's benefit is dose-conditional -- do not extend the freeze training continuation to the stopcur6 lineage without digging into why.

**failed_reason**: run never appeared as 'running' in W&B within 240s

