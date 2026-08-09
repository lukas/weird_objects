# probe-posture-price

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED_RELAUNCH

**created**: 2026-08-08T21:58:15+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**hypothesis**: Probe (audit sec6) for new posture-pricing mechanism: k_support_margin=0.3 + k_load_even=1.5 on stance-champ init at DR 1.0. If-true: both reward parts appear within audited band (margin<=0.3/step, even<=0.25/step), canaries protected, trainer healthy. If-false: term explosion, canary group failure, or crash.

**gate**: mechanical only: reward_support_margin and reward_load_even present in training logs within audited band; no canary group failure; no behavioral claim at 150k

**verdict**: KILLED @~30k (cycle 12): trainer W&B callback lacked the new reward part keys, so the probe's own mechanical gate (terms visible in logs) was unverifiable. Callback patched (88cd50a), relaunched as probe-posture-price2. Not a scientific verdict.

