# probe-posture-price2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-08T22:01:31+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**hypothesis**: Probe (audit sec6) for new posture-pricing mechanism: k_support_margin=0.3 + k_load_even=1.5 on stance-champ init at DR 1.0. If-true: both reward parts appear in W&B within audited band (margin<=0.3/step, even<=0.25/step), canaries protected, trainer healthy. If-false: term explosion, canary group failure, or crash. Relaunch of probe-posture-price after callback fix 88cd50a.

**gate**: mechanical only: reward_support_margin and reward_load_even present in W&B rollup within audited band; no canary group failure; no behavioral claim at 150k

