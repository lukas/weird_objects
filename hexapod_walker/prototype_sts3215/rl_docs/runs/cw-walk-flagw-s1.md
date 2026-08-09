# cw-walk-flagw-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**pod**: hexapod-sweep-s4

**parent**: ppo_goal_cw_walk_dr04b.zip

**wandb_id**: jsjc65dd

**checkpoint**: pod s4: rl_move/sim/policies/ppo_goal_cw_walk_flagw_s1.zip (md5 048643996f3eb4406965d6ba2b8619f7)

**hypothesis**: seed twin of cw-walk-flagw (run variance + best-of-2)

**gate**: same as cw-walk-flagw

**verdict**: FAIL (cycle 11): walk det 0/6 gait-valid @ vel_err 0.036, sto 0/6 @ 0.033; leg 3 parked (duty 0.03-0.05) in 10/12 episodes, 2 sto episodes flip to the tripod anchor (legs 1/3/5 airborne) - same two exploit modes as seed-0. Retention: rise 5/6 det + 6/6 sto, raise 4/6, track 5/6; lower 0/6 det (inherited lineage defect, see dr04b control). Video: NOT WALKING; rise ends propped with flagged legs; lower never descends. NOT HARDWARE-READY. Twin agreement closes the flag-leg penalty at any routing. Champion unchanged.

