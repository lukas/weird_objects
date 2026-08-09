# cw-walk-nv2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**pod**: hexapod-sweep-long5m

**parent**: ppo_goal_cw_walk_nv.zip

**wandb_id**: e58w8dos

**checkpoint**: pod long5m: rl_move/sim/policies/ppo_goal_cw_walk_nv2.zip (md5 97a7e553bf060f5b420a8c87d8bf1bb2)

**hypothesis**: deployable-obs baseline needs its full fixed 8M budget (review section 6: do not extend past 8M unless aac fails to dominate)

**gate**: sto walk >=4/6 @ vel_err <=0.035 at 8M cum; second miss calls the baseline at whatever 8M gives

**verdict**: FAIL at 8M (cycle 11): walk det 0/6 gait-valid @ vel_err 0.034, sto 0/6 @ 0.035, speed 0.029 m/s; leg 3 parked (duty 0.02-0.07) 11/12 episodes + one sto tripod anchor. rise det 4/6 / sto 3/6 (flat weak); raise 5/6; lower 0/6 (inherited); track 5/6. Video: NOT WALKING - antenna leg parked entire episodes; recorded rises never stand. No metric moved vs the 4M mark beyond eval noise. Baseline CALLED per fixed-budget rule: sto walk 0/6 gait-valid @ 0.035 is the bar aac must beat at 28.76M cum. NOT HARDWARE-READY. nv line closed.

