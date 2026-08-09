# cw-walk-parkstart-mjx-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T05:26:01+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**wandb_id**: y8m62x7l

**hypothesis**: Consolidate-in-place discriminator for parkstart-mjx: the residual 1/6 partial sto park is an under-dosed-training artifact (parent segment got ~61 PPO updates vs the pre-registered ~325), not a synthetic-park distribution mismatch. Same config, zero new variables, 20M steps ~= 305 updates = update parity. If-true: standard 15s fwd 12/12 with park 0/12, park-exit >=10/12 retained, retention slip <=1.8 held. If-false: park persists ~1/6 at full parity -> dose is NOT the binding factor -> next arm harvests the policy's OWN park states as resets (distribution mismatch) or escalates to rung-2 time-averaged load evenness. Strongest alternative: more updates overfit park-starts and erode normal-start walking - distinguished by det fwd mean dropping below parent 0.745.

**gate**: 15s DR0 harness 6eps/mode det AND sto: fwd >=0.40m 12/12 AND gait_valid 12/12 AND >=2 swings/leg AND 0 term AND no final-third degradation AND det fwd mean >=0.50m; park-exit eval at walk_park_start_frac=1.0: >=10/12 fwd >=0.30m AND gait_valid; retention 5s det slip/m <=1.8

**verdict**: FAIL (15s fwd 11/12 vs 12/12 — sto[5] partial park PERSISTS at full update parity, fwd 0.203, duty [0.85,0.13,0.82,0.22,0.79,0.19]; park-exit clause MET 11/12; retention slip 1.516<=1.8 MET; det fwd mean 0.712 vs parent 0.745, 5/6 seeds lower — mild slide). NOT HARDWARE-READY (1/6 sto park, skating slip/m ~1.4-1.6, DR0 only). HYPOTHESIS REFUTED: dose is NOT the binding factor; if-false branch (a) fires — synthetic exits work, own park stays -> harvest own-park resets. Champion UNCHANGED (parkstart-mjx 01d9ab60).

