# cw-walk-lowent-h15b-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T02:16:27+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowent_h15b.zip

**wandb_id**: 0858yqoz

**hypothesis**: Near-miss continuation of h15b (rew and std anneal both still trending at run end). Park-basin failure class (1/6 sto hard tripod park + det duty-skew churn) is exploration-gated and vanishes as std anneals below ~1.5. One variable vs h15b: +4M steps identical config. If-true: 15s det+sto fwd>=0.40 and gait_valid 12/12, std keeps falling. If-false: std <1.5 but park draw persists -> basin structural -> pricing work (ii)/(iii), no third segment. Parent ppo_goal_cw_walk_lowent_h15b.zip md5 d0a12a94. Snapshot 6c579c5.

**gate**: 15s DR0 harness 6eps/mode det AND sto: forward >=0.40m 12/12 AND gait_valid 12/12 AND >=2 swings/leg AND 0 terminations AND no final-third frame degradation. Retention: 5s det slip mean <=0.93

**verdict**: FAIL 15s gate: fwd 10/12 (det[2]=0.285 churn, sto[4]=0.037 hard tripod park), gait_valid 11/12, 0 terminations, retention slip 0.275<=0.93 but distance-confounded (slip/m 1.50 vs parent 1.53 identical). policy_std 1.485 (<1.5 threshold) and park persisted at same 1/6 rate + same seed indices as parent -> HYPOTHESIS REFUTED, park basin STRUCTURAL. NOT HARDWARE-READY (1/6 park freeze risk, skating 1.3-1.6 m/m, DR0 only). Champion UNCHANGED (h15b d0a12a94); c1 recorded as preferred warm-start parent for pricing arms. Evals logs/ckpt_eval/cw_walk_lowent_h15b_c1_{15s,5s}, all 24 eps on camera (--video-every 1), 8 strips reviewed w/ md5s in RL_LOG cycle 21.

