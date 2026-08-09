# cw-walk-lowent-h15b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T01:38:30+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_step0_lowent.zip

**wandb_id**: y9jav0y4

**hypothesis**: 0-b rung 1: 5s training horizon leaves long-horizon regulation unlearned; training at 15s consolidates sustained walking. One variable vs parent lowent (episode-seconds 5->15). If-true: 15s harness det+sto >=0.40m 12/12, gait_valid 12/12, no final-third decay, 5s retention (det slip <=0.93). If-false: (a) matches parent's eval-only 15s (det 6/6, sto ~5/6) -> horizon rung redundant; (b) gait degrades -> horizon-dependent reward pricing sensitivity, stop before DR. Alt: sto wander ep was binomial noise - distinguished by 12/12 at 15s. Parent ppo_goal_cw_walk_step0_lowent.zip md5 923ee55c. Snapshot e5f0c3e (tag exp/cw-walk-lowent-h15). NAME NOTE: h15 attempt 1 crashed at init (parent ckpt missing on pod, since copied+md5-verified); W&B name h15 burned by the crashed run rsut0epw -> relaunched as h15b.

**gate**: 15s DR0 harness 6eps/mode det AND sto: forward >=0.40m 12/12 AND gait_valid 12/12 AND >=2 swings/leg AND 0 terminations AND no final-third frame degradation; retention: 5s det slip mean <=0.93

**verdict**: FAIL gate (fwd 10/12, gait_valid 11/12: recurring 1/6 sto tripod-park ep, on camera in _15s_allvid; 0 terminations, retention 5s det slip 0.584<=0.93 OK). Hypothesis REFUTED on core claim (if-false branch a: gate clauses match parent within noise). Real side-gains: det slip -20% non-overlap both horizons, det 5s success 0/12->4/6, std anneal resumed 2.08->1.74. NOT HARDWARE-READY (park ep + skating, DR0 only). CHAMPION UPDATE walk line: h15b supersedes lowent. Evals: logs/ckpt_eval/cw_walk_lowent_h15b_{15s,5s,15s_allvid}.

