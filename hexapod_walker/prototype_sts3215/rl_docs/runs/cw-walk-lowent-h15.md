# cw-walk-lowent-h15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T01:38:09+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_step0_lowent.zip

**hypothesis**: 0-b rung 1: 5s training horizon leaves long-horizon regulation unlearned; training at 15s consolidates sustained walking. One variable vs parent lowent (episode-seconds 5->15). If-true: 15s harness det+sto >=0.40m 12/12, gait_valid 12/12, no final-third decay, 5s retention (det slip <=0.93). If-false: (a) matches parent's eval-only 15s (det 6/6, sto ~5/6) -> horizon rung redundant; (b) gait degrades -> horizon-dependent reward pricing sensitivity, stop before DR. Alt: sto wander ep was binomial noise - distinguished by 12/12 at 15s. Parent ppo_goal_cw_walk_step0_lowent.zip md5 923ee55c. Snapshot e5f0c3e. RETRY 1: first attempt failed on missing parent ckpt on pod (now copied, md5 verified).

**gate**: 15s DR0 harness 6eps/mode det AND sto: forward >=0.40m 12/12 AND gait_valid 12/12 AND >=2 swings/leg AND 0 terminations AND no final-third frame degradation; retention: 5s det slip mean <=0.93

**verdict**: PHANTOM FINISH (cycle 24): W&B rsut0epw state=failed, 0 steps — attempt 1 crashed at init (parent ckpt absent on walk pod), relaunch REFUSED for duplicate name, lineage continued as cw-walk-lowent-h15b. No checkpoint, no eval possible; terminal status set so ledger_verdicted() dedupes this name permanently.

**refused_reason**: W&B already has a run named cw-walk-lowent-h15 (names are append-only; pick a new one)

