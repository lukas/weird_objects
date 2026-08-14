# cw-arch-modeseq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-14T12:48:51+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-arch-gru-dual2

**hardware_ready**: False

**hypothesis**: Teach one model to stand up, walk, sit down and stand up again on command, in a single continuous run - this arm tests whether training directly on chained mode sequences (the new goal.mode_seq episode generator, 75% sequence / 25% single-mode diet) gets the RL policy through the mode switches that specialists only survive when an external script re-anchors them. TRANSITIONS_DIRECTIVE Arm 2: dual1's proven dual-core GRU stack, warm from the dagger1 BC init per the pre-registered warm-start order (transdagger2 gate FAIL on the two rise clauses, gru-dual2 gate FAIL), mode sequencing as the ONLY new variable.

**gate**: Pre-registered (TRANSITIONS_DIRECTIVE Arm 2): sequence eval det+sto DR0 + own-DR0.5 zero falls >=11/12 det AND per-segment criteria >=9/12 AND single-mode retention at dual1 levels (walk gait_valid >=5/6 prog >=0.80, hold >=4/6, lower >=4/6, rise n=12 method >= its own init 3/12) AND switch-window max tilt reported (baseline, no bar in v1).

**verdict**: INFRA FAIL (not science): died at the first mode-switch ~1min in — the goal.mode_seq canonical-frame mint existed only in the in-process MjxVecEnv; the sharded path (--host-workers 24, the actual training path) never minted frames and hit the invariant RuntimeError (train log, mjx_sharded_vec_env._worker_main -> _seq_maybe_switch). Sharded mint twin written same cycle (parent choreography + worker seq_capture protocol + bitwise-vs-inprocess mode_seq test); relaunch as cw-arch-modeseq1-r1 once the pod MJX test result (train-1 /tmp/td2_shardtest.log) reads green.

**failed_reason**: run never appeared as 'running' in W&B within 240s

