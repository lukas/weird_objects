# cw-standwalk-unified1-mix-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T03:49:25+00:00

**pod**: hexapod-mjx-train-2

**steps**: 16000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1-acq8m

**wandb_id**: 6kyxn8t9

**hypothesis**: Plain English: seed-1 twin of cw-standwalk-unified1-mix-s0 - can the rescued seed1 policy (the lineage that needed the walk-anchor rescue after an anchor4-class collapse) also absorb joystick-style command changes and hold-height commands in chained sessions without walk regression? Operator order 20260828T033725Z. Continues the s1-acq8m PASS checkpoint on the mixed-session gate's exact command bundle with the coef=1.0 command-conditioned walk anchor live. Seed1 matters independently: it is the catastrophe-prone seed, so it tests whether the unified diet re-triggers the collapse the rescue fixed. Prediction-if-true: same shape as seed0 (terminations down, direction tracking up, gait clean). Prediction-if-false: seed1 relapses (gait_valid drops / sac legs) under the widened diet while seed0 holds - the rescue is command-distribution-fragile and the anchor dose or staged curriculum is next. Strongest alternative: command-tracking paddle exploit - slip/m + video catch it.

**gate**: At 16M: PASS if DR-0 det walk gait_valid >=5/6 zero-sac AND own-cfg session episodes improve vs the s1-acq8m parent's mixed-session baseline (terminations down, dir-err median <=40 deg trend); PARTIAL if gait holds, tracking flat; FAIL if gait_valid <5/6 or sacrificed legs reappear (rescue relapse under unified diet - escalate anchor dose / staged cmd curriculum, 08-21 ruling applies).

