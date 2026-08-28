# cw-standwalk-unified1-mix-s0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T03:45:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 16000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-acq8m

**wandb_id**: 8fmm3u26

**hypothesis**: Plain English: can the proven rise/walk/lower policy learn to follow joystick-style command changes (forward/side/reverse/sweep/stop walk commands resampled every ~4 s, plus hold-height up/down commands) inside chained rise->walk->lower sessions, WITHOUT losing the walking it just acquired? Operator order 20260828T033725Z: train the unified command-following model now from the PASSed acq8m checkpoints. This arm continues seed0's acq8m checkpoint on the EXACT command distribution the mixed-session DONE-gate instrument measures (stress_mix + hold_height_cmd + 8-12 s segments, matching eval_mixed_session's canonical bundle), keeping the command-conditioned walk BC anchor at coef=1.0 live so walking regression hurts every update. Prediction-if-true: session-episode terminations fall and command direction error drops while DR-0 det walk gait_valid stays clean (the anchor holds the gait). Prediction-if-false: the widened command distribution collapses walk quality despite the anchor (gait_valid regresses / sacrificed legs return) - then a staged goal.walk_cmd_stage curriculum or higher anchor dose is the next lever, not abandonment. Strongest alternative: reward rises via slip-heavy paddling that tracks commands numerically - slip/m + video catch it.

**gate**: At 16M: PASS if DR-0 det walk gait_valid >=5/6 with zero sacrificed legs (walk retention held) AND own-cfg session episodes improve over the acq8m parent's mixed-session baseline (terminations per episode down, direction-err median <=40 deg trend); PARTIAL if gait holds but command tracking is flat (budget/curriculum question); FAIL if gait_valid <5/6 or sac legs reappear on the walk panel (retention broke - escalate to staged cmd curriculum or anchor dose, per 08-21 ruling not a lineage kill).

