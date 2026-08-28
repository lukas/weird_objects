# cw-standwalk-unified1-mix-long-s0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T03:53:35+00:00

**pod**: hexapod-mjx-train-4

**steps**: 16000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-acq8m

**wandb_id**: olibz31b

**hypothesis**: Plain English: does training on FULL-LENGTH 60 s command sessions (5-7 chained rise/hold/walk/lower segments, joystick-style walk commands, hold-height commands) teach the long-horizon zero-termination behavior the DONE gate demands better than the 30 s twin? The gate instrument (eval_mixed_session) scores 60 s and 180 s sessions with a HARD zero-terminations bar; the 30 s training arms only ever practice ~2-3 segment chains, so boundary/termination residuals late in long chains are never trained against. Same recipe and command bundle as cw-standwalk-unified1-mix-s0 (walk BC anchor coef=1.0 live), only episode length 30->60 s and segment cap 5->7 change. Operator order 20260828T033725Z. Prediction-if-true: at matched 16M this arm beats the 30 s twin on session terminations-per-episode and completion fraction. Prediction-if-false: 30 s chains transfer fine to 60 s sessions and the cheaper episodes win (fewer resets also means less rise-from-down start diversity per step, the real cost of this arm). Strongest alternative: longer episodes just dilute rise-start practice and hurt rise robustness - the rise panel catches it.

**gate**: At 16M, paired vs cw-standwalk-unified1-mix-s0 at matched steps: PASS if DR-0 det walk gait_valid >=5/6 zero-sac AND session terminations-per-episode / completion fraction beat the 30 s twin outside eval noise; PARTIAL if equal (30 s transfers - prefer cheaper twin); FAIL if gait_valid regresses or rise-mode success drops materially vs the twin (reset-diversity cost dominates).

