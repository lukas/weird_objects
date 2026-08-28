# cw-standwalk-unified1-mix-long-s0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-28T03:53:35+00:00

**pod**: hexapod-mjx-train-4

**steps**: 16000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-acq8m

**wandb_id**: olibz31b

**hypothesis**: Plain English: does training on FULL-LENGTH 60 s command sessions (5-7 chained rise/hold/walk/lower segments, joystick-style walk commands, hold-height commands) teach the long-horizon zero-termination behavior the DONE gate demands better than the 30 s twin? The gate instrument (eval_mixed_session) scores 60 s and 180 s sessions with a HARD zero-terminations bar; the 30 s training arms only ever practice ~2-3 segment chains, so boundary/termination residuals late in long chains are never trained against. Same recipe and command bundle as cw-standwalk-unified1-mix-s0 (walk BC anchor coef=1.0 live), only episode length 30->60 s and segment cap 5->7 change. Operator order 20260828T033725Z. Prediction-if-true: at matched 16M this arm beats the 30 s twin on session terminations-per-episode and completion fraction. Prediction-if-false: 30 s chains transfer fine to 60 s sessions and the cheaper episodes win (fewer resets also means less rise-from-down start diversity per step, the real cost of this arm). Strongest alternative: longer episodes just dilute rise-start practice and hurt rise robustness - the rise panel catches it.

**gate**: At 16M, paired vs cw-standwalk-unified1-mix-s0 at matched steps: PASS if DR-0 det walk gait_valid >=5/6 zero-sac AND session terminations-per-episode / completion fraction beat the 30 s twin outside eval noise; PARTIAL if equal (30 s transfers - prefer cheaper twin); FAIL if gait_valid regresses or rise-mode success drops materially vs the twin (reset-diversity cost dominates).

**verdict**: Own paired gate (vs the 30s twin cw-standwalk-unified1-mix-s0, matched 16M steps): PASS -- the 60s-episode/mode_seq_max_segments=7 recipe beats the cheaper 30s twin outside eval noise on every session-health axis: terminations 2/90 vs twin's 6/90, session_complete_frac 0.978 vs 0.933, rise-mode terminations 0/90 vs twin's 3/90 (rise success did NOT drop -- clears this gate's own FAIL clause), isolated DR-0 gate walk/det gait_valid 6/6 zero-sac equal to the twin. Command-tracking absolute quality is in the same ballpark as the twin, NOT separately solved by this arm (dir_err_med 64.56deg vs twin 63.57, slip/m_med 9.17 vs 9.0, progress_ratio 0.131 vs 0.133) -- this PASS answers 'is the longer/harder training recipe worth funding going forward', not 'is command-following done'. vs the matched-seed acq8m parent (8M): terminations 2/90 vs 5/90, dir_err 64.6 vs 69.5, slip 9.17 vs 25.98, prog 0.131 vs 0.079 -- same real-but-modest improvement pattern as the twin. Reward still rising through 16M (quarters 57/668/1418/2239). sacrificed_legs_seen=[2] in the full session, identical to what the acq8m parent already shows -- pre-existing, not new. Recommendation: prefer the 60s/segments=7 recipe (this arm) over the 30s twin for any further command-following continuation; the open question is still continue-budget vs graded-heading-curriculum for the dir_err/slip gap, pending the long-s1 twin's own read.

