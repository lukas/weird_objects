# cw-standwalk-unified1-mix-s0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-28T03:45:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 16000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-acq8m

**wandb_id**: 8fmm3u26

**hypothesis**: Plain English: can the proven rise/walk/lower policy learn to follow joystick-style command changes (forward/side/reverse/sweep/stop walk commands resampled every ~4 s, plus hold-height up/down commands) inside chained rise->walk->lower sessions, WITHOUT losing the walking it just acquired? Operator order 20260828T033725Z: train the unified command-following model now from the PASSed acq8m checkpoints. This arm continues seed0's acq8m checkpoint on the EXACT command distribution the mixed-session DONE-gate instrument measures (stress_mix + hold_height_cmd + 8-12 s segments, matching eval_mixed_session's canonical bundle), keeping the command-conditioned walk BC anchor at coef=1.0 live so walking regression hurts every update. Prediction-if-true: session-episode terminations fall and command direction error drops while DR-0 det walk gait_valid stays clean (the anchor holds the gait). Prediction-if-false: the widened command distribution collapses walk quality despite the anchor (gait_valid regresses / sacrificed legs return) - then a staged goal.walk_cmd_stage curriculum or higher anchor dose is the next lever, not abandonment. Strongest alternative: reward rises via slip-heavy paddling that tracks commands numerically - slip/m + video catch it.

**gate**: At 16M: PASS if DR-0 det walk gait_valid >=5/6 with zero sacrificed legs (walk retention held) AND own-cfg session episodes improve over the acq8m parent's mixed-session baseline (terminations per episode down, direction-err median <=40 deg trend); PARTIAL if gait holds but command tracking is flat (budget/curriculum question); FAIL if gait_valid <5/6 or sac legs reappear on the walk panel (retention broke - escalate to staged cmd curriculum or anchor dose, per 08-21 ruling not a lineage kill).

**verdict**: Full unified command-following session (mixedsession: dr0+owndr+dr0_long, 90 eps, matched vs the acq8m 8M parent) reads PARTIAL: walk retention holds (isolated DR-0 gate walk/det gait_valid 6/6 zero-sac; session gait_valid_frac 0.967, session_complete_frac 0.933) and every walk-quality axis improves over the matched-seed acq8m parent -- dir_err_med 63.6deg (parent 69.5), slip/m_med 8.99 (parent 25.98, ~3x better), progress_ratio_med 0.133 (parent 0.079) -- but terminations are flat within noise (6/90 vs parent 5/90, all over_current) and absolute command tracking is still far from usable (dir_err cap ~40deg, slip cap ~2.9). Video/frame-strip matches the numbers: upright six-leg walking, no drag, just slow/imprecise heading-following -- no gate/video disagreement. Harness's own soft-gate already flags slip_ok=false, dir_err_ok=false, matching this run's pre-registered PARTIAL branch ('gait holds, tracking flat -- budget/curriculum question'). sacrificed_legs_seen=[2] shows up in the full session (not the isolated gate) but is IDENTICAL to the acq8m PARENT's own reading -- a pre-existing recipe quirk, not a new regression. Reward still monotonically rising through 16M (quarters 58/743/1376/2031 -- 08-21 continue signal, not plateaued). Next: hold for the long-s0/long-s1 twins' joint read (same wave) before deciding continue-budget vs graded-heading-curriculum mechanism.

