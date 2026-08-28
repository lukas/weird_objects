# cw-standwalk-unified1-mix-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-28T03:49:25+00:00

**pod**: hexapod-mjx-train-2

**steps**: 16000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1-acq8m

**wandb_id**: 6kyxn8t9

**hypothesis**: Plain English: seed-1 twin of cw-standwalk-unified1-mix-s0 - can the rescued seed1 policy (the lineage that needed the walk-anchor rescue after an anchor4-class collapse) also absorb joystick-style command changes and hold-height commands in chained sessions without walk regression? Operator order 20260828T033725Z. Continues the s1-acq8m PASS checkpoint on the mixed-session gate's exact command bundle with the coef=1.0 command-conditioned walk anchor live. Seed1 matters independently: it is the catastrophe-prone seed, so it tests whether the unified diet re-triggers the collapse the rescue fixed. Prediction-if-true: same shape as seed0 (terminations down, direction tracking up, gait clean). Prediction-if-false: seed1 relapses (gait_valid drops / sac legs) under the widened diet while seed0 holds - the rescue is command-distribution-fragile and the anchor dose or staged curriculum is next. Strongest alternative: command-tracking paddle exploit - slip/m + video catch it.

**gate**: At 16M: PASS if DR-0 det walk gait_valid >=5/6 zero-sac AND own-cfg session episodes improve vs the s1-acq8m parent's mixed-session baseline (terminations down, dir-err median <=40 deg trend); PARTIAL if gait holds, tracking flat; FAIL if gait_valid <5/6 or sacrificed legs reappear (rescue relapse under unified diet - escalate anchor dose / staged cmd curriculum, 08-21 ruling applies).

**verdict**: Full unified command-following session (mixedsession: dr0+owndr+dr0_long, 90 eps, matched vs the acq8m-s1 8M parent) reads PARTIAL, stronger than the s0 twin: isolated DR-0 gate walk/det gait_valid 6/6 zero-sac; session gait_valid_frac 1.0 (parent 0.983), sacrificed_legs_seen=[] (parent had [2] -- the parent's own leg-2 quirk is GONE, not reappeared). Terminations 2/90 (parent 4/90), session_complete_frac 0.978 (parent 0.956), dir_err_med 59.98deg (parent 66.5), slip/m_med 14.51 (parent 23.34), progress_ratio_med 0.146 (parent 0.078, ~2x). Every axis improves over the matched-seed parent, but absolute command tracking is still far from a usable band (dir_err cap ~40deg, slip cap ~2.9) -- harness soft-gate flags slip_ok=false, dir_err_ok=false, matching this run's own pre-registered PARTIAL branch ('gait holds, tracking flat'). Reward still monotonically rising through 16M (quarters 68/726/1522/2047). This is the catastrophe-prone rescued lineage (seed1) and it shows the CLEANEST read of the whole wave (no sac leg at all). Next: hold for the long-s0/long-s1 twins' joint read before deciding continue-budget vs graded-heading-curriculum mechanism.

