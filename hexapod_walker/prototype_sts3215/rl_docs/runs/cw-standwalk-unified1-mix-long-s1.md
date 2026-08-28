# cw-standwalk-unified1-mix-long-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-28T03:58:14+00:00

**pod**: hexapod-mjx-train-5

**steps**: 16000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1-acq8m

**wandb_id**: poe0uaa1

**hypothesis**: Plain English: seed-1 twin of cw-standwalk-unified1-mix-long-s0 - does 60 s full-length session training (5-7 chained segments, joystick command mix, hold-height commands) beat 30 s training on long-horizon zero-termination behavior, on the catastrophe-prone rescued seed? Continues the s1-acq8m PASS checkpoint; walk BC anchor coef=1.0 stays live. Operator order 20260828T033725Z. Completes the 2x2 grid (seed x episode length) so the episode-length read is not a single-seed fluke. Prediction-if-true: beats cw-standwalk-unified1-mix-s1 on session terminations/completion at matched steps with clean gait. Prediction-if-false: no session gain and/or seed1 walk relapse under the longer, rarer-reset diet. Strongest alternative: rise-start practice dilution hurts the rise panel - compare rise success vs the 30 s twin.

**gate**: At 16M, paired vs cw-standwalk-unified1-mix-s1 at matched steps: PASS if DR-0 det walk gait_valid >=5/6 zero-sac AND session terminations-per-episode / completion fraction beat the 30 s twin outside eval noise; PARTIAL if equal; FAIL if gait_valid regresses (seed1 relapse) or rise success drops materially vs the twin.

**verdict**: PASS vs its pre-registered gate (paired vs the 30s twin cw-standwalk-unified1-mix-s1 at matched 16M steps). Evidence (session_verdict.json, 90-episode dr0+owndr+dr0_long panel, pulled off-pod): 0/90 terminations (twin: 2/90, both over_current on rise/rsi), zero_falls=true (twin: false), session_complete_frac 1.0 (twin 0.978), segments_reached_frac 1.0 (twin 0.986), gait_valid_frac 1.0 sac=[] (twin also 1.0/sac=[] -- not a regression), rise success owndr det 5/6 sto 6/6 (twin 4/6 det, 6/6 sto -- not dropped, mildly better). DR-0 det walk gait_valid 6/6 sac=[] (gate/report.json) clears the >=5/6 zero-sac clause. Why: this is the SAME conclusion long-s0 already reached against its own 30s twin (2 terms/90 vs 6/90) -- the full 2x2 seed x episode-length grid now agrees on both seeds: the 60s/mode_seq_max_segments=7 session recipe reliably beats the 30s one on termination/completion at matched budget, with rise/gait quality flat-or-better, not just one lucky seed. Command tracking itself (dir_err med 62.1deg, slip/m med 14.0) is still far outside the joystick-band gate (dir_err<=40, slip<=2.9) and reward rose monotonically through 16M (quarters -4.7/564.7/1392.5/2315.2, no plateau) with dir_err/slip both improving ~5-9deg / 2-3x since the 8M acq8m parent across every arm -- per the 08-21 ruling this is a continue signal, not a stop. What's next: adopt the 60s recipe as the standard going forward; continue both seeds' 16M checkpoints for more budget toward the tracking gate (queued this cycle).

