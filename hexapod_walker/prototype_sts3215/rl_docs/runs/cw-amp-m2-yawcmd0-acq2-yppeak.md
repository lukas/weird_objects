# cw-amp-m2-yawcmd0-acq2-yppeak

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T01:37:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m2-turnclone-yawcmd0-acq1-r2

**wandb_id**: c6v97xte

**hypothesis**: Plain English: we found why more training made turning worse -- the reward pays a robot that spins PAST the commanded yaw rate more than one that tracks it, so RL erodes accurate turning into sloppy over-spinning while total reward rises. Evidence: 08-23 income audit (probe_walk_income yawcmd0 stack, logs/probe_walk_income/yawcmd0_turn_income_audit.json): eroded acq1-r2 over-rotates (yaw ratio 1.78 vs champion 1.09) yet collects MORE reward_yaw_prog (169 vs 148) via the legacy clip's 1.25x overshoot headroom; deeper, instantaneous-wz pricing fines the honest gait's zero-mean stride oscillation (0.95-ratio scripted tracker earns NEGATIVE yaw_prog) while paying a smooth 2x spinner -- the same DC-vs-AC defect fixed for k_yaw_still on 08-11. Also explains yawprice3x (3x income amplified the farm). This arm = EXACT acq1-r2 respec (6M continuation off yawcmd0-r2, seed/cfg identical) + ONLY the two audited fixes ON: reward.yaw_prog_overshoot_decay=1.0 (income peaks at ratio 1.0, decays past, never negative on overshoot) + reward.yaw_prog_avg_s=1.0 (ratio priced on wz EMA). Both keys new/default-off; TURN semantics bank extended (legacy defect-proof + fix + bit-exact cases) 8/8 GREEN. Prediction-if-true: turn accuracy no longer erodes -- eval_yaw tip errs <=0.16 at 6M (vs acq1-r2's 0.35-0.40 decay), translation held. Prediction-if-false: erosion recurs at matched budget -> the yaw_prog farm was not the driver; residual mechanism = hold/forward income dominance (audit: parker's +28.6/ep sits in hold +160 / forward +118, hold-freeze 1473 > honest tip ceiling 1209) -- that gets priced next. Strongest alternative: capacity interference (net trades turn skill for hold/forward polish regardless of pricing).

**gate**: Acquisition (6M continuation, DR-0), judged vs BOTH yawcmd0-r2 (parent) and acq1-r2 (matched-budget FAIL baseline). Manual eval_yaw own-cfg: PASS = tip-left AND tip-right err <=0.16 (no erosion at the budget that previously eroded to 0.35-0.40) AND translation gait_valid 6/6 det+sto with slip/prog within 20% of yawcmd0-r2; PASS-strong = tip errs <=0.10 (eval_yaw strict bar). INFORMATIVE-ceiling = errs in 0.16-0.20 band with yaw terms still improving at cutoff (08-21 ruling: continue). FAIL = tip err >0.20 or translation erodes -> farm was not the erosion driver; next lever prices hold/forward income dominance. Class-note: a FAIL here does NOT touch yawcmd0-r2's champion status.

