# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-kernelema-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T08:41:08+00:00

**pod**: hexapod-mjx-train-4

**steps**: 6000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: jypos2i3

**hypothesis**: Plain English: does the kernel-noise-tax EMA fix (both axes) help when applied as a BUDGET CONTINUATION on the tipfrac05 checkpoint's own fixed basin, instead of a fresh 2M from-scratch retrain -- the exact distinction the 3-arm decomposition grid could not make (kernelema1/-yawonly/-velonly2 all regressed tip-tracking to the same 0.21-0.23 band regardless of axis, and a concurrent cycle attributed that to fresh-retrain basin-selection noise rather than a real defect). Single lever vs the already-verdicted -acq1 run (same +6M-from-tipfrac05 continuation, same seed=7, same everything): add reward.walk_kernel_vel_ema=1 + walk_kernel_yaw_ema=1 (tau=0.75s both) on top of it. This answers TWO open questions at once: (1) tracks.json/q_20260823T0240Z item b -- does repricing rescue the acq1 erosion (acq1 alone eroded tips 0.162/0.184 -> 0.204/0.269 under +6M with the EMA off; does the EMA fix hold turn-tracking steady this time on the SAME fixed basin the grid never actually tested)? (2) a live mechanism check: this cycle's code read of walk_task.py found the EMA update is UNCONDITIONAL across command transitions by design, and eval_amp_m5's yaw section always measures tip-left/right immediately after translating arc-max segments in one continuous rollout -- if THAT (not basin noise) is the real driver, tips should regress here too even from a mature, fixed, already-passing basin; if it's pure basin noise, this continuation should hold steady like acq1's own reward trend suggested was otherwise achievable.

**gate**: HOLD/IMPROVED = eval_amp_m5 tips stay <=0.20 both signs (ideally tighter than tipfrac05's own 0.162/0.184) with walk/push/fault sections holding at/above their acq1 bars and own-cfg DR-0 gait_valid >=11/12 -> kernel-EMA repricing is confirmed as a real, budget-safe fix (not basin noise) and becomes the funded path off acq1's erosion; relaunch the bundled kernelema1/velonly2/yawonly arms as CONTINUATIONS (not fresh retrains) if so. STILL ERODED/WORSE (tips >0.20, especially if similar to or worse than acq1's own 0.204/0.269) on this SAME fixed basin = the EMA mechanism itself has a real transition-handling defect (this cycle's REFINED MECHANISM note in amp/STATUS.md), not just noise -- do not spend further budget on naive kernel-EMA arms; build the targeted command-transition-aware EMA reset before retrying. FLAT relative to acq1's own erosion (no better, no worse) = genuinely ambiguous, escalate to hold/forward income repricing via a different lever (the actuation-cost/reward-shape asymmetry this cycle's probe_walk_income re-read measured: kernel sway-tax terms -401/-171 per 15s dominate the hold/forward gap, current/gyro/roll only -43 combined).

