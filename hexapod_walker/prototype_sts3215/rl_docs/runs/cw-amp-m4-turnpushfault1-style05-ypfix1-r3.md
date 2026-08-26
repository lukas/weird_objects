# cw-amp-m4-turnpushfault1-style05-ypfix1-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T03:06:47+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m4-turnpushfault1-style05-r2

**wandb_id**: 0p3se9bz

**hypothesis**: Plain English: the only policy that carries all four M5 axes (walk/turn/push/fault) turns in the RIGHT direction but 1.7x too fast, because the reward still pays over-spinning more than accurate tracking -- this run keeps training it with the already-built pricing fix ON so income peaks at exactly the commanded rate. Re-launch of ypfix1-r2, which LAUNCH_CRASHED at 0 steps on a stale --obs-pad-transplant=18 inherited from its parent's launch template (init-from checkpoint is already 93-dim, no padding needed); single fix here is an explicit trailing --obs-pad-transplant=0 override (argparse last-flag-wins, verified). Same evidence as before: signed income probe wz_mean +0.478/-0.518 on tip_left/right, yaw_ratio 1.65/1.79 through the legacy 1.25x overshoot clip -- correct-sign, miscalibrated 1.7x, the closest-yet starting point for the yaw_prog_overshoot_decay/yaw_prog_avg_s keys to finish the job.

**gate**: Acquisition-repair (6M more, 8M total). PASS = corrected-bus eval_yaw (fast profile 1500/80, dr.fault_prob=0/ext_push_prob=0, obs.fault_health=1) tip-left AND tip-right err <=0.20 with income-probe yaw_ratio in [0.8,1.25], AND safety bar holds (own-cfg DR-0 gate gait_valid >=10/12, topples <=2/6 det + <=2/6 sto, no new sacrificed legs) => first genuine M5 full-suite candidate, run eval_amp_m5 immediately. PARTIAL = ratio drops toward 1.0 but tips stay >0.20 => overshoot farm closed, residual = hold/forward income dominance (yppeak FAIL-branch mechanism), next lever is the dominance repricing key (code+bank work, dig-in). FAIL = ratio unchanged ~1.7 or safety bar erodes => keys ineffective on composed stacks, escalate to reward-structure dig-in before any further turn+X budget.

**verdict**: Ledger housekeeping only, no new science: this run finished and was fully analyzed 08-23 ~03:4x (amp STATUS.md) but its ledger status field was never flipped from RUNNING, leaving it perpetually in the untriaged queue. Result stands as recorded then: the banked yaw-overshoot-decay pricing keys (6M continuation, 8M total, reward rose 99->194) did NOT move tip behavior at all -- deterministic tip rollouts BIT-IDENTICAL to parent r2 (yaw_ratio 1.649/1.786, income probe exact float match) despite weights moving (max|dW|=0.87); corrected-bus eval_yaw tips 0.39/0.47 (bar 0.20) and safety eroded to 10/12 gait_valid with two NEW sacrificed legs. Root cause found same-cycle-cluster: tip-scenario joint speed commands pin exactly at the 5deg/tick safety clamp, a quantization attractor reward pricing cannot sculpt. Matches its own pre-registered FAIL branch. No further action: this specific turn+push+fault composition lineage was superseded by the amp track's M5 GATE GREEN (2026-08-23/24, phasehz11 lineage) which is the current champion; ypfix1-r3's turn-composition question is closed history, not blocking work.

