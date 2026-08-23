# cw-amp-m4-turnpushfault1-style05-ypfix1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: LAUNCH_CRASHED

**created**: 2026-08-23T02:59:26+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m4-turnpushfault1-style05-r2

**hypothesis**: Plain English: the only policy that carries all four M5 axes (walk/turn/push/fault) turns in the RIGHT direction but 1.7x too fast, because the reward still pays over-spinning more than accurate tracking -- this run keeps training it with the already-built pricing fix ON so income peaks at exactly the commanded rate. Evidence the fix is aimed right: signed income probe on this exact checkpoint reads wz_mean +0.478 on tip_left cmd +0.3 and -0.518 on tip_right cmd -0.3, yaw_ratio 1.65/1.79 with reward_yaw_prog 291/294 collected through the legacy 1.25x overshoot clip -- the SAME audited farm as the substrate's 1.78 ratio, and it REFUTES the wrong-sign-rotation hypothesis in the 02:5x STATUS banner (correct sign both directions). yppeak precedent: on the M2-eroded checkpoint these keys halved erosion (0.40->0.27) but hold/forward dominance kept it over bar; HERE the skill is present and merely miscalibrated 1.7x, the closest-yet starting point for the keys to finish the job. 6M continuation, byte-identical cfg otherwise, keys reward.yaw_prog_overshoot_decay=1.0 + reward.yaw_prog_avg_s=1.0 (banked, TURN bank 8/8 green). Matched no-keys control cont1 launched same cycle to attribute any delta to pricing, not budget.

**gate**: Acquisition-repair (6M more, 8M total). PASS = corrected-bus eval_yaw (fast profile 1500/80, dr.fault_prob=0/ext_push_prob=0, obs.fault_health=1) tip-left AND tip-right err <=0.20 with income-probe yaw_ratio in [0.8,1.25], AND safety bar holds (own-cfg DR-0 gate gait_valid >=10/12, topples <=2/6 det + <=2/6 sto, no new sacrificed legs) => first genuine M5 full-suite candidate, run eval_amp_m5 immediately. PARTIAL = ratio drops toward 1.0 but tips stay >0.20 => overshoot farm closed, residual = hold/forward income dominance (yppeak FAIL-branch mechanism), next lever is the dominance repricing key (code+bank work, dig-in). FAIL = ratio unchanged ~1.7 or safety bar erodes => keys ineffective on composed stacks, escalate to reward-structure dig-in before any further turn+X budget.

**verdict**: Died at 0 steps on the documented respec-hygiene bug class: --obs-pad-transplant=18 inherited from the parent's launch template while --init-from-source pointed at the parent's OWN 93-dim output checkpoint (transplant already consumed). Same class as pushfault1-noamp r1 and turnfault1-acq1 r1-r4. Fixed relaunch ypfix1-r3 (explicit trailing --obs-pad-transplant=0) VERIFIED RUNNING on train-1 with identical keys/budget/hypothesis. The plain ypfix1 name is a REFUSED evidence-stub (zero steps), superseded. Nothing trained; no behavioral information.

**failed_reason**: run never appeared as 'running' in W&B within 240s

