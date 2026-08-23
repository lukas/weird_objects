# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-yawprice2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T11:37:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: jcavr5tk

**hypothesis**: Plain English: the robot turns in place at only ~25% of the commanded rate because the reward pays too little for actually rotating; doubling that payment should make it turn closer to the commanded speed without giving up the push-safety fix. Single lever vs pushcal518 (the new 5-18N safety base, 0/12 falls 3/3 seeds): reward.k_yaw_prog 1.0->2.0. Dig-in finding this tests: the whole tipfrac05 family (11 old-range + 3 recal checkpoints) converges to tip-err ~0.21-0.25 vs the 0.20 M5 v1 bar at ANY push range/budget — reward-optimum misalignment, not a recalibration cost. Over-spin farming is priced out (yaw_prog_overshoot_decay=1.0 + yaw_prog_avg_s=1.0 already on, bank-validated), so raising the income dose is now safe where the pre-decay yawprice3x was not.

**gate**: Own-cfg DR-0 gate + eval_amp_m5 yaw+walk sections. PASS = 0/12 raw-terminated falls AND both m5 tip errs <=0.20 AND walk det slip med <=3.8 (no slip regression). PARTIAL = tips improve >=0.02 vs pushcal518's 0.2157/0.2351 but miss 0.20 (dose-response exists -> read yawprice3 before re-dosing). FAIL = tips unmoved (+-0.02) or any real fall / walk slip >3.8 — income pricing cannot buy turn-in-place authority; escalate to mechanism (stance geometry / wz curriculum), not dose.

**verdict**: Result: doubling the turn-in-place reward income (k_yaw_prog 1.0->2.0) did NOT recover yaw-tip tracking margin against the M5 bar. Evidence: eval_amp_m5 yaw section tip_left_err 0.2164 (parent pushcal518 0.2157, unmoved) and tip_right_err 0.2596 (parent 0.2351, +0.024 WORSE, outside the +-0.02 noise band) -- one side regressed, neither cleared the 0.20 bar. Walk section also still misses its bar (det slip med 3.62 vs <=3.5) but did not regress past 3.8. Safety intact: own-cfg DR-0 gate 0/12 raw falls (walk+sto), gait_valid 12/12, video-clean six-leg gait, roll_peak max 17.1deg. Why: this is the pre-registered dose-grid gate's own FAIL condition (tips unmoved/worse) -- turn-in-place tip accuracy is not purchasable by raising k_yaw_prog alone at this dose. What's next: read jointly with yawprice3 (also finished) before deciding dose-vs-mechanism; a joint FAIL of both doses refutes income pricing as the tip-error lever per the pre-registered gate and escalates to mechanism (stance geometry / wz curriculum), not further dosing.

