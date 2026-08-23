# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-yawprice3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T11:41:27+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: wd74pmef

**hypothesis**: Plain English: second dose point of the turn-payment raise — if doubling (yawprice2) moves turn-in-place accuracy but not far enough, tripling maps the dose-response; if even 3x does not move it, the limit is not the price. Single lever vs pushcal518: reward.k_yaw_prog 1.0->3.0. Note the historical yawprice3x failure predates the overshoot-decay/EMA fix (over-spin farm, since priced out and bank-proven); this re-tests the 3x dose under corrected semantics.

**gate**: Own-cfg DR-0 gate + eval_amp_m5 yaw+walk sections. PASS = 0/12 raw falls AND both tip errs <=0.20 AND walk det slip med <=3.8. PARTIAL = tips improve >=0.02 vs 0.2157/0.2351 but miss 0.20. FAIL = tips unmoved or falls/slip regression; joint FAIL of yawprice2+3 = income pricing refuted as the tip lever, escalate to mechanism not dose.

