# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-yawexpo02

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T18:48:01+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: momlgc60

**hypothesis**: Plain English: half of all training commands have ZERO yaw, so the policy spends only ~50% of rollout time practicing turns at all; cutting walk_yaw_zero_frac from 0.5 to 0.2 gives ~2.7x more nonzero-yaw practice per rollout with everything else identical, testing whether raw turn-practice VOLUME (not demo shape, pricing, or discriminator obs -- all seven of those classes are closed FAIL as of turnclip1) is what caps achieved turn rate at the 0.15-0.16 rad/s teacher saturation. Second arm of the curriculum/exposure fork, paired with yawenv045 (envelope-overshoot); single lever on the unmutated pushcal518 recipe (teacher_v2, seed 7). Prediction-if-true: tips improve >=0.03 with slip in the family band. Prediction-if-false: tips unmoved (+-0.02) -- combined with yawenv045 this closes the exposure class entirely and escalates the yaw fork to joint-space capability (delta-clamp probe / phase-clock cadence).

**gate**: PASS: both tip_left_err and tip_right_err improve by >=0.03 vs parent seed7 pooled reads (0.2157/0.2351) with >=1 side <=0.22, AND m5 walk det_slip_med within 3.2-4.0 (if <6 translating episodes rerun with --walk-per-mode 24 before judging slip), AND 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: one tip clears or tips move but slip regresses >0.3 -- needs a second seed. FAIL: tips unmoved (+-0.02) -- closes the exposure-dose lever; if yawenv045 also FAILs the exposure class is closed and the fork escalates to joint-space capability.

**verdict**: Giving the robot ~2.7x more turning practice made its turn tracking WORSE, not better -- cutting the zero-yaw command fraction 0.5->0.2 (so most rollout time carries a nonzero turn command) landed m5 tip errors at 0.2443/0.2793 vs parent seed7 0.2157/0.2351 (deltas +0.029/+0.044, right side beyond the +0.02 floor in the wrong direction; pre-registered FAIL branch, exposure-dose lever closed). Everything else is clean: 0 falls, 0 terms, gait_valid 12/12 on all sections, walk det_slip_med 3.56 at n_translating=6 (at the family's measured 3.553 noise-floor bar), push PASS (det_slip 3.19), fault PASS. WHY: more yaw practice under the same incentives just reinforces the same saturated ~0.15 rad/s turn style; dose is not the constraint. TOGETHER WITH yawenv045 (also FAIL, tips unmoved): the curriculum/EXPOSURE class is now CLOSED -- neither bigger commands nor more practice moves turn authority, on top of the seven closed demo-side mechanisms. NEXT: yaw fork escalates to joint-space capability -- the delta-clamp probe already refuted the action-rate clamp eval-only, so the named candidates are 1.333 Hz phase-clock cadence and stance geometry.

