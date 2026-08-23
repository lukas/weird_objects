# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-yawenv045

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T18:46:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: yv8xgyec

**hypothesis**: Plain English: the policy is graded on turning at 0.30 rad/s but has never once been COMMANDED faster than 0.30 during training, so the bar sits at the extreme edge of its command distribution where tracking-kernel gradients are weakest; raising only the training yaw command ceiling to 0.45 rad/s (plant-reachability of ~0.29 proven by the CPG turn-authority probe, so most of that range is honest) makes 0.30 an interior command and should force higher turn authority at 0.30. First arm of the curriculum/exposure fork after turnclip1 closed the 7th and last demo-side mechanism (tips 0.2375/0.2457 vs parent 0.2157/0.2351, achieved arc wz still pinned at teacher's 0.15-0.16 despite 0.29-capable demos). Single lever on the unmutated pushcal518 recipe (teacher_v2, seed 7); eval envelope stays 0.30. Prediction-if-true: tips improve >=0.03 with slip in the family band. Prediction-if-false: tips unmoved (+-0.02) -- combined with yawexpo02 this would close the envelope half of the exposure class.

**gate**: PASS: both tip_left_err and tip_right_err improve by >=0.03 vs parent seed7 pooled reads (0.2157/0.2351) with >=1 side <=0.22, AND m5 walk det_slip_med within 3.2-4.0 (read at n_translating; if <6 translating episodes rerun with --walk-per-mode 24 before judging slip), AND 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: one tip clears or tips move but slip regresses >0.3 -- needs a second seed before promotion. FAIL: tips unmoved (+-0.02, wzmask2-grid-calibrated floor) -- closes the envelope-overshoot lever; if yawexpo02 also FAILs the whole exposure class closes and the fork escalates to joint-space capability (delta-clamp probe / phase-clock cadence).

**verdict**: Raising the training yaw-command ceiling did NOT teach the robot to turn faster -- commanding turns up to 0.45 rad/s in training (so the graded 0.30 becomes an interior command, not the distribution edge) left the m5 turn-in-place tip errors unmoved: 0.2238/0.2179 vs parent seed7 0.2157/0.2351 (deltas +0.008/-0.017, both inside the wzmask2-grid-calibrated +-0.02 noise floor; pre-registered FAIL branch). Arc scenarios still track at 0.13-0.18 |wz_err|. Safety fully clean: 0 falls, 0 terms, gait_valid 12/12 on walk/push/fault (best fault read in the family), push PASS, fault PASS; walk det_slip_med 4.56 is a thin-sample read (n_translating=3, noise per q_20260823T0700Z, not adjudicated). WHY: the policy trained 2M steps with 0.45 rad/s commands and still never produces >~0.17 rad/s -- exposure to bigger commands does not unlock the plant-reachable 0.29 rad/s the CPG probe demonstrated. NEXT: envelope-overshoot lever closed; with yawexpo02 also FAIL the whole exposure class closes and the yaw fork escalates to joint-space capability (phase-clock cadence, stance geometry -- delta-clamp already refuted eval-only).

