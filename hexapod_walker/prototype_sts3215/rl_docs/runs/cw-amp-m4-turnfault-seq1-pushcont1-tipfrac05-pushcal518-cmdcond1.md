# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-cmdcond1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T16:01:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: zzt34h4y

**hypothesis**: Plain English: the AMP discriminator has been blind to WHAT ROTATION RATE WAS COMMANDED the whole time -- it only ever sees the robot's raw body spin, so any policy that turns faster than the teacher's own demo clips (which embody ~0.13-0.18 rad/s no matter what label they carry) reads as 'moving unlike the teacher' and gets docked, capping turn-in-place tracking. Four DIFFERENT mechanism classes already failed to fix this on the pushcal518 lineage: pricing the turn income harder (k_yaw_prog 1x/2x/3x), widening the demo ceiling itself (teacher_v3, +30% measured wz), removing the style term entirely (noamp1, which also broke slip), and reset-densifying into more turn states (tipspawn1b/tipspawn3). This arm is the first STRUCTURAL fix: append the LIVE commanded (vx_ref, vy_ref, wz_ref) to the discriminator's 60-dim obs_style (-> 63-dim, goal.amp_style_cmd_cond=1, new code this cycle, tests green) and rebuild the motion library the SAME way (teacher_v2_cmdcond.npz -- bit-identical to teacher_v2.npz on all 45 clips' first 60 dims and slip/m, verified; only the command tail is new) so the discriminator can finally judge 'does this look like the teacher's motion GIVEN what was asked for' instead of penalizing high wz outright. Single coupled lever vs pushcal518 (flag + matching-dim library must change together or it's a shape-mismatch crash by design, not two independent variables). Prediction-if-true: yaw tip_left/right_err move meaningfully toward or under the 0.20 bar (>=0.03 improvement on the worse side, mirroring the noamp1/pricing-grid noise band already measured at seed-to-seed std ~0.02) without a new fall/fault regression. Prediction-if-false: tips stay within the family's own measured seed-noise band (tipspawn1b-recipe reads at fixed seeds 11/13/17 span tip_left 0.19-0.24, tip_right 0.21-0.26, i.e. +-0.03) -- command-conditioning doesn't help either, closing structural obs-space fixes and leaving only the 0.20 bar amendment (q_20260823T0130Z) as an option. Strongest alternative: the discriminator's LS-GAN dynamics don't actually use the new tail dims meaningfully (co-training absorbs them as noise) -- watch amp/d_real vs amp/d_fake and amp/style_reward_mean stay healthy (not saturated, not collapsed) as a mechanism-health check independent of the behavior bars.

**gate**: eval_amp_m5 full suite (walk/yaw/push/fault) + weight-movement precheck vs turnfault_seq1 (non-log_std tensors must differ, this run really trained) + discriminator health (amp/d_real vs amp/d_fake stay separated, not saturated/collapsed). PASS = 0/12 raw falls AND walk gait_valid 12/12 AND tip_left_err<=0.20 AND tip_right_err<=0.20 AND walk det_slip_med within 0.5 of pushcal518's own 3.67 (no new slip regression) AND fault gait_valid>=10. PARTIAL = at least one tip clears 0.20 or both move >=0.03 toward it with 0 new falls, but the other or slip does not fully clear -- mechanism real, worth an acquisition-budget continuation or composing with the CPG-demo-swap arm (cpgdemo1, concurrent). FAIL = both tips land within the family's own +-0.03 seed-noise band of pushcal518's 0.2157/0.2351 (i.e. no clear improvement beyond noise) -- command-conditioning refuted, close structural-fix search on the discriminator obs side. Falls/fault regression override any tip improvement to FAIL regardless of tip numbers (safety first).

