# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T19:17:22+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: bs1myv8w

**hypothesis**: Plain English: the robot may turn slowly because it paces its gait to a fast 1.33 Hz metronome with small ~9-degree steps; the CPG controller that actually reaches the graded 0.29 rad/s turn on this exact plant uses a 2.7x SLOWER cadence (0.5 Hz, measured by FFT on cpg_v1.npz joint tracks) with ~40-50% bigger strides (13-15 vs 9 deg median p2p) -- more yaw per stride, fewer strides. This arm slows the observed phase clock (goal.walk_phase_hz 1.333->0.5, single lever, everything else the unmutated pushcal518 recipe) so the plant-proven slow-big-stride gait becomes the paced default the policy can settle into. This is the first JOINT-SPACE CAPABILITY arm after the exposure class closed (yawenv045 tips unmoved, yawexpo02 tips worse) on top of seven closed demo-side mechanisms; the delta-clamp eval probe already showed the action-rate limit is not binding. Prediction-if-true: m5 tip errs improve >=0.03 (achieved tip wz finally rises above the teacher's 0.15-0.16 saturation), walk stays clean. Prediction-if-false: tips unmoved +-0.02 -- the policy ignores the slower clock or the 1.33 Hz teacher_v2 style anchor holds it at small strides (the sibling -phasehz05-cpglib arm removes that anchor conflict). Strongest alternative: the warm-started 1.33 Hz habit plus style reward simply wins and the clock obs is behaviorally inert, in which case BOTH arms land unmoved and the phase-clock lever closes.

**gate**: eval_amp_m5 suite. PASS: both tip_left_err and tip_right_err improve >=0.03 vs parent seed7 pooled reads (0.2157/0.2351) with >=1 side <=0.22, AND m5 walk det_slip_med within 3.2-4.0 read at n_translating>=6 (if <6 translating episodes rerun with --walk-per-mode 24 before judging slip), AND 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: one tip clears or tips move >=0.03 but slip regresses >0.3 or fault gait_valid drops below 10 -- mechanism real, needs dose/repair. FAIL: tips unmoved (+-0.02, wzmask2-grid-calibrated floor) -- clock obs alone does not change the gait; if -phasehz05-cpglib also FAILs the phase-clock cadence lever closes and the joint-space fork narrows to stance-geometry code work / the q_20260823T0700Z-family bar amendment.

