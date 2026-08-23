# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T19:17:22+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: bs1myv8w

**hypothesis**: Plain English: the robot may turn slowly because it paces its gait to a fast 1.33 Hz metronome with small ~9-degree steps; the CPG controller that actually reaches the graded 0.29 rad/s turn on this exact plant uses a 2.7x SLOWER cadence (0.5 Hz, measured by FFT on cpg_v1.npz joint tracks) with ~40-50% bigger strides (13-15 vs 9 deg median p2p) -- more yaw per stride, fewer strides. This arm slows the observed phase clock (goal.walk_phase_hz 1.333->0.5, single lever, everything else the unmutated pushcal518 recipe) so the plant-proven slow-big-stride gait becomes the paced default the policy can settle into. This is the first JOINT-SPACE CAPABILITY arm after the exposure class closed (yawenv045 tips unmoved, yawexpo02 tips worse) on top of seven closed demo-side mechanisms; the delta-clamp eval probe already showed the action-rate limit is not binding. Prediction-if-true: m5 tip errs improve >=0.03 (achieved tip wz finally rises above the teacher's 0.15-0.16 saturation), walk stays clean. Prediction-if-false: tips unmoved +-0.02 -- the policy ignores the slower clock or the 1.33 Hz teacher_v2 style anchor holds it at small strides (the sibling -phasehz05-cpglib arm removes that anchor conflict). Strongest alternative: the warm-started 1.33 Hz habit plus style reward simply wins and the clock obs is behaviorally inert, in which case BOTH arms land unmoved and the phase-clock lever closes.

**gate**: eval_amp_m5 suite. PASS: both tip_left_err and tip_right_err improve >=0.03 vs parent seed7 pooled reads (0.2157/0.2351) with >=1 side <=0.22, AND m5 walk det_slip_med within 3.2-4.0 read at n_translating>=6 (if <6 translating episodes rerun with --walk-per-mode 24 before judging slip), AND 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. PARTIAL: one tip clears or tips move >=0.03 but slip regresses >0.3 or fault gait_valid drops below 10 -- mechanism real, needs dose/repair. FAIL: tips unmoved (+-0.02, wzmask2-grid-calibrated floor) -- clock obs alone does not change the gait; if -phasehz05-cpglib also FAILs the phase-clock cadence lever closes and the joint-space fork narrows to stance-geometry code work / the q_20260823T0700Z-family bar amendment.

**verdict**: Slowing the gait metronome from 1.33 Hz to 0.5 Hz is the FIRST lever in eight mechanism classes that actually teaches the robot to turn in place at the graded rate -- and it fixes slip too, but it halves forward walking speed. Evidence (m5 suite + walk-per-mode-24 re-read, n_translating=28): yaw tips 0.1488/0.1557 vs parent 0.2157/0.2351 (improve 0.067/0.079, BOTH sides clear the strict 0.20 M5 bar for the first time on this lineage; yaw section PASS); walk det_slip_med 2.443 -- best ever on this lineage (family band 3.47-3.83, bar 3.5), gait_valid 48/48, 0 terms; fault 11/12 gv (leg 5 sacrificed, family range), push 12/12 gv with 1 det term (bar <=2). BUT walk det_prog_med 0.52 vs family ~0.94 and m5 bar 0.75 -- achieved speed ~0.04 m/s vs 0.08 commanded, a cost the pre-registered gate did not price, so PARTIAL not PASS (gate note: slip landed BELOW the literal 3.2-4.0 band -- on the good side; judged as improvement per the band's anti-regression intent). Why: cadence was the binding constraint on yaw-per-stride, exactly as the CPG 0.5Hz/13-15deg measurement predicted; but 2M steps at 0.5 Hz has not yet grown strides enough to hold 0.08 m/s translation. Next: cadence dose sweep 0.7/0.9/1.1 (find the point where tips stay <=0.20 and prog recovers >=0.75) + continue this arm +4M (reward still rising 43.5->216.5 by quarters; 08-21 ruling) to test if stride growth recovers prog at 0.5 Hz. Video: walk det strips clean six-leg cycling, upright, no flag leg.

