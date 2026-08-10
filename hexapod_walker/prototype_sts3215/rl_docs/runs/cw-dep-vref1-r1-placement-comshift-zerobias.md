# cw-dep-vref1-r1-placement-comshift-zerobias

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T18:21:12+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: nzvw3wh4

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: does a THIRD individually-benign assembly-tolerance axis (per-joint zero-calibration bias, 3deg) stay benign stacked on the just-PASSED 2-axis placement+CoM-shift bundle? All three axes model imperfections fixed at build/assembly time (imprecise joint mounting, off-center weight, and hand-zeroed servo horns) that the real chassis will have simultaneously, not one at a time. Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + all 3 axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the assembly-tolerance stack keeps composing free. If-false: three compounding fixed offsets (angle + CoM + calibration) break tracking in a way no pair did alone -- a real pre-attempt-#2 assembly-QA risk worth flagging.

**gate**: own-cfg (DR0.35 + placement_noise_deg=6 + com_offset_m=0.03 + joint_zero_bias_deg=3) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 retention clean; frames watched det

**verdict**: PASS (dig-in, resolves the triage flag as a misread). OBSERVATIONS: own-cfg (DR0.35 + placement6deg + com0.03m + zerobias3deg) det gv 6/6 (prog med 0.97, slip/m med 1.26), sto gv 6/6 (prog med 0.91, slip 1.28), 0 term, 0 sacrificed — slip ~11% over vref1-r1's 1.13 det ceiling, inside the ±20% tolerance the parent itself passed at (1.28). DR0 retention: 11/12 clean (det prog med 0.95 slip 1.20, sto 1.01/1.09, 0 term); det/4 is the lineage's KNOWN fixed-draw march-in-place crater — the same seed-0 draw (cmd 0.694m) craters in EVERY sibling at DR0: parent placement-comshift prog -0.03/slip 29.7 (its own PASS verdict names it), zerobias -0.18/27.5, placement 0.05/25.8, comshift 0.02/29.0, comshift-deadband 0.08/27.1, THIS -0.09/25.3. INTERPRETATION: the triage claim 'failure absent from the parent on the same draw' is false — the parent fails the draw identically; only the STALL POSTURE differs: this ckpt idles with leg 3 mostly aloft (duty 0.06 vs parent 0.31/zerobias 0.20), flipping the sacrifice detector on an episode where no family member walks at all. Video (det/4): 15s level march-in-place at 0.014-0.033 m/s vs ref 0.051, tilt <=7deg, no fall, Imax 2.6A — the known low-speed-command creep-economics crater, not a flag-leg walk. All other 23 episodes across both evals are clean six-leg gait; the equivalent draw at own-DR passes (det/4 prog 0.89, gv True). Root cause chain: stall <- low-speed command draw <- contact-pricing creep economics (pre-existing lineage limitation, operator calibration) — NOT the zerobias axis stack. VERDICT: PASS — the THIRD assembly-tolerance axis (3deg per-joint zero-calibration bias) composes free onto the placement+CoM 2-axis bundle; gv=False on the DR0 crater is a stall-posture artifact, not gait erosion. hardware-ready: no (inherits vref1-r1's paddle-gait economics, like every sibling). Watch-item: on the DR0 idx4 stall this ckpt parks on 5 legs (leg 3 aloft) — benign stationary stance, but if any FUTURE dep compose shows a sacrificed leg while actually TRANSPORTING, that is a real fail, not this fingerprint. HYPOTHESIS STATUS: if-true confirmed.

