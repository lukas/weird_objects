# cw-dep-vref1-r1-placement-comshift-zerobias

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T18:21:12+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: nzvw3wh4

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: does a THIRD individually-benign assembly-tolerance axis (per-joint zero-calibration bias, 3deg) stay benign stacked on the just-PASSED 2-axis placement+CoM-shift bundle? All three axes model imperfections fixed at build/assembly time (imprecise joint mounting, off-center weight, and hand-zeroed servo horns) that the real chassis will have simultaneously, not one at a time. Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + all 3 axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the assembly-tolerance stack keeps composing free. If-false: three compounding fixed offsets (angle + CoM + calibration) break tracking in a way no pair did alone -- a real pre-attempt-#2 assembly-QA risk worth flagging.

**gate**: own-cfg (DR0.35 + placement_noise_deg=6 + com_offset_m=0.03 + joint_zero_bias_deg=3) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 retention clean; frames watched det

