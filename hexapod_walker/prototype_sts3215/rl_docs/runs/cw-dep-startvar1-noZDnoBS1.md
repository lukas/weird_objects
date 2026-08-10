# cw-dep-startvar1-noZDnoBS1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T16:08:09+00:00

**pod**: hexapod-mjx-train-4

**steps**: 18000000

**parent**: cw-dep-startvar1-r1

**wandb_id**: pq2txcn5

**hypothesis**: Third isolation arm triangulating the cw-dep-startvar1-r1/-s1 FAIL (systemic gait breakdown, real sacrificed-leg episode, declining reward). Single-axis ablations both came back partial: noZD1 (zero_drift_cmd_frame->0 alone) eliminates the catastrophic sacrificed-leg failure but slip/prog still don't recover to vref1-r1's band; noBS1 (bad_start_prob->0 alone) recovers 4/6 det episodes but 2/6 still catastrophic-skate. Neither single removal is sufficient -- this arm removes BOTH (zero_drift_cmd_frame->0 AND bad_start_prob->0), keeping placement_noise_deg=6 + joint_zero_bias_deg=3 + k_current=0, everything else identical to startvar1-r1/warm-start vref1-r1. If-true: gait fully recovers to vref1-r1's own band (gv 6/6, no flag leg, slip/m 0.9-1.3, reward climbs not declines) -- confirms an INTERACTION between zero-drift-frame and bad-start is the sufficient cause, placement6+zerobias3 alone are benign (consistent with placementnoise6-r3 PASS elsewhere). If-false: still degraded with both removed -- placement_noise_deg=6 or joint_zero_bias_deg=3 (previously assumed benign in isolation) becomes the next suspect, or k_current=0 removes a stabilizing income term; escalate to a full one-axis-at-a-time ladder on this exact warm-start before trusting any start-variation compose for hardware.

**gate**: Own-cfg (its own trained varied-start config) det+sto 6/6 gait_valid, 0 sacrificed-leg episodes, slip/m med within vref1-r1's own band (0.89-1.36), prog_ratio med >=0.85, reward quarters climbing/plateauing not declining through training; frames watched det for flag-leg/skate

