# cw-dep-startvar1-placementonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T17:34:22+00:00

**pod**: hexapod-mjx-train-10

**steps**: 18000000

**parent**: cw-dep-startvar1-noZDnoBS1

**wandb_id**: rksqxrhr

**hardware_ready**: False

**hypothesis**: Fourth isolation arm in the startvar1 triangulation. noZDnoBS1 (zero_drift_cmd_frame->0 AND bad_start_prob->0, keeping placement_noise_deg=6 + joint_zero_bias_deg=3) still showed the SAME 2/6 degraded det draws (idx3 partial, idx4 full march-in-place crater) that its single-axis siblings noZD1 and noBS1 also showed at those exact indices -- ruling OUT zero_drift_cmd_frame and bad_start_prob as the cause of this residual failure. The remaining suspects are the two axes held constant across all three prior arms: placement_noise_deg=6 and joint_zero_bias_deg=3, each individually PASSED clean on the plain vref1-r1 line (ppo_goal_cw_dep_vref1_r1_placement, _zerobias) but NEVER tested together until noZDnoBS1. This arm isolates placement_noise_deg=6 ALONE (joint_zero_bias_deg->0, zero_drift/bad_start stay off) on the exact same warm-start/seed/eval panel. If-true: gait fully recovers to vref1-r1's own band (det+sto 6/6 gv, slip/m 0.89-1.36, prog med >=0.85) -- placement_noise_deg=6 is benign alone, which means the joint_zero_bias_deg=3 x placement_noise_deg=6 INTERACTION (not either axis alone) is the residual cause, and the next arm should test zerobias-alone to confirm the interaction from the other side. If-false: still degraded -- placement_noise_deg=6 alone is NOT as benign in this varied-start/walk-panel context as the standalone _placement PASS suggested (a context-dependent axis, not a fixed property), and the mechanism rework needs to look at how placement noise interacts with the walk task's continuous foot-placement demand, not just at zero-drift/bad-start.

**gate**: Own-cfg (placement6 alone, zerobias/badstart/zerodrift all off) det+sto 6/6 gait_valid, 0 sacrificed-leg episodes, slip/m med within vref1-r1's own band (0.89-1.36), prog_ratio med >=0.85, reward quarters climbing/plateauing not declining; frames watched det for flag-leg/skate; explicit compare vs noZDnoBS1's own det ep3/ep4 values

**verdict**: FAIL vs own gate, informative isolation result: DR0-gate det ok 3/6 (prog med 0.75, slip med 1.62, outside vref1-r1 band 0.89-1.36), sto clean 6/6; own-cfg(DR0.35) det 5/6, sto 3/6. Reproduces parent noZDnoBS1 det ep3/ep4 failure near-identically with joint_zero_bias_deg ALSO zeroed (only placement_noise_deg=6 + walk-panel varied-start left) and adds a new mild ep5 dip -- rules OUT the zerobias x placement interaction (if-true refuted): placement_noise_deg=6 ALONE reproduces the residual failure, so it (not an interaction) is the remaining suspect. gv 6/6 both modes, 0 term/sacrificed-leg; video is the lineage known clean six-leg march-in-place-stall signature, no flag-leg/skate. Closes the 4-arm startvar1 isolation triangulation (zero_drift_cmd_frame, bad_start_prob, joint_zero_bias_deg all individually+jointly ruled out; placement_noise_deg=6 implicated).

