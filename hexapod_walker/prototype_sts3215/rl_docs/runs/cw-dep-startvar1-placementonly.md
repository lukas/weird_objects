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

**verdict**: FAIL vs own gate; CORRECTED root-cause (initial framing overclaimed placement_noise_deg as sole cause -- checked against the standalone seed11/20M cw-dep-vref1-r1-placement PASS, which is cleaner evidence). DR0-gate det ok 3/6 (prog med 0.75, slip med 1.62, band 0.89-1.36), sto clean 6/6. The det/3+det/4 degraded pair reproduces IDENTICALLY across all 4 seed=12/18M isolation siblings (noZD1, noBS1, noZDnoBS1, placementonly) regardless of whether zero_drift/zerobias are on or off, while the seed=11/20M standalone _placement compose (SAME placement_noise_deg=6, no other stressors) shows det/3 clean and only the well-known lineage det/4 crater -- so det/3 is better explained by seed=12 training variance than by placement_noise_deg=6 itself, which cannot be cleanly implicated without a seed-12+placement=0 control (not run). The one clear surviving signal: noZD1 (bad_start_prob=0.4 still ON) is markedly worse than the 3 bad_start=0 siblings (det 0/6 ok + sto also degraded) -- bad_start_prob=0.4 remains the strongest identified stressor, consistent with the earlier noBS1 finding (a contributor, not sole cause). gv 6/6 all arms, 0 term/sacrificed-leg every episode, video clean six-leg creep (no flag-leg/skate) matching the lineage known march-in-place-stall signature. Net: 4-arm triangulation rules OUT zero_drift_cmd_frame and joint_zero_bias_deg as causes; bad_start_prob=0.4 is implicated; placement_noise_deg=6 is UNRESOLVED (confounded with training seed, needs a seed-12+placement=0 control before blaming it). hardware_ready=false; cw-dep-vref1-r1 (no start-variation) remains the hardware-attempt-#2 base.

