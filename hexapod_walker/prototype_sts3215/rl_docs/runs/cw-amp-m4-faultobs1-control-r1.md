# cw-amp-m4-faultobs1-control-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T22:15:54+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m4-faultsmoke1-control

**wandb_id**: 459eenli

**hypothesis**: Plain English: sanity control for cw-amp-m4-faultobs1-noamp -- does merely WIDENING the obs with the (all-ones-when-healthy) fault_health() vector cost anything on a healthy, unfaulted policy, or does the zero-padded transplant start and stay behaviorally identical to the faultsmoke1-control baseline the way the transplant math promises? Single lever vs faultsmoke1-control: obs.fault_health 0->1 + --obs-pad-transplant 18, dr.fault_prob stays 0.0 (no fault ever), same 2M budget, same everything else, same pre-fault ppo_goal_cw_amp_m2_bcinit_sec5_noamp parent checkpoint. (-r1 suffix: first attempt's run name collided with a tag pushed by a stale earlier REFUSED retry of this same launch.)

**gate**: Discovery continuation (2M, DR-0). PASS/no-cost = det+sto gait_valid stays 6/6, prog_ratio/slip_per_m/height_err within 08-22 6-ep noise of faultsmoke1-control's own numbers (prog 1.16/0.88 det/sto, slip 2.36/4.08, height_err 18-31mm band) -- confirms the obs-pad transplant is a true no-op lever, isolating faultobs1-noamp's own delta as the fault-observability effect alone. FAIL = any real regression vs faultsmoke1-control beyond noise -- the transplant itself (not fault-seeing) would be the confound to fix before trusting the paired noamp arm.

**verdict**: No-op control confirms the fault-observability obs-pad transplant costs nothing on its own. DR-0 gate: gait_valid 6/6 det+sto, zero sacrificed legs, det prog med 1.01/slip 2.86/fwd 0.61m, sto prog med 0.86/slip 3.82/fwd 0.84m -- within 6-ep noise of the faultsmoke1-control baseline (prog 1.16/0.88, slip 2.36/4.08); frame-strip (walk_det_4) clean six-leg alternating-tripod cycling, no drag/skate/flag-leg. Confirms the transplant itself introduces no regression, so any delta measured on the paired faultobs1-noamp arm (fault injection + the new health-vector obs) can be attributed to the fault-observability mechanism, not this plumbing change. hardware-ready: no (2M discovery continuation, DR-0, no faults injected in THIS control arm).

