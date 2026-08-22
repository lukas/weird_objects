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

**verdict**: Sanity control confirms the obs-pad transplant is a true no-op: with dr.fault_prob=0 (fault_health always all-ones), DR-0 gate is statistically indistinguishable from faultsmoke1-control's own numbers -- det gait_valid 6/6 (vs 6/6), prog_ratio 1.10/0.88 det/sto (vs 1.16/0.88), slip/m 2.73/4.04 (vs 2.36/4.08) -- every delta within 6-ep noise, no regression from widening the obs by 18 dims. This isolates faultobs1-noamp's measured +18% prog / -27% slip improvement (on the identical fault draw vs faultsmoke1-noamp) as the fault-observability effect itself, not a transplant artifact. Reward finite/rising (mirrors the control lineage). hardware-ready: no (2M continuation, DR-0).

