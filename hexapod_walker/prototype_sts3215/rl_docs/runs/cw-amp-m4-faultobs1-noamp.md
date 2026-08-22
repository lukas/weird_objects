# cw-amp-m4-faultobs1-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T22:11:26+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-faultsmoke1-noamp

**wandb_id**: wdxyl3uf

**hypothesis**: Plain English: if the walking policy can actually SEE which joint fault it has (18-dim fault_health() vector -- 1.0 healthy / 0.0 dead-frozen / intermediate = weakened, now wired into obs per the M4 obs-wiring lever built this cycle, test_fault_injection.py 14/14) instead of compensating blind, does it degrade LESS under the identical fault distribution than the faultsmoke1-noamp baseline did (det gait_valid 5/6, prog_ratio 0.49/slip 5.99 on the faulted episode)? Single lever vs faultsmoke1-noamp: obs.fault_health 0->1 + --obs-pad-transplant 18 (zero-pads the new input columns so the transplanted policy starts bit-identical to the parent until training moves them), same dr.fault_prob=1.0 fault_mix, same 2M budget, same everything else, still warm-started from the pre-fault ppo_goal_cw_amp_m2_bcinit_sec5_noamp checkpoint.

**gate**: Discovery continuation (2M, DR-0). Read jointly against faultsmoke1-noamp (blind) and faultobs1-control (obs on, no fault) launched alongside. PASS/adaptation-works = det gait_valid >=5/6 AND the faulted-leg episode's prog_ratio/slip_per_m measurably beats faultsmoke1-noamp's degraded episode (prog_ratio > 0.49, slip/m < 5.99) by more than 6-ep noise, with video showing visible fault-specific compensation (e.g. favoring the healthy legs, not just generic softer gait). FAIL/no-benefit = degraded episode's numbers are statistically indistinguishable from the blind baseline (obs present but unused) or WORSE (transplant destabilized training). Does not gate M4/M5 alone -- first real M4 adaptation reading.

