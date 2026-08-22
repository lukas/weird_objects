# cw-amp-m4-faultsmoke1-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T21:31:29+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp

**wandb_id**: f3irueeu

**hypothesis**: Plain English: can the just-proven BC-init walking policy (cw-amp-m2-bcinit-sec5-noamp, gait_valid 6/6, real forward travel, no crouch) keep training and stay numerically stable when every episode injects one random joint fault (weakened servo / frozen joint / disabled leg) via the newly built dr.fault_* mechanism (M0 checklist item, test_fault_injection.py 10/10 PASS) -- a mechanism that has NEVER been exercised in an actual training run yet, only unit-tested. This is a SMOKE arm per that mechanism's own documented discipline ('first training use should ride a 2M smoke arm'): it checks the training loop stays healthy (finite losses/values, no crash) and reports what a walking hexapod visibly does under joint faults, NOT a graded M4 capability test -- the fault health vector is not wired into actor obs yet (separate M4 work), so this measures INNATE fault tolerance/degradation on an unaware policy, not adaptation. Single lever vs the finished checkpoint: dr.fault_prob 0.0->1.0 (default fault_mix 0.45 weak/0.25 frozen/0.30 disabled-leg), 2M more steps, everything else (sec5 minimal reward, forward-only 0.08 m/s command, DR-0 otherwise) byte-identical, warm-started from the noamp checkpoint. Matched twin cw-amp-m4-faultsmoke1-control (identical continuation, dr.fault_prob=0.0) isolates whether any behavior/metric change is the fault or just 2M more steps of training. (Note: an identically-named cw-amp-m4-faultsmoke-noamp attempt by a concurrent cycle hit a pod-busy REFUSED + left a stale git tag; this is a fresh distinctly-named attempt of the same idea, not a duplicate in flight.)

**gate**: Discovery smoke (2M). PASS/mechanism-safe = run completes with finite losses/values the whole 2M (no NaN/crash), checkpoint loads, and det video under dr.fault_prob=1.0 shows the hexapod attempting locomotion / visible compensation under a fault (not instant collapse every single episode). FAIL = NaN/blowup or crash before completion, or catastrophic fall in effectively all episodes regardless of fault type/severity -- mechanism needs a fix (or M4 obs-wiring must come BEFORE any more training) before another arm. Read jointly against the -control twin: any reward/eval delta beyond noise vs -control is the first measured cost of bare (unobserved) fault injection. Does NOT gate M4 or M5 -- only clears dr.fault_* for real training use.

