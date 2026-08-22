# cw-amp-m4-faultsmoke1-control

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T21:35:05+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp

**hypothesis**: Plain English: matched control for cw-amp-m4-faultsmoke1-noamp -- identical 2M-step continuation of the same BC-init walking checkpoint (cw-amp-m2-bcinit-sec5-noamp) with dr.fault_prob explicitly held at 0.0 (bit-exact no-op, same as the source run's own default). Isolates whether any reward/eval/behavior delta seen in the fault-injection twin is caused by the fault mechanism itself or is just noise from continuing training 2M more steps on the same checkpoint/seed.

**gate**: Discovery smoke (2M), read only jointly against cw-amp-m4-faultsmoke1-noamp. Expect near-identical continuation of the parent's walking behavior (gait_valid stays 6/6, no crouch, fwd travel comparable to the 2M parent). Any large unexpected drift here (not just the fault twin) would mean the delta between the two arms is not attributable to fault injection.

