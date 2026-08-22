# cw-amp-m4-faultsmoke1-control

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T21:35:05+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp

**hypothesis**: Plain English: matched control for cw-amp-m4-faultsmoke1-noamp -- identical 2M-step continuation of the same BC-init walking checkpoint (cw-amp-m2-bcinit-sec5-noamp) with dr.fault_prob explicitly held at 0.0 (bit-exact no-op, same as the source run's own default). Isolates whether any reward/eval/behavior delta seen in the fault-injection twin is caused by the fault mechanism itself or is just noise from continuing training 2M more steps on the same checkpoint/seed.

**gate**: Discovery smoke (2M), read only jointly against cw-amp-m4-faultsmoke1-noamp. Expect near-identical continuation of the parent's walking behavior (gait_valid stays 6/6, no crouch, fwd travel comparable to the 2M parent). Any large unexpected drift here (not just the fault twin) would mean the delta between the two arms is not attributable to fault injection.

**verdict**: Mechanism-safe control PASS. Matched no-op twin (dr.fault_prob=0.0) continuing the proven bcinit-sec5-noamp walking checkpoint 2M more steps: reward rose every quarter (31.7/134.0/237.9/320.8, finite throughout, fps 12731), DR-0 gate gait_valid 6/6 det+sto (was 6/6 at parent), prog_ratio improved to 1.16 det/0.88 sto (parent 1.04/0.55), slip/m held/improved (2.36 det/4.08 sto vs parent 2.17/5.34), dir_err improved (35.2 det/52.4 sto vs 44.2/63.4). Video/contact sheets watched: clean six-leg alternating gait, no crouch, no new sacrificed legs, height stable. Confirms the fault twin's own delta is attributable to the fault mechanism, not just 2M extra training steps. Closes the isolation half of the faultsmoke1 pair; see cw-amp-m4-faultsmoke1-noamp for the fault-injected read.

