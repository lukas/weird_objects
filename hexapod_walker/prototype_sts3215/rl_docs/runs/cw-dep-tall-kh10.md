# cw-dep-tall-kh10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-11T22:06:00+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-tall30

**hypothesis**: TALL LADDER T2b: k_height crank 10x (100->1000) at ref -15, warm from tall30. At 60mm err this charges ~3.6/tick = parity with walk income; the crouch can no longer outbid it arithmetically. Risk: quadratic charge this big may suppress walking itself (freeze incentive).

**gate**: PASS: height_err_end <=8mm at -15 ref, speed >=0.028, survived 1, slip <=1.8, no park. FAIL modes to distinguish: err flat (charge STILL outbid = pricing refuted at any sane dose) vs walk collapse/park (charge too blunt).

**refused_reason**: hexapod-mjx-train-3 code marker 39d6355ea869cf2fc02ab8625ea9dfecfd7eccb5 != local HEAD 558207d02bf2888b2b7d36dac811b69dd5cc5d53. Sync first: snapshot.sh --sync hexapod-mjx-train-3 (and snapshot/commit before that if the tree is dirty).

