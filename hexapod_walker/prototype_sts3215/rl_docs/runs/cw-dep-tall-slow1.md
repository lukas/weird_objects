# cw-dep-tall-slow1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-11T22:05:47+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-dep-tall30

**hypothesis**: TALL LADDER T4: speed trade. Shrink the commanded band 0.05-0.06 -> 0.03-0.04 at ref -15, warm from tall30. If walk income pressure is what pins the body at -44mm, easing the speed demand should free posture: expect height_err_end to drop below 15mm. Recover speed in a later rung if it works.

**gate**: PASS: height_err_end <=8mm at -15 ref, speed within the 0.03-0.04 band, survived 1, slip <=1.8. If PASS: rung speed back up (0.04-0.05) with the height held. FAIL: err stuck ~29mm = speed pressure is not the pin; the wall answer is T5s probe.

**refused_reason**: hexapod-mjx-train-4 code marker 39d6355ea869cf2fc02ab8625ea9dfecfd7eccb5 != local HEAD de02e489a09ed7ed04dbdf08a7c15bbb0e332524. Sync first: snapshot.sh --sync hexapod-mjx-train-4 (and snapshot/commit before that if the tree is dirty).

