# cw-dep-tip1-takeoff25-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-11T23:36:52+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-tip1

**hypothesis**: Make the walking champion survive the 20-25 degree takeoff wobble that knocks the real robot over right after gait start; this arm raises the tipped-start training regime to the tilt range actually measured on hardware. Correct relaunch of cw-dep-tip1-takeoff25 (the drained stub trained default DR — no science). Hardware evidence (camera bench 08-11 eve, bench_blast_20260811_18*/19*): BOTH deployed walkers show a 20-25deg takeoff roll transient; vref1 fell 3/3, tip1 clean 1/1 but tripped 2/3 fwd attended — every fall keeps the takeoff-transient shape. tip1 trained at dose 6-18deg prob 0.30, BELOW the measured regime. One variable vs tip1: dr.tipped_start_prob 0.30->0.5, dr.tipped_start_deg 6-18 -> 12-25 (sim_env dr.* override path verified, tuple parses).

**gate**: SCORE/tipped_recovery_success at 20-25deg injections det+sto materially above the matched parent under the IDENTICAL injection (eval_checkpoint --baseline cw-dep-tip1); DR0 walk retention unchanged (gait_valid 6/6, prog med >= 0.85, no paddle); frames watched det.

