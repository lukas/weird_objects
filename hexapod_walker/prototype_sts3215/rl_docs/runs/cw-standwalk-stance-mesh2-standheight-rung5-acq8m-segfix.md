# cw-standwalk-stance-mesh2-standheight-rung5-acq8m-segfix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T08:00:42+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-standheight-rung5-acq8m

**wandb_id**: a9jnk4dr

**hypothesis**: Does widening the composed mode_seq segment window (9-11s vs the default 6-8s) fix the flat-start rise failures the corrected-contract seqprobe revealed? Code-read root cause: this recipe's own rise schedule needs >=7.0s (1s hold + rise_ramp_s=6.0) to reach the commanded height once, but the default segment draw U(6,8) lands under 7.0s roughly half the time when rise is the sequence's first segment, cutting a flat start's full physical climb short before it completes and riding a still-low height into the next segment (hold_low_height fires there). Same mechanism class as this file's already-diagnosed lower-phase segment-timing residual. Continuation off the promoted acq8m checkpoint (seed 0), no skill relearning needed, pure timing-budget test.

**gate**: PASS: composed seqprobe's flat-start rise sub-count improves (fewer hold_low_height terms on flat/bridge draws specifically) with hold/lower staying >= the acq8m checkpoint's own level (no new majority term). FAIL: flat sub-count unchanged or worse -- refutes the segment-timing hypothesis, points to genuine skill interference requiring a different (multi-teacher/KL) mechanism, dig-in scope.

