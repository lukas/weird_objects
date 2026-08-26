# cw-standwalk-stance-mesh2-standheight-rung5-acq8m-segfix-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T08:04:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-standheight-rung5-acq8m-s1

**wandb_id**: i50v00p9

**hypothesis**: Seed-1 twin of the segment-window widening test (9-11s vs default 6-8s): does it fix seed1's own sharper flat-start rise residual (7/12 in the corrected-contract seqprobe, mostly hold_low_height stalls on flat draws)? Same code-read root cause as the seed-0 twin: rise's own schedule needs >=7.0s and the default segment draw sometimes lands shorter. Continuation off seed-1's own acq8m checkpoint.

**gate**: PASS: composed seqprobe's flat-start rise sub-count improves (fewer hold_low_height terms) with hold/lower staying >= this seed's own acq8m level (6/6+6/6 lower, 6/6+6/6 hold). FAIL: unchanged/worse -- joint with the seed-0 twin per the same disagreement convention this campaign uses throughout.

