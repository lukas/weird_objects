# cw-amp-m3-pushhard1-noamp-n2040

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T00:19:22+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m3-pushacq1-noamp

**wandb_id**: md1fco72

**hypothesis**: Plain English: companion arm to cw-amp-m3-pushhard1-noamp-repeat3-r1, escalating per-shove FORCE instead of push COUNT, on the noamp line that just read clearly better than style05 at the single-push acquisition gate (0/12 vs 1/6+0/6 terminations). Doubles the shove range from 10-25N to 20-40N (dr.ext_push_n=20,40), single shove per episode, everything else identical to pushacq1-noamp. Mirrors the already-running cw-amp-m3-pushhard1-style05-n2040 arm exactly, decomposing the same push-COUNT-vs-push-FORCE question the style05 escalation grid opened, on the better-reading substrate. Prediction-if-true: DR-0 own-cfg gate under the harder single push holds gait_valid >=5/6 det+sto with topples <=1/6 det, <=2/6 sto (i.e. force headroom exists beyond 10-25N) and training tilt-terminations fall over the 6M budget from an elevated Q1 starting point. Prediction-if-false: terminations stay high/flat -- 20-40N is past this policy's recoverable envelope, naming per-shove force (not count) as the harder M3 axis. Strongest alternative: force and count both transfer easily from the clean single-push floor, meaning the whole M3 push-hardening problem was mostly about getting the FIRST push right, which pushacq1-noamp already did.

**gate**: Acquisition (6M, DR-0, 20-40N single shove/episode). Joint read vs cw-amp-m3-pushhard1-style05-n2040 (same lever, style-kept twin) AND vs cw-amp-m3-pushhard1-noamp-repeat3-r1 (count-escalation twin on the same base checkpoint): PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, topples <=1/6 det AND <=2/6 sto, zero sacrificed legs, det prog med >=0.9, video shows genuine recovery from the harder shove (not a lucky miss). INFORMATIVE-plateau = topples clearly worse than the 10-25N floor with training tilt-terms flat over the last 2M => 20-40N exceeds this budget's recoverable force range, names a force curriculum (not a flat jump) as the next lever. FAIL = collapse/statue/numerical blowup.

