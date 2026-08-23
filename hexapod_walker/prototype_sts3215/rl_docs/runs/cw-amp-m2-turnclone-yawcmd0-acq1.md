# cw-amp-m2-turnclone-yawcmd0-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T00:27:02+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**hypothesis**: Plain English: yawcmd0-r2 (the BC-turn-clone's zero-tip-exposure control) is the best-rounded M2 arm to date -- real command-signed turning (tip err 0.15/0.16, both <=0.20) AND clean full-heading translation (det gait_valid 6/6, slip 1.9-2.8/m, progress_ratio 0.80-1.13, sto 6/6 too) on the SAME checkpoint, no tip curriculum needed. Does a 3x budget acquisition continuation (matching the M3 push-track's discovery->acquisition pattern) tighten the turn-in-place accuracy further (ideally under eval_yaw's own strict 0.10 rad/s default gate, not just this run's looser <=0.20 bar) while holding the translation quality, the same way pushacq1 converted pushsmoke1's 'survive-most' into 'survive-every-sampled-push'?

**gate**: Acquisition (6M continuation, DR-0). Manual eval_yaw (own cfg): PASS-full = tip-left AND tip-right err <=0.10 (clears eval_yaw's own built-in gate, not just this lineage's looser bar) with correct sign, zero falls, translation still gait_valid 6/6 det+sto with slip/m and progress_ratio not degraded >20% vs yawcmd0-r2. PASS-partial = tip errs improve but stay in the 0.10-0.20 band. FAIL = regresses toward yawcmd0-r2's own 0.15/0.16 or worse, or translation erodes -- 3x budget does not buy more accuracy at this pricing, next lever is reward-side (tighter yaw kernel / more yaw income) not more steps.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

