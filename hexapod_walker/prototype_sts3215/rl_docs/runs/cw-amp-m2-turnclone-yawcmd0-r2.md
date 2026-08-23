# cw-amp-m2-turnclone-yawcmd0-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T00:08:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd

**hypothesis**: Retry of cw-amp-m2-turnclone-yawcmd0 (REFUSED-at-runtime: crashed at step 0, '--obs-pad-transplant 1 but obs widened by 0 (75 -> 75)' -- the turn-clone checkpoint already carries the wz_ref obs column natively (built with goal.walk_yaw_cmd=1 from the start), so unlike the original yawcmd run's own 73->74 pad-transplant off the yaw-blind headingsfull checkpoint, THIS init needs zero padding). Single lever now correctly isolated: --obs-pad-transplant 0 (no-op plumbing fix) + --init-from swapped to the turn-clone. Same plain English question as the crashed attempt: does a BC init that was actually taught to turn do better than the original yawcmd run (parked, tip err 0.2995/0.3008 == |wz_ref|, command-invariant left drift while translating) even WITHOUT dedicated tip exposure (frac=0, arc-turns-while-translating + occasional stress_mix zero-speed segments only)?

**gate**: Discovery continuation (2M, DR-0). Manual eval_yaw on the run's own pod: PASS/PARTIAL/FAIL-washout per the tip50/tip90-turnclone siblings' bar, judged against this arm's own frac=0 command mix.

