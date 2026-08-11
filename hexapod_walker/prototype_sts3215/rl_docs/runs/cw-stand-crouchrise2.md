# cw-stand-crouchrise2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-11T17:26:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**hypothesis**: DISCOVERY (2M, warm from cw-stand-holdbc1-hard1, NOT from crouchrise1 — its hold cheat may be baked in): crouchrise1 proved the crouch-60% start mix fixes the lineage's last defect (det crouch 5/5, RSI-off 8/8 vs hard1 0/8) but its second delta — goal-mix skew rise 0.45->0.6 / lower 0.45->0.3 — starved lower and let the flag-leg hold cheat back in (det hold feet 1+4 duty 0.07/0.01; valid_plant 7/12). crouchrise2 = ONE variable vs crouchrise1: restore hard1's exact goal mix (hold=0.1,rise=0.45,lower=0.45), keep goal.rise_flat_frac=0.10 / rise_partial_frac=0.30. Same holdbc1 stack, seed 13.

**gate**: PASS if det crouch-start rise valid >= 3/4 with zero tilt falls AND hold retention at hard1 level: hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet (explicit — crouchrise1's flag-leg hold passed valid_plant=True; duty is the telemetry that caught it) AND det flat/bridge rise not worse than hard1, no flag-leg/tripod cheat on video. FAIL if crouch <= 2/4 or hold/duty regress. PASS -> replaces hard1 as the stance deploy candidate; the robot export MUST ship WITH its goal-ramp profile (08-10 stale-push lesson) and the stand lineage closes.

**refused_reason**: hexapod-mjx-train-0 code marker bf38df2d2df6bc6d5521d277f386281e0b430a5f-dirty != local HEAD bf38df2d2df6bc6d5521d277f386281e0b430a5f. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

