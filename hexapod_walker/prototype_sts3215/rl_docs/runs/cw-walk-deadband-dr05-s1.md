# cw-walk-deadband-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T23:21:46+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-deadband-dr05

**wandb_id**: qtl7uqs9

**hardware_ready**: False

**hypothesis**: Seed twin of PASS-with-caveat cw-walk-deadband-dr05 (DR0.5 + servo deadband 1-3x compose off champion). Ruling-7 promotion-panel completeness -- same config, seed 1. The seed0 caveat was DR0 nominal retention slip/m 1.22 sitting right at the 1.24 cap edge with mild fwd shading (1.42 vs champion 1.57). If-true: seed1 matches the same tight-but-passing retention band, confirming the shading is a structural (small, tolerable) tax of this compose, not seed luck. If-false: seed1's retention crosses the 1.24 cap (fails outright) or is clearly cleaner (no shading at all) -- either way the seed0 result was noise, not a stable characterization.

**gate**: Own-cfg harness DR0.5 + dr.deadband_scale=1.0,3.0 det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m; DR0 nominal retention det 6/6 gv, slip/m <=1.24 (compare margin to seed0's 1.22); frames watched det for lurching

**verdict**: PASS. Seed-1 twin of the PASS-with-caveat cw-walk-deadband-dr05: own-cfg DR0.5+deadband1-3x det+sto 6/6 gv, 0 term, det med fwd 1.36m (>=1.1 gate). DR0 nominal retention slip/m med 1.18 (<=1.24 cap, cleaner margin than seed0's 1.22) with fwd 1.47m (seed0 was 1.42 vs champion 1.57) -- confirms the mild nominal-tracking shading from this compose is a small structural tax of servo-deadband-under-DR, not seed luck. Frames det: clean six-leg cycling on both best and worst draws, no lurching/flag-leg. Ruling-7 promotion-panel COMPLETE for the deadband-dr05 compose (2/2 seeds).

