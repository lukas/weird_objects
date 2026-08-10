# cw-walk-payload-dr05-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:20:42+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-payload-dr05-s1

**hypothesis**: 3rd seed to settle the payload-dr05 1-1 split (seed0 FAIL c61: DR0 no-payload retention slip 1.38>1.24, prog 0.54; seed1 PASS this cycle: slip 1.23<=1.24, prog 0.93). Tie-breaker per ruling-7. Prediction-if-true (matches seed1): retention clean (slip<=1.24, prog>=0.75) - majority PASS, seed0 was the outlier, mass-axis DR compose does NOT systematically charge nominal walking. Prediction-if-false (matches seed0): retention erodes again - majority FAIL, seed1 was the outlier, the mass-DR-charges-nominal tax is real and payload-dr05 stays closed as a compose.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd >=1.1m; plus DR0 no-payload retention det 6/6 gv, det slip/m med <=1.24 AND prog med >=0.75; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

