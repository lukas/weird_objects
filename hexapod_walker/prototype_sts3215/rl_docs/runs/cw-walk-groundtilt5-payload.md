# cw-walk-groundtilt5-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:06:41+00:00

**pod**: hexapod-mjx-train-3

**steps**: 18000000

**parent**: cw-walk-groundtilt5

**hypothesis**: NEW compose, untried pairing: floor-slope robustness (5deg tilt, PASSED+seed-confirmed ruling-7) x payload (mass_scale 1.0-1.5x, validated range). Every groundtilt-dr05 compose so far paired slope with generic DR0.5, not specifically payload; every payload compose so far paired mass with the flat/driving-package gaits, not slope. ONE variable off cw-walk-groundtilt5: add dr.mass_scale=1.0,1.5 cfg-set (payload only). Directly relevant to hardware: carrying something across a sloped/uneven real-room floor. Prediction-if-true: own-cfg panel (tilt5+payload) gv 12/12, 0 term, det median fwd >=1.2m; DR0 no-payload-no-tilt retention clean (slip<=1.24). Prediction-if-false: payload tips the slope-compensation strategy over (uphill draws stall harder under load, or downhill slide worsens) - slope+payload doesn't compose freely. Strongest alternative: passes own-cfg but nominal retention erodes like payload-dr05 c61 (mass-axis DR compose charges nominal walking) - would be a 3rd example of that pattern.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=5.0 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal (no tilt, no payload) retention det 6/6 gv, det slip/m <=1.24; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

