# cw-walk-payload70

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T00:30:09+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-payload50

**wandb_id**: z8p7qfnu

**hardware_ready**: False

**hypothesis**: Ladder rung off the payload50 PASS (top of range 1.4-1.5x mass was marginal: 2/6 det draws squat-shuffle at ~half speed). One variable off payload50's own checkpoint: widen mass range to dr.mass_scale=1.0,1.7 (like the groundtilt5->8 and lowgait ceiling-finding pattern). If-true: own-cfg gv 12/12, 0 term, det med fwd >=1.1m, no NEW falls/flag-leg at the heavier end -- ceiling extends past 1.5x. If-false: falls or flag-leg pathology appear (not just slower shuffle) -- payload ceiling sits between 1.5x and 1.7x. Strongest alternative: median holds only because light draws dominate while heavy draws silently worsen from shuffle toward failure.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.mass_scale=1.0,1.7, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on heaviest draws; DR0 no-payload retention det 6/6 gv, det slip/m <=1.24; compare episode pattern to payload50 at triage; frames watched det

**verdict**: PASS (letter met, soft ceiling not a hard wall) — payload ladder rung widening mass_scale 1.0-1.5x (payload50) to 1.0-1.7x. Own-cfg det+sto 6/6 @30s: gv 12/12, 0 term, det med fwd 1.20m (>=1.1m gate). 2/6 heaviest det draws squat-shuffle at 40-54% speed (prog 0.40/0.54, slip/m 2.81/3.70, fwd 0.72-0.81m) -- the strongest-alternative outcome predicted in the hypothesis (median holds because light draws dominate) DID happen, same severity band as payload50's own top-of-range tail (slip 3.4-3.8, ~half speed), not a new/worse pathology, no falls/flag-leg. DR0 no-payload retention: det clean 6/6 gv (slip/m med 1.12<=1.24 cap, prog 0.97, fwd 1.48m = champion band, no erosion); sto showed one severe near-stall outlier (prog 0.10, slip 15.18, near in-place churn, no fall/flag-leg) -- matches the well-documented rare fixed-draw stochastic-stall canary class, non-gating (gate specifies det only). Frames watched det: level body, six legs cycling even on the heaviest/slowest draws. Conclusion: the ceiling is soft and gradual, not a wall between 1.5x and 1.7x -- widening the range costs median distance (1.20m vs payload50's 1.31m) without buying new failure information; payload50 remains the safer promotion point, payload70 mainly documents that the same shuffle-degradation pattern stretches further. Not hardware-ready.

