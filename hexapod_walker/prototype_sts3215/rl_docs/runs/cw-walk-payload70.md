# cw-walk-payload70

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T00:30:09+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-payload50

**wandb_id**: z8p7qfnu

**hypothesis**: Ladder rung off the payload50 PASS (top of range 1.4-1.5x mass was marginal: 2/6 det draws squat-shuffle at ~half speed). One variable off payload50's own checkpoint: widen mass range to dr.mass_scale=1.0,1.7 (like the groundtilt5->8 and lowgait ceiling-finding pattern). If-true: own-cfg gv 12/12, 0 term, det med fwd >=1.1m, no NEW falls/flag-leg at the heavier end -- ceiling extends past 1.5x. If-false: falls or flag-leg pathology appear (not just slower shuffle) -- payload ceiling sits between 1.5x and 1.7x. Strongest alternative: median holds only because light draws dominate while heavy draws silently worsen from shuffle toward failure.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.mass_scale=1.0,1.7, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on heaviest draws; DR0 no-payload retention det 6/6 gv, det slip/m <=1.24; compare episode pattern to payload50 at triage; frames watched det

