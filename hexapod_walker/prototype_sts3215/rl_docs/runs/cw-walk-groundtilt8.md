# cw-walk-groundtilt8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T23:51:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**hardware_ready**: False

**hypothesis**: Ladder rung off the PASSED groundtilt5 (0-5deg marginal: 2/6 steepest draws shuffle at 1/3 speed, no falls). One variable: widen tilt range to u(0,8deg), warm-start from groundtilt5's own checkpoint (continuation, like the lowgait crouch ladder). If-true: own-cfg gv 12/12, 0 term, det med fwd still >=1.1m, no NEW falls at the steeper end -- ceiling extends past 5deg. If-false: falls or flag-leg pathology appear at 6-8deg (not just slower shuffle) -- ladder tops out between 5 and 8, matching lowgait's find-the-ceiling pattern. Strongest alternative: median holds only because shallow draws dominate the sample while steep draws silently worsen from shuffle to fall.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=8.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on steepest draws; DR0 flat retention det 6/6 gv, slip/m <=1.24; compare episode pattern to groundtilt5 at triage; frames watched det

**verdict**: Launch failure, not a science result: never appeared RUNNING in W&B within 240s (fleet launch-collision storm, gotcha 13b) - 0 steps trained. No verdict on the 8deg tilt ladder rung; groundtilt5 remains top of confirmed ladder.

**failed_reason**: run never appeared as 'running' in W&B within 240s

