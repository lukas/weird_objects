# cw-walk-multiaxis1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T19:31:13+00:00

**pod**: hexapod-mjx-train-9

**steps**: 18000000

**parent**: cw-walk-longdist-r2

**hypothesis**: Compose rung (operator 08-09, fill idle slot): the four INDIVIDUALLY validated physics axes (payload 1.0-1.4x mass, latency 0.5-2.5x, deadband 1.0-3.0x, CoM +30mm) randomized TOGETHER at dr-scale 0 off the champion. If-true: axes compose without interference (multiaxis own-cfg gv 12/12, 0 term, det med fwd >=1.2m; DR0 nominal retention clean) - recipe becomes the robustness-champion base. If-false: interference collapses gait (terminations/prog craters) - compose needs staging (add one axis per rung) not all-at-once. Strongest alternative: policy passes by finding one conservative crouch gait that ignores the axes - height_err/duty/frames will show it.

**gate**: Own-cfg harness at dr-scale 0 + all four dr overrides, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv; frames watched det

