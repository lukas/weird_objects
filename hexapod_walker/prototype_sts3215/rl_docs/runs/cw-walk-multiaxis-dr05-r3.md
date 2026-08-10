# cw-walk-multiaxis-dr05-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:41:54+00:00

**pod**: hexapod-mjx-train-0

**steps**: 14000000

**parent**: cw-walk-multiaxis-dr05-r2

**hypothesis**: 3rd attempt (r1+r2 both died at init 0 steps, fleet launch-collision storm gotcha 13b -- no science result yet). Same spec unchanged: multiaxis-dr05 (multiaxis1's 4-axis DR0 stack + generic dr-scale=0.5) passed its own-cfg exposure gate clean but missed the pre-registered DR0 nominal retention cap by a small margin (slip 1.27 vs 1.24 cap). +14M more steps tests whether the miss is under-training (closes with more steps) or a hard stacking ceiling (multiaxis1 stands as the robustness-champion base either way). If-true: extra steps close the retention gap under 1.24. If-false: retention slip stays >1.24 even with more steps -- confirms a real ceiling at 4 axes + generic DR0.5, don't stack further.

**gate**: Own-cfg (4-axis stack + DR0.5) det+sto 6/6: gait_valid 6/6, 0 term, prog med>=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; frames watched det.

**failed_reason**: run never appeared as 'running' in W&B within 240s

