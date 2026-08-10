# cw-walk-torquescale-r2-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T03:16:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-walk-torquescale-r1

**hypothesis**: 3rd launch attempt: base + r1 both died 0-steps to launch-collision/stale-code-marker infra (gotcha 13b + a stale train-pod .code_sha this window, now synced). Same spec unchanged: OPERATOR WISHLIST 13b torque-droop-under-load axis off cw-walk-longdist-r2.

**gate**: own-cfg torque-droop-DR det+sto gv 6/6 @30s, 0 term, det med fwd within champion band; DR0 nominal-torque retention det 6/6 gv, slip<=1.24; frames watched det

