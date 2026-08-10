# cw-dep-startvar1-noBS1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T07:05:12+00:00

**pod**: hexapod-mjx-train-9

**steps**: 18000000

**parent**: cw-dep-startvar1-r1

**hypothesis**: Second, parallel isolation arm for the cw-dep-startvar1-r1/-s1 FAIL (both showed systemic gait breakdown: all det episodes degraded, one real sacrificed leg, slip/m up to 22, declining reward through training). cw-dep-startvar1-noZD1 (zero_drift_cmd_frame->0, running) tests the prime suspect. This arm tests the alternative in parallel given the hardware-window time pressure: keep zero_drift_cmd_frame=1 and placement_noise_deg=6, but remove dr.bad_start_prob (0.4->0.0) -- bad-start (starting from a slumped/park pose) stacked with a drifted logical zero could plausibly be the interaction that breaks foot-placement timing, independent of whether zero-drift alone is fine. If-true: gait recovers (no flag legs, slip/m near vref1-r1 band, reward climbs not declines) -- bad_start_prob x zero_drift is the interaction, not zero_drift alone. If-false: still broken with bad-start removed -- points back to zero_drift_cmd_frame itself (or placement_noise) as sufficient on its own; compare against noZD1's result to triangulate.

**gate**: Own-cfg (contract + placement6+zerodrift3+k_current=0, NO bad-start) det+sto 6/6 at 15s: gait_valid 12/12 (0 sacrificed legs), 0 term, slip/m within vref1-r1 own band (det ~0.89, sto ~1.13) +-20%; reward quarters climbing or flat, not declining; frames watched det

