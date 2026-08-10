# cw-dep-startvar1-noBS1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:05:12+00:00

**pod**: hexapod-mjx-train-9

**steps**: 18000000

**parent**: cw-dep-startvar1-r1

**wandb_id**: msxrbsn4

**hardware_ready**: False

**hypothesis**: Second, parallel isolation arm for the cw-dep-startvar1-r1/-s1 FAIL (both showed systemic gait breakdown: all det episodes degraded, one real sacrificed leg, slip/m up to 22, declining reward through training). cw-dep-startvar1-noZD1 (zero_drift_cmd_frame->0, running) tests the prime suspect. This arm tests the alternative in parallel given the hardware-window time pressure: keep zero_drift_cmd_frame=1 and placement_noise_deg=6, but remove dr.bad_start_prob (0.4->0.0) -- bad-start (starting from a slumped/park pose) stacked with a drifted logical zero could plausibly be the interaction that breaks foot-placement timing, independent of whether zero-drift alone is fine. If-true: gait recovers (no flag legs, slip/m near vref1-r1 band, reward climbs not declines) -- bad_start_prob x zero_drift is the interaction, not zero_drift alone. If-false: still broken with bad-start removed -- points back to zero_drift_cmd_frame itself (or placement_noise) as sufficient on its own; compare against noZD1's result to triangulate.

**gate**: Own-cfg (contract + placement6+zerodrift3+k_current=0, NO bad-start) det+sto 6/6 at 15s: gait_valid 12/12 (0 sacrificed legs), 0 term, slip/m within vref1-r1 own band (det ~0.89, sto ~1.13) +-20%; reward quarters climbing or flat, not declining; frames watched det

**verdict**: FAIL (partial improvement, informative) -- isolation arm removing dr.bad_start_prob (0.4->0) from the startvar1-r1 FAIL recipe (placement6 + zerodrift-frame3 + k_current=0) does NOT recover the own-cfg gate, but it materially improves it. vs r1 (0/6 det clean, real sacrificed-leg episode, worst-case slip 21.99): noBS1 det is 4/6 clean (fwd 0.55-0.73m, no flag leg, gv True every episode) and only 2/6 still pathological -- ep3 mild (slip 2.27, fwd 0.31m) and ep4 catastrophic (slip 28.5, prog 0.03, march-in-place per frames, NOT a flag leg). sto is fully clean 6/6 (prog med 0.94, slip med 1.12 -- inside vref1-r1s own sto band). Own-cfg det slip med 1.49 still sits well above the vref1-r1 det band (0.89 +-20%) and 2/6 catastrophic episodes remain, so the gate is not met. Conclusion: bad_start_prob is A contributor (removing it eliminates the flag-leg failure mode and clears 4/6 previously-uniform-degraded episodes) but NOT the sole cause -- zero_drift_cmd_frame (still =1 here) or placement_noise, or their interaction with it, still produces an intermittent catastrophic-skate failure mode on 1-2/6 draws. Triangulate against the parallel cw-dep-startvar1-noZD1 arm (zero_drift_cmd_frame->0, bad_start kept) to isolate further; if noZD1 is fully clean, zero-drift-frame is the dominant culprit and noBS1s residual failures are its interaction with placement-noise alone. Do not use for hardware; cw-dep-vref1-r1 remains the recommended base for attempt #2.

