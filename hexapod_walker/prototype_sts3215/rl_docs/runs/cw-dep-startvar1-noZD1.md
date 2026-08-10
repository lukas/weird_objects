# cw-dep-startvar1-noZD1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T06:54:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 18000000

**parent**: cw-dep-startvar1-r1

**wandb_id**: 6vc03dsq

**hardware_ready**: False

**hypothesis**: Root-cause isolation for cw-dep-startvar1-r1/-s1 FAIL (both det+seed-twin show systemic gait breakdown: ALL 6 det episodes degraded vs vref1-r1 own band, one real sacrificed-leg episode, slip/m up to 22, reward quarters DECLINING through training 439->347 / 487->347 unlike every other compose tonight which climbs/plateaus). Pre-registered if-false said needs isolating which DR axis is the culprit. Prime suspect: dr.zero_drift_cmd_frame=1 -- brand-new mechanism, only probe-smoked in isolation, never trained at 18M-step scale; it silently offsets the physical joint frame the policy relies on for foot placement, which could plausibly break a walking gaits timing/coordination in a way placement-noise and bad-start (both independently proven benign elsewhere: placementnoise6-r3 PASS, payload/comshift composes routinely absorb bad-start-like exposure) would not. One variable off startvar1-r1: zero_drift_cmd_frame 1 to 0, everything else identical (placement6+badstart0.4+k_current=0, warm-start vref1-r1). If-true: gait recovers to near vref1-r1 own band (no flag legs, slip/m back near 0.9-1.3, reward climbs/plateaus not declines) -- zero-drift-frame is the culprit, needs its own smaller-scale ablation before retrying. If-false: still broken -- placement6+badstart0.4 (or their interaction) is the culprit instead, or k_current=0 removes a stabilizing income term; escalate to a full one-axis-at-a-time ladder.

**gate**: Own-cfg (contract + placement6+badstart0.4+k_current=0, NO zero-drift) det+sto 6/6 at 15s: gait_valid 12/12 (0 sacrificed legs), 0 term, slip/m within vref1-r1 own band (det ~0.89, sto ~1.13) +-20%; reward quarters climbing or flat, not declining; frames watched det

**verdict**: FAIL against its own gate, but the diagnostically important half of the isolation pair. Removing zero_drift_cmd_frame (this ablation: placement6+badstart0.4+zerobias3 kept, zero-drift-frame OFF) ELIMINATES the catastrophic mechanical failure the full startvar1-r1/-s1 compose showed: gait_valid 6/6 det here (no sacrificed leg anywhere), vs r1's real flagged/sacrificed leg in ep3 and s1/r1 det slip up to ~22. BUT slip/prog do not recover to vref1-r1's clean band: det med slip 1.73 (band ~0.71-1.07), sto med slip 1.36 (right at the upper edge of the ~0.9-1.36 band); one det episode (idx4) still craters to the lineage's known march-in-place stall (slip 29.16, fwd 0.06m -- same fixed-draw pattern seen across tonight's whole campaign, not new). Training reward quarters still decline (439.9->486.2->423.2->387.2) -- milder than r1's decline (439.6->471.5->389.6->350.8) but not the registered climbing/flat pass condition. CONCLUSION: zero_drift_cmd_frame is A major contributor to the catastrophic breakdown (sacrificed leg, extreme slip) but not the SOLE cause -- bad_start_prob=0.4 (still present here, along with placement_noise=6 and joint_zero_bias=3) leaves real residual degradation. Complementary ablation cw-dep-startvar1-noBS1 (bad_start->0, zero-drift kept ON) is running concurrently and will show whether bad-start is the dominant remaining factor. Operator: cw-dep-vref1-r1 (clean PASS, no start-variation) remains the fallback base for hardware attempt #2; the startvar1 start-variation compose is not yet salvaged by this ablation alone.

