# cw-dep-startvar1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T05:33:58+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-dep-vref1-r1

**wandb_id**: er1yahvp

**hardware_ready**: False

**hypothesis**: operator start-variation compose on the deployment contract, correctly re-launched (r1) now that cw-dep-vref1-r1 is verdicted PASS (no erosion under meas:=ref + 25deg tilt): placement noise 6deg + bad starts 0.4 + logical-zero-drift FRAME mode 3deg (dr.zero_drift_cmd_frame=1, probe-smoked) + k_current=0 until hardware current economics are calibrated (GPT handoff item 6). If-true: own-cfg det+sto 6/6 gv, 0 term, vel/slip within the vref1-r1 band; varied-start eval panel (placement+bad-start+zero-drift at eval time) also clean. If-false: start-variation composed onto the contract line breaks gait_valid or inflates slip -- the contract change and start-robustness interact, needs isolating which DR axis is the culprit.

**gate**: Own-cfg (contract + placement6+badstart0.4+zerodriftframe3+k_current=0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89, sto ~1.13) +-20%; varied-start eval panel (placement/bad-start/zero-drift at eval time, not just training time) clean; frames watched det

**verdict**: FAIL -- URGENT P0 finding, do NOT warm-start hardware attempt #2 from this checkpoint. Own-cfg (its own trained varied-start config) gate is broken: det 1/6 pass, ALL 6 det episodes degraded (fwd 0.23-0.65m; expect ~0.8m ideal @15s/0.055m/s), one episode (ep3) has a REAL sacrificed leg [1] (gait_valid False -- disqualifying per the gait-validity gate regardless of vel error), one episode (ep4) slip/m=21.99 (catastrophic near-total-skate). sto milder but still degraded (4/6, prog 0.79, slip 1.83 vs vref1-r1's own sto band ~1.13). Training itself shows reward quarters DECLINING (439.6->471.5->389.6->350.8) -- unlike every other compose tonight, which climbs or plateaus. Seed twin cw-dep-startvar1-s1 reproduces the same qualitative breakdown (0/6 det, prog 0.52, slip 6.66; no flag-leg this draw but same systemic degradation) -- not seed luck, a real defect in the compose. Matches the pre-registered if-false exactly: "the contract change and start-robustness interact ... needs isolating which DR axis is the culprit." Frames confirm mechanically: ep3's flag leg is visible (one front leg extended/non-cycling across the strip), other episodes show erratic, non-progressing gait. Root-cause hypothesis (untested, prime suspect): dr.zero_drift_cmd_frame=1 is a brand-new mechanism only probe-smoked in isolation, never trained at 18M-step scale, and silently offsets the physical joint frame the policy needs for foot placement -- placement-noise and bad-start are independently proven benign elsewhere (placementnoise6-r3 PASS; payload/comshift composes routinely absorb similar exposure). Isolation ablation queued+running: cw-dep-startvar1-noZD1 (same recipe, zero_drift_cmd_frame->0). Operator: fall back to cw-dep-vref1-r1 (PASSed, no start-variation) as the hardware-attempt-#2 base until this is resolved.

