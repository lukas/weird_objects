# cw-dep-startvar1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T05:33:58+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-dep-vref1-r1

**wandb_id**: er1yahvp

**hypothesis**: operator start-variation compose on the deployment contract, correctly re-launched (r1) now that cw-dep-vref1-r1 is verdicted PASS (no erosion under meas:=ref + 25deg tilt): placement noise 6deg + bad starts 0.4 + logical-zero-drift FRAME mode 3deg (dr.zero_drift_cmd_frame=1, probe-smoked) + k_current=0 until hardware current economics are calibrated (GPT handoff item 6). If-true: own-cfg det+sto 6/6 gv, 0 term, vel/slip within the vref1-r1 band; varied-start eval panel (placement+bad-start+zero-drift at eval time) also clean. If-false: start-variation composed onto the contract line breaks gait_valid or inflates slip -- the contract change and start-robustness interact, needs isolating which DR axis is the culprit.

**gate**: Own-cfg (contract + placement6+badstart0.4+zerodriftframe3+k_current=0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89, sto ~1.13) +-20%; varied-start eval panel (placement/bad-start/zero-drift at eval time, not just training time) clean; frames watched det

