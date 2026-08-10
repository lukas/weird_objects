# cw-walk-joyquad15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T06:27:50+00:00

**pod**: hexapod-mjx-train-3

**steps**: 12000000

**parent**: cw-walk-joyquad30

**wandb_id**: sfqdrvio

**hardware_ready**: False

**hypothesis**: Quad-mix dose-response (P0 ruling 7: map the frontier, not abandon the skill). joyquad30 (30% mix on driving champion joylat25) FAILED the compound gate's walk-retention leg: own-cfg det/sto slip/m med 1.72 vs cap 1.55 (parent joylat25 band 1.48/1.51), while quad-hold itself was excellent (eval/quad survived_frac 1.0, height_err 2-9mm, clean video). quad-hold1-r2's 50% mix on the plain walk champion also eroded (slip 1.42 vs cap 1.25). One variable off joyquad30: halve the mix to 15%. If-true: own-cfg det/sto slip/m med <=1.55 (matches joylat25 band) AND quad-hold still solid (eval/quad survived_frac>=0.9, height_err_end<=15mm) -- erosion scales down with dose, a low-dose quad button is viable. If-false: even 15% still erodes walk slip meaningfully -- erosion is not simply proportional to mix fraction and quad needs to be trained as a separate skill blended only transiently at deploy time (specialist head / KL-anchor to frozen champion), not baked into the training mix at all.

**gate**: Own-cfg (DR0.5+latency, matching joylat25's own gate config) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m med<=1.55 (joylat25 band); training's own eval/quad telemetry survived_frac>=0.9 with height_err_end<=15mm across checkpoints; JOYSTICK GATE @DR0.2 retained (0 in-envelope falls); frames watched det for flag-leg/dragging AND quad video for clean lift/no tipping

**verdict**: FAIL on the walk-retention leg of the compound gate (if-false confirmed: erosion is NOT proportional to mix fraction). Own-cfg (DR0.5+latency, matching joylat25 own gate) harness: det slip/m med 1.64, sto slip/m med 1.70 -- both above the pre-registered <=1.55 cap and joylat25 own band (1.48/1.51); halving the mix from 30% (joyquad30: det/sto 1.72/1.72) to 15% only bought a ~5% slip reduction, nowhere near the cap. Progress ratio fine (det med 0.89, sto med 0.95, >=0.85 gate) and gait_valid 12/12, 0 term -- video (det+sto frame strips) shows clean six-leg cycling, level torso, no flag-leg/dragging; the slip inflation is a numbers-only degradation like the rest of this lineage, invisible on casual video. Quad-hold mechanism itself stays solid throughout training: eval/quad/survived_frac 1.0 at every checkpoint, height_err_end_mm 2-14mm (<=15mm gate), track_err<1.1deg. 4th data point for P0 ruling 7: quad-mix erosion of walk-mode slip does not scale down proportionally with dose -- halving the mix is not a viable lever; JOYSTICK GATE not run since the compound gate already fails unambiguously on the pre-registered walk leg. Next lever per the pre-registered if-false: an explicit anti-slip/economy term active specifically during quad-mix episodes, or train quad as a transient specialist blended only at deploy time, not baked into the training mix ratio.

