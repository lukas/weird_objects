# cw-walk-joylat60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T19:43:18+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-joylat25

**wandb_id**: tzjsyiht

**hypothesis**: Driving endurance rung on the new best driving candidate: joylat25 trained/evaled at 15s episodes (~7 command changes); real joystick sessions run minutes. One variable off joylat25: episode-seconds 15->60 (same DR0.5 + latency 0.5-2.5 + abrupt 1.5s +-60% resample package) - ~40 command changes/episode. Plain: does flip-hardened driving hold up over long drives, or does gait quality decay as perturbations accumulate (wander60 held @DR0 with GENTLE resamples; abrupt+DR+latency is untested at this horizon). If-true: own-cfg @60s gv 12/12, 0 term, prog med >=0.85 with no first-half vs second-half decay, joystick gate still 0 falls - minute-scale joystick driving banked. If-false: late-episode degradation (prog/slip worsening in 2nd half, terminations late) - endurance needs its own exposure rung before demos. Strongest alternative: policy passes by parking more between commands - check stop-fraction vs joylat25.

**gate**: Own-cfg (DR0.5 + dr.latency_scale=0.5,2.5) det+sto 6/6 @60s: gait_valid 12/12, 0 term, prog med >=0.85, no det first/second-half slip decay beyond noise; JOYSTICK GATE eval_drive @DR0.2 own cfg 0 in-envelope falls; frames watched det

