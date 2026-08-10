# cw-walk-joylat60-payload-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-10T02:28:46+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-joylat60-payload

**hypothesis**: Clean retry of the orphaned cw-walk-joylat60-payload spec (INTENT, never launched -- an earlier drain call timed out mid-launch and the item vanished from backlog with 0 science, my mistake). Same one-variable design unchanged: composes chassis-payload exposure (dr.mass_scale=1.0,1.4x) onto the 60s abrupt-flip+DR0.5+latency driving ENDURANCE package (joylat60, JOYSTICK GATE PASS, no decay). If-true: own-cfg det+sto 6/6 @60s gv 12/12, 0 term, prog med>=0.75, no first/second-half decay; DR0 retention gv 6/6 slip<=1.24 prog>=0.9; JOYSTICK GATE 0 falls. If-false: payload compounds over 60s into late-episode decay or nominal erosion the 15s packages (joyfric-payload PASS) didn't show.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.latency_scale=0.5,2.5 + dr.mass_scale=1.0,1.4, det+sto 6/6 @60s: gait_valid 12/12, 0 term, det prog median >=0.75, no first/second-half decay; DR0 nominal (no payload) retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 0 in-envelope falls retained; frames watched det

**verdict**: Killed immediately, no science lost (<1M steps). Pure duplicate: the original cw-walk-joylat60-payload (same config, same seed) got mechanically relaunched moments before my respec landed this identical spec on train-1 -- a race, not a real second variable. cw-walk-joylat60-payload (train-0) is the one to triage when it finishes; this entry is not evidence of anything.

