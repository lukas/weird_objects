# cw-walk-joylat60-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T02:24:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joylat60

**wandb_id**: qp6io4l6

**hypothesis**: Composes chassis-payload exposure (dr.mass_scale=1.0,1.4x) onto the 60s abrupt-flip+DR0.5+latency driving ENDURANCE package joylat60 (JOYSTICK GATE PASS, no first/second-half decay). Payload already composes safely onto 15s driving packages (joyfric-payload PASS c70) but never onto a 60s ENDURANCE horizon. If-true: own-cfg det+sto 6/6 @60s gv 12/12, 0 term, prog med>=0.75, no decay; DR0 retention gv 6/6 slip<=1.24 prog>=0.9; JOYSTICK GATE 0 falls. If-false: payload compounds over 60s into late-episode decay or nominal erosion the 15s packages didn't show. (Requeue: an identical earlier queue attempt this cycle was orphaned by my own timeout-truncated drain call, popped from backlog.json but never launched -- 0 science, no trace, my mistake -- this is the same spec, clean retry.)

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.latency_scale=0.5,2.5 + dr.mass_scale=1.0,1.4, det+sto 6/6 @60s: gait_valid 12/12, 0 term, det prog median >=0.75, no first/second-half decay; DR0 nominal (no payload) retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 0 in-envelope falls retained; frames watched det

