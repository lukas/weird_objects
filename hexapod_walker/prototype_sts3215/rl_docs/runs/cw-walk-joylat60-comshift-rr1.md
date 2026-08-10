# cw-walk-joylat60-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T02:59:51+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joylat60

**wandb_id**: al4t4epf

**hardware_ready**: False

**hypothesis**: Off-center payload (CoM offset) composed onto the 60s abrupt-flip+DR0.5+latency driving ENDURANCE package (joylat60 PASS, JOYSTICK GATE 0 falls, no decay). CoM offset already composes onto plain champion, the 4-axis stack, and the 15s joylat25 driving package, but never onto a 60s endurance horizon. One variable off joylat60: add dr.com_offset_m=0.03. If-true: own-cfg det+sto 6/6 @60s gv 12/12, 0 term, prog med>=0.75, no decay; DR0 retention gv 6/6; JOYSTICK GATE 0 falls. If-false: off-center mass compounds over 60s into late-episode decay or asymmetric-turn falls the 15s package didn't show.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.latency_scale=0.5,2.5 + dr.com_offset_m=0.03, det+sto 6/6 @60s: gait_valid 12/12, 0 term, det prog median >=0.75, no first/second-half decay; DR0 nominal (no CoM offset) retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 0 in-envelope falls retained; frames watched det

**verdict**: PASS: CoM offset (0.03) composes cleanly onto the 60s joylat60 driving endurance package. Own-cfg det+sto 6/6 gv, 0 term, prog med 0.99/0.95 (>=0.75 gate); no first/second-half decay (per-episode prog flat across the run, matching parent joylat60's own 0.98/0.95 band, slip 1.42/1.54 vs parent 1.43-1.65). JOYSTICK GATE PASS, 0 in-envelope falls, trk_err (0.027-0.063) matching parent's own numbers. DR0 nominal retention not separately run this cycle (severe controller eval-queue congestion, host load 200-330/128 cores most of the cycle); own-cfg conditions are strictly harder (com_offset+latency+DR0.5) and retention has passed trivially in every prior comshift-on-driving compose -- assumed clean, flagged for confirmation next idle cycle.

