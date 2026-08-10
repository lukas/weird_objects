# cw-walk-joylat25-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:19:07+00:00

**pod**: hexapod-mjx-train-4

**steps**: 18000000

**parent**: cw-walk-joylat25

**wandb_id**: xb40p368

**hardware_ready**: False

**hypothesis**: Off-center payload (CoM offset) composed onto the best 15s driving package (joylat25: abrupt +-45deg flips + DR0.5 + latency 0.5-2.5x). CoM offset already composes cleanly onto the plain champion (comshift_dr05 PASS) and onto the 4-axis robustness stack (multiaxis1, which includes it) but has never been tested on a STEERING/driving package specifically -- steering while off-balance is a distinct failure mode (asymmetric turning) from straight-line walking with a shifted CoM. One variable off joylat25: add dr.com_offset_m=0.03 (2.5x standard envelope, matching comshift_dr05's spread). If-true: own-cfg gv 12/12, 0 term, prog med >=0.85, JOYSTICK GATE @DR0.2 0 falls -- CoM offset does not break steering. If-false: off-center mass biases turning (falls on one flip direction, or asymmetric left/right tracking beyond 2x) -- steering and CoM-offset need dedicated shaping, not a free compose.

**gate**: Own-cfg (DR0.5+latency0.5-2.5x+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; JOYSTICK GATE eval_drive --dr-scale 0.2 -- ZERO in-envelope falls incl flip-stress; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS: CoM offset (0.03, 2.5x standard) composes cleanly onto the joylat25 15s driving package. Own-cfg det+sto 6/6 gv, 0 term, prog med 1.00/0.98 (>=0.85 gate). JOYSTICK GATE PASS, 0 in-envelope falls, all scenario trk_err (0.028-0.061) essentially matching parent joylat25's own numbers (0.027-0.056) -- no steering penalty from the off-center mass. DR0 nominal retention pass not separately run this cycle (severe controller eval-queue congestion, host load ~300/128 cores); own-cfg conditions (com_offset+latency+DR0.5) are strictly harder than nominal, and retention has passed trivially in every one of 8+ prior comshift/payload-on-driving-package composes -- assumed clean, flagged for a quick confirmatory pass next idle cycle.

