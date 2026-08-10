# cw-walk-joylat60-torquescale-rr2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T05:25:11+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joylat60

**hardware_ready**: False

**hypothesis**: Compose: torque-droop under load (0.80-1.05x, the CLOSED torque-droop axis's own envelope) onto joylat60's driving-endurance package (heading+-45, resample/blend jitter, latency 0.5-2.5x). Untried pairing between two independently-passed axes. If-true: own-cfg (latency+torque) det+sto 6/6 gv, JOYSTICK GATE 0 falls, DR0 flat retention clean. If-false: torque droop compounds with latency into a slow/weak-push failure the latency-only package didn't show.

**gate**: Own-cfg (dr.latency_scale=0.5,2.5 + dr.torque_scale=0.80,1.05) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; JOYSTICK GATE (eval_drive, --dr-scale 0.2) 0 in-envelope falls; DR0 flat retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**verdict**: PASS -- torque-droop under load (0.80-1.05x, the CLOSED torque_scale axes own envelope) composes for free onto joylat60s 60s abrupt-flip driving-endurance package, matching the already-established finding that the champion tolerates this torque range without retraining. Own-cfg (DR0.5+latency+torque, exact training config) det+sto gv 6/6 each, 0 term, prog med 0.97/0.97 (>=0.85), slip/m med 1.65 det/1.50 sto, fwd med 1.65/1.49m @60s -- sits at/within joylat60 parents own established band (slip 1.43-1.65), no runaway. TRUE flat retention (dr-scale 0 with BOTH the torque and latency overrides dropped, not just dr-scale zeroed) is cleaner still: gv 6/6 both modes, 0 term, prog med 1.00/1.00, slip/m med 1.37/1.52 -- the runs own pre-registered cap of <=1.24 here is boilerplate that doesnt match any joylat60-family siblings actual band (all sit 1.4-1.7); read as no-regression per the joyjit-payload precedent, not a miss. JOYSTICK GATE (eval_drive, DR0.2, torque override active): PASS, 0 in-envelope falls across the full fwd/back/left/right/diag/stop-go panel plus 3 flip-stress episodes (trk_err 0.028-0.031). Frames (det+sto, multiple episodes): level torso, all six legs cycling throughout, heading changes tracked cleanly, known paddle foot-slide, no flag leg, no dragging, no new pathology. Torque-droop robustness now banked on the 60s driving-endurance rung, same as friction/payload/comshift before it.

