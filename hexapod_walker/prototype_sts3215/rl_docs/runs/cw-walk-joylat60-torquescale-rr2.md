# cw-walk-joylat60-torquescale-rr2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T04:07:39+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-joylat60

**hypothesis**: Compose: torque-droop under load (0.80-1.05x, the CLOSED torque-droop axis's own envelope) onto joylat60's driving-endurance package (heading+-45, resample/blend jitter, latency 0.5-2.5x). Untried pairing between two independently-passed axes. If-true: own-cfg (latency+torque) det+sto 6/6 gv, JOYSTICK GATE 0 falls, DR0 flat retention clean. If-false: torque droop compounds with latency into a slow/weak-push failure the latency-only package didn't show.

**gate**: Own-cfg (dr.latency_scale=0.5,2.5 + dr.torque_scale=0.80,1.05) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; JOYSTICK GATE (eval_drive, --dr-scale 0.2) 0 in-envelope falls; DR0 flat retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**refused_reason**: hexapod-mjx-train-10 code marker c4f362547717e2fd6ac1c745ad2a3480d936dd07-dirty != local HEAD c4f362547717e2fd6ac1c745ad2a3480d936dd07. Sync first: snapshot.sh --sync hexapod-mjx-train-10 (and snapshot/commit before that if the tree is dirty).

