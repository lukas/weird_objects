# cw-walk-joylat60-torquescale-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T03:48:44+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-joylat60

**hypothesis**: Compose: torque-droop (0.80-1.05x) onto joylat60 driving package. 2nd requeue -- first 2 attempts (cw-walk-joylat60-torquescale, twice) vanished/died with no ledger trace under the fleet's severe concurrent-drain launch-collision storm this window, 0 compute lost either time.

**gate**: Own-cfg (dr.latency_scale=0.5,2.5 + dr.torque_scale=0.80,1.05) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; JOYSTICK GATE (eval_drive, --dr-scale 0.2) 0 in-envelope falls; DR0 flat retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

