# cw-walk-yawgate1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T15:26:26+00:00

**pod**: hexapod-mjx-train-1

**steps**: 12000000

**parent**: cw-walk-yawcmd1-rr1

**wandb_id**: nrp989dr

**hypothesis**: Queue-0 named next arm (yaw income gating). yawcmd1 failed its yaw gate with walk_yaw_err FLAT across both seeds - the pre-registered if-false: the Gaussian yaw kernel pays heading-hold income at wz_ref~0 without requiring achieved tracking, so turning never has to emerge (same exploit class the linear kernel had before walk_kernel_prog_gate). One variable vs yawcmd1-rr1: reward.walk_yaw_kernel_gate=1.0 gates yaw income on achieved |wz| (smoke_yaw_quad covers the gated path). Prediction-if-true: turn-segment |wz_err| med drops from 0.24 toward <=0.10 rad/s and right turns become tracked, with JOYSTICK GATE + forward band retained. Prediction-if-false: |wz_err| stays ~0.24 - free income was not the binding exploit; escalate to a dedicated turn-in-place curriculum arm. Strongest alternative: turning emerges but linear driving erodes (kernel competition) - then ladder k_walk_yaw down, not off.

**gate**: eval_yaw.py own-cfg: commanded-turn |wz_err| med <=0.10 rad/s AND wz_ref=0 |wz| med <=0.05 AND JOYSTICK GATE @DR0.2 0 in-envelope falls AND forward det prog within joyjit-dr05-c1 band, slip <=1.25; frames watched det

