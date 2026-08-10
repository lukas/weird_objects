# cw-walk-joyfric-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:22:14+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-joyfric

**wandb_id**: 4yj76cyy

**hypothesis**: NEW compose, untried pairing: joystick driving (+-45deg heading, DR0.5+latency+floor-grip 0.4-1.6x, the joyfric PASS) x off-center CoM payload (0.03m). Comshift composes for free onto the plain champion and groundtilt5; payload/mass composes for free onto joyfric already (joyfric-payload PASS) -- comshift specifically has not been tried on a steering package yet at this heading width. If-true: own-cfg (DR0.5+lat+fric+comshift) det+sto 6/6 gv, 0 term, prog med matching joyfric's own band (~0.94-0.98); JOYSTICK GATE @45deg 0 falls; DR0 nominal retention prog/slip within noise of joyfric's own retention band (prog 1.00/1.00, slip 1.46/1.51). If-false: the off-axis load biases turning response and costs falls under command flips that mass-only payload did not.

**gate**: Own-cfg (DR0.5+lat+fric+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 6/6, 0 term, det prog med>=0.85; JOYSTICK GATE @45deg 0 in-envelope falls; DR0 nominal retention det 6/6 gv, prog med>=0.90, slip within noise of joyfric's own flat-retention band (slip 1.46/1.51); frames watched det

