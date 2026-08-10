# cw-walk-joyheadtilt3-payload-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T02:49:06+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-joyheadtilt3-payload

**wandb_id**: 6xvf2i9x

**hardware_ready**: False

**hypothesis**: Mechanical retry of cw-walk-joyheadtilt3-payload: 2 prior attempts lost to infra (1 vanished from backlog with no trace, 1 died 0-step EOFError launch-collision). Same hypothesis unchanged: payload compose onto joyheadtilt3.

**gate**: Own-cfg (DR0.5+lat+dr.ground_tilt_deg=3.0+dr.mass_scale=1.0,1.4) det+sto 6/6 @15s: gait_valid 6/6, 0 term, det prog med>=0.80; JOYSTICK GATE @90deg 0 in-envelope falls; DR0 TRUE FLAT no-payload retention det 6/6 gv, prog med>=0.85, slip within noise of joyheadtilt3's own flat-retention band (1.51/1.70); frames watched det

**verdict**: Compose PASS: 90deg head-turn driving on 3deg ground-tilt x payload (1.0-1.4x mass). Own-cfg (dr-scale 0.5, correct rate) det+sto 12/12 gait_valid, 0 term, det prog med 0.86 (>=0.80 gate); DR0 TRUE FLAT no-payload retention det 6/6 gv prog med 0.89, slip 1.55 matching joyheadtilt3's own flat band (1.51/1.70). JOYSTICK GATE PASS @90deg, 0 in-envelope falls incl flip-stress. Frames: real six-leg cycling gait on worst draws, no flag leg.

