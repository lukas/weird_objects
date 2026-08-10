# cw-walk-joyheadfric-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:04:35+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joyheadfric

**wandb_id**: 6b0xevjm

**hypothesis**: Composes chassis-payload exposure (dr.mass_scale=1.0,1.4x) onto the widest floor-hardened driving package joyheadfric (+-90deg envelope, DR0.5 latency+friction, JOYSTICK GATE PASS @90deg, seed-confirmed by joyheadfric-s1r1). joyfric-payload (this cycle's 45deg-envelope sibling) already tests whether payload composes safely on a driving line vs eroding like plain-walk payload-dr05; this run asks whether that answer holds at the WIDER +-90deg envelope specifically, since wider steering already costs more foot-slide (1.7-1.9 vs 1.3-1.7 slip/m) even without payload. If-true: own-cfg gv 12/12 + DR0 nominal retention clean (slip<=1.24, prog>=0.85, matching joyheadfric's own band) AND JOYSTICK GATE @90deg still 0 falls -- payload is compose-safe at the wide envelope too. If-false: DR0 retention erodes OR falls appear at 90deg under payload -- the wider envelope is payload-fragile even though the 45deg package (joyfric-payload) wasn't, meaning envelope width and payload interact (a genuinely new finding, not just repeating joyfric-payload).

**gate**: Own-cfg harness DR0.5+latency+friction+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.85 (joyheadfric's own band); JOYSTICK GATE @DR0.2 --heading-max-deg 90 0 in-envelope falls retained; frames watched det

**failed_reason**: W&B global_step not advancing (0 -> 0)

