# cw-walk-joyfric-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T23:24:39+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-joyfric

**wandb_id**: bi4oiwsa

**hypothesis**: Composes chassis-payload exposure (dr.mass_scale=1.0,1.4x) onto the floor-hardened driving package joyfric (DR0.5 jitter+latency+friction, JOYSTICK GATE PASS). Payload-on-WALK-champion (payload-dr05) FAILED nominal retention (DR training on mass charged nominal tracking: slip 1.38>1.24, prog 0.54 vs 0.95) -- tests whether that erosion pattern is structural to mass-DR composes generally, or specific to the plain-walk lineage. If-true: own-cfg gv 12/12 + DR0 nominal retention clean (slip<=1.24, prog>=0.9, no erosion) AND JOYSTICK GATE still 0 falls -- payload is compose-safe on the driving line, unlike on plain walk. If-false: DR0 nominal retention erodes the same way as payload-dr05 -- the erosion is a structural property of mass-DR training, not an accident of one lineage, closing the mass-DR-compose sub-class generally.

**gate**: Own-cfg harness DR0.5+latency+friction+dr.mass_scale=1.0,1.4 det+sto 6/6 @30s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 0 in-envelope falls retained; frames watched det

