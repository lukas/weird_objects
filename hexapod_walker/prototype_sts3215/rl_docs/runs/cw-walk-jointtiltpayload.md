# cw-walk-jointtiltpayload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:22:57+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joytilt3

**hypothesis**: Composes chassis-payload exposure (dr.mass_scale=1.0,1.4x) onto the slope-hardened driving package joytilt3 (DR0.5 jitter+latency0.5-2.5x+ground_tilt3deg, JOYSTICK GATE PASS), distinct base from the already-running joyfric-payload (friction base) and joyhead90-payload (wide-envelope base). Tests whether the mass-DR-compose erosion pattern (seen on plain-walk payload-dr05) is structural or lineage-specific, on a THIRD distinct driving base. If-true: own-cfg gv 12/12 + DR0 nominal retention clean (slip<=1.24, prog>=0.9) AND JOYSTICK GATE 0 falls -- payload compose-safety generalizes across driving bases. If-false: DR0 retention erodes the same way -- confirms mass-DR erosion is structural, not an accident of the plain-walk lineage.

**gate**: Own-cfg harness DR0.5+latency+ground_tilt3deg+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 heading<=45deg 0 in-envelope falls; frames watched det

**verdict**: INFRA-ONLY: launch failed with 0 steps (gotcha-13b launch-collision class, W&B q2mhserd state=failed steps=None) — no science verdict on the payload-compose axis; retried as -r1 (also collided, 0 steps) then -r2 (queued for drain, c69); parent joytilt3 ckpt pushed to t11.

**failed_reason**: run never appeared as 'running' in W&B within 240s

