# cw-walk-lowgait-dr035-latjit

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:14:21+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035

**hypothesis**: Composes bus-latency jitter (dr.latency_scale=0.5,2.5x) onto the crouch-DR line cw-walk-lowgait-dr035 (-50mm height ref, survives generic DR up to 0.35). The crouch line has only been tested against generic DR (mass/friction/etc together); this isolates a single, driving-relevant axis on top of it, mirroring the successful axis-onto-DR05-compose pattern already validated on the flat-walk line (latjit-dr05 PASS, fricvar-dr05 PASS). If-true: own-cfg gv 12/12, 0 term, mean end-height err <=10mm, slip/m <=1.6 (crouch DR0.35 gate) retained WITH latency added, AND DR0-no-latency-no-DR nominal retention clean (height err <=8mm, slip/m<=1.15) -- crouch tolerance and latency tolerance compose without interference, like every other successful compose this campaign. If-false: latency on top of a lowered stance destabilizes height tracking (short leg travel + delayed commands overshoot) -- crouch-height composes are latency-sensitive, unlike flat-walk composes.

**gate**: Own-cfg harness DR0.35 (lowgait-dr035 cfg) + dr.latency_scale=0.5,2.5 det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m<=1.6; DR0 nominal (no latency, no DR) retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det for lurching/height overshoot

**verdict**: Launch failure (gotcha 13b EOFError collision storm), 0 steps, no science result.

**failed_reason**: run never appeared as 'running' in W&B within 240s

