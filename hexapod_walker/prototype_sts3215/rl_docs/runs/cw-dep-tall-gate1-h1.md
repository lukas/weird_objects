# cw-dep-tall-gate1-h1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T21:39:40+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-dep-tall-gate1

**wandb_id**: o7soakk5

**hypothesis**: TALL LADDER T3-h1: gated recipe at the budget that broke the ungated one. T1 proved 6M finds a 0.051 m/s gait but pays for it with a -67mm crouch when height is only a ref; the gate (sigma 30, reachable ref -30) prices that trade at ~0.5x income at 40mm err. If the gate works, the optimizer must find speed WITHIN the posture envelope: expect height_err_end <=15mm held AND speed climbing toward 0.04+.

**gate**: JACKPOT: height_err_end <=8mm AND speed >=0.040 -> immediate Gate 0 export + tipped retention, hardware candidate. PASS: err <=15mm held AND speed >=0.035. INFORMATIVE FAIL: err grows >25mm = gate loses to walk income under budget even at sigma 30 reachable ref -> the income structure cannot buy tall+fast together; escalate to T4 (shrink the speed band) or accept the pareto pair (tall30 for posture / tall15-h1 for speed).

