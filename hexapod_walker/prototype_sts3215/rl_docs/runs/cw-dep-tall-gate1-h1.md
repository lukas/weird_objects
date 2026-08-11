# cw-dep-tall-gate1-h1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-11T21:39:40+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-dep-tall-gate1

**wandb_id**: o7soakk5

**hardware_ready**: False

**hypothesis**: TALL LADDER T3-h1: gated recipe at the budget that broke the ungated one. T1 proved 6M finds a 0.051 m/s gait but pays for it with a -67mm crouch when height is only a ref; the gate (sigma 30, reachable ref -30) prices that trade at ~0.5x income at 40mm err. If the gate works, the optimizer must find speed WITHIN the posture envelope: expect height_err_end <=15mm held AND speed climbing toward 0.04+.

**gate**: JACKPOT: height_err_end <=8mm AND speed >=0.040 -> immediate Gate 0 export + tipped retention, hardware candidate. PASS: err <=15mm held AND speed >=0.035. INFORMATIVE FAIL: err grows >25mm = gate loses to walk income under budget even at sigma 30 reachable ref -> the income structure cannot buy tall+fast together; escalate to T4 (shrink the speed band) or accept the pareto pair (tall30 for posture / tall15-h1 for speed).

**verdict**: INFORMATIVE FAIL (per pre-registered gate). height_err_end grew 15mm(2M)->36-47mm(6M det, DR0/0.35) despite speed staying 0.054-0.056 m/s (above the speed bar) -- the sigma-30 income gate SLOWS but does not STOP the posture-for-speed drift under a 6M budget. Direct confirmation via probe_tall_wall.py on this checkpoint: steady-state walking height -72.6mm, essentially unchanged from parent tall30's own -75mm wall, and leg-yaw limit margin still negative (-0.5 to -0.3 deg, i.e. pinned at the 35 deg splay limit) -- same stability-purchase signature as T5, gate income did not move it. Video clean six-leg gait, no flag-leg/park cheat (gait_valid 5/6 det, 6/6 own-DR det). Per plan: escalate to T2/T4 (already launched this cycle) or accept the tall30/tall15-h1 pareto pair; do not schedule further gate-sigma variants at this dose.

