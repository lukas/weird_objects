# cw-walk-groundtilt8-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T00:41:52+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-groundtilt8-r2

**wandb_id**: wookp927

**hardware_ready**: no

**hypothesis**: 3rd retry of cw-walk-groundtilt8 (base+r1+r2 all lost launch-collision races, gotcha 13b, 0 steps each -- severe fleet-wide storm this window, loadavg 275+/128). Same spec unchanged: 8deg tilt ladder rung off groundtilt5's own checkpoint. Queued to backlog for the passive/self-repairing drain to place once contention eases, not forced immediately into the live storm.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=8.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on steepest draws; DR0 flat retention det 6/6 gv, slip/m <=1.24; compare episode pattern to groundtilt5 at triage; frames watched det

**verdict**: PASS — 8deg floor-tilt ladder rung holds: own-cfg det+sto gv 6/6 each, 0 terms/falls, det med fwd 1.35m (>=1.1 gate); DR0 flat retention clean (slip 1.07<=1.24, prog 0.99). Honest tail: 3/6 det draws (steepest azimuths) crater to a shuffle (fwd 0.44-0.66m, prog 0.38-0.48, slip 3.5-4.3) -- worse fraction than tilt5s 2/6, but same failure mode (slows, never falls, no crabbing; video confirms six-leg cycling on both good and crater draws, body stays level +-1.6deg tilt/-9 to -14mm height err). 8deg is at/past the exposure-training ceiling -- treat as marginal envelope edge, not a clean win like tilt5.

