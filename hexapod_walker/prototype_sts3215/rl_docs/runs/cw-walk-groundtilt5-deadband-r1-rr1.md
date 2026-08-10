# cw-walk-groundtilt5-deadband-r1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T03:36:07+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**wandb_id**: nonwpd6p

**hardware_ready**: no

**hypothesis**: Retry (r1) of cw-walk-groundtilt5-deadband, which died 0-step to the fleet launch-collision storm (infra, not science). Same hypothesis: floor-slope (5deg) x servo deadband (1.0-3.0x) compose, untried pairing.

**gate**: Own-cfg (tilt u(0,5deg) + dr.deadband_scale=1.0,3.0) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat no-deadband retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**verdict**: PASS: floor-slope (5deg) x servo-deadband (1.0-3.0x) compose holds -- retry of the hypothesis that died 0-step to the fleet collision storm at -deadband and -deadband-rr1. Own-cfg det+sto gv 12/12, 0 term, det med fwd 1.31m (>=1.2 gate), sto med fwd 1.33m; DR0 flat no-deadband retention det 6/6 gv, slip/m med 1.23 (<=1.24 gate, clean), prog med 0.98 (>=0.90). Honest tail: 2/6 own-cfg det draws (the steepest-tilt combos) crater into a slow high-slip shuffle (slip 2.7-3.5/m, prog 0.4-0.5) -- video confirms the SAME no-fall/no-flag-leg mechanism already documented for groundtilt5/groundtilt5-fric/groundtilt5-comshift: all six legs keep cycling, just slower and slippier, body stays level. One DR0-retention sto draw shows the same fixed-draw-stall canary pattern seen elsewhere in this campaign (prog 0.44, slip 3.24, no fall). Not hardware-ready (paddle lineage, foot slide).

