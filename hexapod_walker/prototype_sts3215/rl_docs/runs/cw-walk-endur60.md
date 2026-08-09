# cw-walk-endur60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T13:00:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: ml3y8z72

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST: long-distance endurance. 60 s episodes (4x champion horizon), narrow band. If-true: det median fwd distance >=2.4 m @ 60 s with gait_valid and no late-episode degradation; if-false: gait decays over time (fatigue-like drift in policy state or accumulating pose error).

**gate**: DR0 det+sto 6/6: median fwd distance >=2.4 m @ 60 s, zero terminations, gait_valid

**verdict**: PASS. 60s endurance (4x horizon, narrow band, off anchorgate): DR0 det median fwd 3.17m (gate >=2.4), gv 12/12, 0 term, det slip/m 0.84-0.94, prog ~1.0 — NO late-episode degradation; frames: level six-leg cycling for the full 60s. One sto draw-stall ep (prog 0.29, still 1.13m) = known lineage trait. Informational DR1.0 (trained DR0): det 4/6 sto 2/6, gv all, no falls. Endurance question answered: gait does not decay with horizon; seed twin endur60-s1 confirms/denies concordance. hardware-ready: no (lineage stalls + contact pricing).

