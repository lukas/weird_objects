# cw-walk-wander-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T14:29:39+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-wander

**wandb_id**: 9x8kcuuo

**hardware_ready**: no

**hypothesis**: Robustness rung for the PASSed driving base (cw-walk-wander: ±45°/5s resample/15% stops, gv 12/12 at DR0, c45). One variable off the wander checkpoint: train at model-DR 0.5 instead of --no-dr. Direction-change transitions are where the paddle gait is weakest (change-eps slip ~2x straight) — DR exposure tests whether transition handling survives physics uncertainty. If-true: DR0.5 own-eval holds gv 12/12, 0 term, prog med ~1.0 through command changes with change-ep slip no worse than the DR0 value (~2.1) + noise. If-false: transitions break under DR (stalls/parks at command changes) — the driving demo needs transition-specific work before hardware.

**gate**: own-cfg DR0.5 6+6 resampled commands: gv 12/12, 0 term, prog_ratio med >=0.85, slip/m med <=2.4; DR0 det retention gv 6/6; frames watched det

**verdict**: PASS. DR0.5 training kept driving transitions under physics uncertainty: own-cfg DR0.5 det+sto (±45° resample/5s, 15% stops) gv 12/12, 0 term, prog med 0.96/0.95 (gate ≥0.85), slip/m med 1.39/1.70 (gate ≤2.4, no worse than DR0 change-ep ~2.1); DR0 det retention gv 6/6, prog 1.01, slip 1.52. Frames det: level six-leg cycling through command changes, no flag leg, no stalls. Hardware-ready: no (contact-pricing slip root).

