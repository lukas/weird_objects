# cw-walk-strafe-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T14:27:21+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-strafe

**wandb_id**: andpllw9

**hypothesis**: Robustness rung for the PASSed lateral base (cw-walk-strafe: ±90° transport, prog 0.81-1.21 all 12, gv 12/12 at DR0). One variable off the strafe checkpoint: train at model-DR 0.5 instead of --no-dr (same recipe that produced dr05-r1 from the fwd champion). Feeds the driving demo on real hardware. If-true: DR0.5 own-eval keeps lateral transport (prog med >=0.8, gv 12/12, 0 term) with slip/m med <= 2.4 (DR0 value 2.0 + noise band). If-false: the lateral paddle collapses under physics uncertainty (prog craters or flag legs appear) — lateral robustness needs its own shaping, not just DR exposure.

**gate**: own-cfg DR0.5 6+6: gv 12/12, 0 term, prog_ratio med >=0.8, slip/m med <=2.4; DR0 det retention prog med >=0.8 gv 6/6; frames watched det

