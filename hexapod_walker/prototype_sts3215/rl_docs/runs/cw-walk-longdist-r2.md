# cw-walk-longdist-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T11:19:07+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: mga4l25v

**hardware_ready**: False

**hypothesis**: OPERATOR LAUNCH retry (r1 crashed: init-from checkpoint was missing on pod, now pushed + launcher preflight added). LONGER DISTANCE: 30 s horizons, narrow forward band, off champion 35234ddc. If-true: det median fwd distance >=1.2 m @ 30 s, gait_valid, zero terminations.

**gate**: DR0 det+sto 6/6: median fwd distance >=1.2 m @ 30 s, zero terminations, gait_valid, det slip/m <= 1.24

**verdict**: NEAR-MISS on strict gate (sto 5/6: one draw stalled at 0.62m, slip/m 6.1, no term); det 6/6 is campaign-best: median fwd 1.57m@30s (gate 1.2), det slip/m 0.96 (gate <=1.24, champ 1.24@DR1.0), prog_ratio 0.98 (champ 1.43), gait_valid 12/12, 0 terms. Frames: level six-leg alternating gait persists the full 30s, no flag leg/collapse. Still ~0.96m foot-slide per meter -> NOT hardware-ready. PROMOTED TO WALK CHAMPION (cycle 43): seed twin cw-walk-longdist-s1 reproduced everything (DR0 det slip 0.94, DR1.0 det 0.98 vs r2 1.06, champ 1.240; same single sto draw misses in both seeds -> lineage trait, not seed luck). ppo_goal_cw_walk_longdist_r2.zip md5 bcddc65c.

