# cw-walk-posture-eff24-ds1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T05:28:20+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-walk-longdist-r2

**wandb_id**: vo5m4rs9

**hypothesis**: Dose-escalation sibling of cw-walk-posture-eff12-ds1 (operator kick 08-24: effort-pricing dose at agent judgment): if the k=1.2 effort charge is once again simply paid (its known failure mode - cw-walk-effort booked 18pct of income for 20M steps with zero behavior change), doubling to k_walk_effort=2.4 (~36pct of income at the crouch, but only ~7pct at plant pose since plant-height stance draws ~5x less hip torque) makes crouch-paddling strictly unprofitable while plant-height stepping stays cheap - the charge GAP between postures is the gradient. Identical otherwise: k_drag_stance=8000/6mm/0.25mm, warm off walk champion ppo_goal_cw_walk_longdist_r2, primitive family, control.hz=25, 40M. If-true: z off >= -10mm, rails ~0, tau p95 <=1.2, walk battery holds where eff12 possibly only half-moves. If-false (starve/park): fwd collapses - dose ladder capped, no further k iteration per RESEARCH_RULES; the pricing family is then exhausted on this lineage and the posture fix belongs to the BC-teacher / standwalk mesh-retrain route. If-false (paid even at 2.4): combined-pricing refuted decisively - same next step.

**gate**: Same as eff12 sibling: DR0 det+sto 6/6 @30s median fwd >=1.2m, 0 terminations, gait_valid 12/12, det slip/m <=1.24; probe_walk_posture_load: z off >= -10mm (parent -39.5), rail duty ~0 pct, stance hip tau p95 <=1.2 (parent 1.58), L/R asym <20pct (parent 28.1).

