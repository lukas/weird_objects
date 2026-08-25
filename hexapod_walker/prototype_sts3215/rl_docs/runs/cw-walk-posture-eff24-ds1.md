# cw-walk-posture-eff24-ds1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T05:28:20+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-walk-longdist-r2

**wandb_id**: vo5m4rs9

**hypothesis**: Dose-escalation sibling of cw-walk-posture-eff12-ds1 (operator kick 08-24: effort-pricing dose at agent judgment): if the k=1.2 effort charge is once again simply paid (its known failure mode - cw-walk-effort booked 18pct of income for 20M steps with zero behavior change), doubling to k_walk_effort=2.4 (~36pct of income at the crouch, but only ~7pct at plant pose since plant-height stance draws ~5x less hip torque) makes crouch-paddling strictly unprofitable while plant-height stepping stays cheap - the charge GAP between postures is the gradient. Identical otherwise: k_drag_stance=8000/6mm/0.25mm, warm off walk champion ppo_goal_cw_walk_longdist_r2, primitive family, control.hz=25, 40M. If-true: z off >= -10mm, rails ~0, tau p95 <=1.2, walk battery holds where eff12 possibly only half-moves. If-false (starve/park): fwd collapses - dose ladder capped, no further k iteration per RESEARCH_RULES; the pricing family is then exhausted on this lineage and the posture fix belongs to the BC-teacher / standwalk mesh-retrain route. If-false (paid even at 2.4): combined-pricing refuted decisively - same next step.

**gate**: Same as eff12 sibling: DR0 det+sto 6/6 @30s median fwd >=1.2m, 0 terminations, gait_valid 12/12, det slip/m <=1.24; probe_walk_posture_load: z off >= -10mm (parent -39.5), rail duty ~0 pct, stance hip tau p95 <=1.2 (parent 1.58), L/R asym <20pct (parent 28.1).

**verdict**: FAIL on the experiment's own posture hypothesis (though the plain walk battery half of the gate PASSES). Plain English: doubling the effort-pricing dose vs its eff12-ds1 sibling was meant to push the champion out of its sprawled crouch harder; instead the crouch got DEEPER and safety got WORSE. probe_walk_posture_load (seed0, 20s@0.055m/s primitive): body z-offset -56.5mm (baseline longdist_r2 -39.5, target >=-10mm) -- WORSE than baseline, not better; hip clamp-rail duty 0.0001 (baseline 0.0023, target ~0) -- charge paid; L/R stance-hip asym 27.3% (baseline 28.1%, target <20%) -- essentially unchanged. So the priced quantities (rail events, mean torque) DID drop, but the policy's way of cutting torque was to crouch even lower, not to stand taller -- the reverse of the intended fix (lower crouch apparently needs less hip torque at this leg geometry, so effort pricing alone pushes the wrong direction on posture). Walk battery itself is clean: DR-0 gate 24/24 gait_valid, 0 terminations, fwd_med 1.27-1.4m (>=1.2 target), det slip_med 0.92 (<=1.24 target) -- so the underlying locomotion competence held up fine, this is purely a posture-fix failure. Safety got WORSE at 2x dose: held-out 60s joygate 12/24 falls (vs eff12-ds1 sibling's 2/24 at half the dose -- monotonically worse with more pricing pressure), interactive session hard-gate FAIL (over_current on right/sit/back segments). Verdict: combined effort+drag+rail pricing is REFUTED for fixing posture on this lineage at both tested doses (1.2 and 2.4) -- per this experiment's own pre-registered fallback, no further k-iteration on this lever; the posture fix belongs to the BC-teacher/standwalk mesh-retrain route. Evidence: /tmp/probe_eff24_ds1_full.txt, logs/ckpt_eval/cw_walk_posture_eff24_ds1_{gate,joygate,session}/, W&B vo5m4rs9.

