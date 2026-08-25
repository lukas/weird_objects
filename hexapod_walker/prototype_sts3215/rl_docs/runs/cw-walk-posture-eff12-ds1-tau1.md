# cw-walk-posture-eff12-ds1-tau1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T05:31:32+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 5r5vkyh3

**hypothesis**: Plain English: make the walking champion stand tall and stop wasting hip torque by making effort itself expensive -- this is the operator-AMENDED version of cw-walk-posture-eff12-ds1 (running un-hinged sibling = matched control), single added lever: a per-motor torque hinge, r -= 2.0 * mean(relu(|qfrc_actuator| - 1.0 N*m)) per walk tick (reward.k_tau_over=2.0, reward.tau_over_nm=1.0, walk-routed like k_walk_effort). Operator load-probe truth: honest plant-height stepping needs at most ~1 N*m per hip (0.23 static on six feet, ~0.5 in tripod, ~1 at dynamic peaks) while the skater crouch runs 1.1-1.5 N*m STATIC and rails the 2.2 N*m clamp -- so a 1.0 N*m hinge prices ONLY the waste and the rail events, never normal stepping. Motor heat is I^2*R: sustained above-threshold torque matters superlinearly, and the mean-current k_walk_effort=1.2 (inherited, still on) underprices peaks; before the hinge nothing existed between free and the over_current episode trip. Sim current model is |tau|*1.2 A/Nm lowpassed, so hinging on raw torque is equivalent and clearer (and sees the peaks the 0.1 s LPF smears). Bank-measured separation (k=1, 3 s): honest gait -0.24, static stance 0.00, railed fight -43.1. Everything else identical to eff12-ds1: warm off ppo_goal_cw_walk_longdist_r2, k_walk_effort=1.2 + k_drag_stance=8000/6mm/0.25mm, primitive family, control.hz=25 legacy pins, 40M. If-true: posture rises toward plant faster/further than the un-hinged control and rails go to ~0. If-false (no delta vs control): the hinge is redundant given effort+drag pricing at this dose -- drop it from future rungs. If-false (starve/park): hinge dose too high at k=2.0 -- halve k, do not touch the threshold (the 1.0 N*m line is probe-anchored).

**gate**: Walk battery: DR0 det+sto 6/6 @30s median fwd >=1.2m, zero terminations, gait_valid 12/12, det slip/m <=1.24 (parent 0.96). Posture/load (probe_walk_posture_load seed0 20s @0.055 primitive): mean body z offset vs anchor >= -10mm (parent -39.5); hip clamp-rail duty ~0 pct at |qfrc_actuator|>2.15 (parent L1 2.45pct of stance); stance hip tau p95 <=1.2 N*m (parent 1.58); L/R stance-hip asym <20pct (parent 28.1). Hinge-specific vs the un-hinged eff12-ds1 control at matched steps: reward_tau_over charge trends to ~0 and walk_tau_max_nm p95 <=1.2 by end; PASS requires beating or matching the control on z-offset and rail duty without losing the walk battery.

