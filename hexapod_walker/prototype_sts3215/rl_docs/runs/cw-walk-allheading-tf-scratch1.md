# cw-walk-allheading-tf-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-29T15:04:33+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**wandb_id**: xdi33tn0

**hypothesis**: Plain English: a brand-new walker trained from scratch to walk in EVERY direction (forward, reverse, crab, diagonals) equally well on the current mesh/100Hz robot, learning gait shape from the scripted tripod teacher while the new bank-proven windowed course-income reward pays supported net motion along the command. Operator order fb_20260829T144550_c921fa. This cycle's 8-heading probe proved the teacher walks ALL directions cleanly under this exact contract (0 falls, <=2.3deg net course err, completion 0.373-0.385 at every heading) — so the anchor target is valid everywhere. Transformer-preferred (2L/128d/8h/64-frame, the tf64 contract). This is a 2M MECHANISM canary: transformer x walk-tick BC anchor x course-income stack has never trained together. Prediction-if-true: bc walk-anchor loss falls, course-income share becomes nonzero and rises, terminations settle after warmup, reward tracks the MLP twin at matched steps. Prediction-if-false: the known transformer collapse signature (reward diverging DOWN vs the twin while the twin learns, entropy collapse, or NaN) -> fall back to GRU/MLP per the operator note. Strongest alternative: the 100Hz from-scratch reward valley makes 2M look bad in ABSOLUTE terms regardless of health — judged ONLY vs the matched-step twin (08-24 FACT), never absolute.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY, mechanism health ONLY (no behavior verdict): PASS if by 2M (1) no NaN/entropy collapse/termination explosion after 500k warmup; (2) train/bc_anchor_loss_walk falling; (3) course-income terms live (reward_walk_course_income nonzero and rising share, walk_course_income_support>0 on walk ticks); (4) reward trajectory within band of the matched-step MLP twin cw-walk-allheading-mlp-scratch1. FAIL only on the pre-registered collapse signature (reward diverging DOWN vs a learning twin, or numerical blowup). Healthy -> 40M acquisition whose cheap first gate is the balanced-heading panel: eval_cmd_suite 8 headings x 0.08 m/s + stop, det+sto, EVERY heading must move (completion >=0.19, half the teacher's own 0.373-0.385) with no falls — lateral/reverse weakness = not passed.

