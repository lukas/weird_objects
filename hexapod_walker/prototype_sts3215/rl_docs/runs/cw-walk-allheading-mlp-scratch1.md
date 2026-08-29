# cw-walk-allheading-mlp-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-29T15:08:40+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**wandb_id**: t681ghbu

**hypothesis**: Plain English: the matched-step MLP control twin for cw-walk-allheading-tf-scratch1 (the operator-ordered from-scratch all-heading walker, fb_20260829T144550_c921fa) — identical env/reward/teacher-anchor/obs (same 64-frame stack), only architecture differs (default 128,128 MLP, the hist64 contract). Exists because (a) the 08-24 ruling requires from-scratch 100Hz canaries be judged against a MATCHED-STEP control trajectory and the course-income reward stack is brand new (no reference trajectory exists for it), and (b) the operator note names GRU/MLP as the explicit fallback/comparison if the transformer shows its collapse signature. Prediction-if-true (both healthy): the two arms track each other through the early 100Hz valley at matched steps. Prediction-if-false: divergence localizes the failure — twin healthy + transformer sick = architecture problem (fall back per the note); both sick = mechanism problem (reward/anchor composition, fix before any architecture conclusion).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY, mechanism health ONLY (no behavior verdict): PASS if by 2M (1) no NaN/entropy collapse/termination explosion after 500k warmup; (2) train/bc_anchor_loss_walk falling; (3) course-income terms live (reward_walk_course_income nonzero and rising share); (4) mutual matched-step reward-band comparison with cw-walk-allheading-tf-scratch1. This arm doubles as the reference trajectory for all future all-heading scratch canaries. Healthy -> 40M acquisition with the same balanced-heading panel gate as the tf twin (eval_cmd_suite 8 headings x 0.08 m/s + stop, det+sto, every heading completion >=0.19, no falls).

