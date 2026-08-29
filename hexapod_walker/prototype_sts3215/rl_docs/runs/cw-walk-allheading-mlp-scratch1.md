# cw-walk-allheading-mlp-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-29T15:08:40+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**wandb_id**: t681ghbu

**hypothesis**: Plain English: the matched-step MLP control twin for cw-walk-allheading-tf-scratch1 (the operator-ordered from-scratch all-heading walker, fb_20260829T144550_c921fa) — identical env/reward/teacher-anchor/obs (same 64-frame stack), only architecture differs (default 128,128 MLP, the hist64 contract). Exists because (a) the 08-24 ruling requires from-scratch 100Hz canaries be judged against a MATCHED-STEP control trajectory and the course-income reward stack is brand new (no reference trajectory exists for it), and (b) the operator note names GRU/MLP as the explicit fallback/comparison if the transformer shows its collapse signature. Prediction-if-true (both healthy): the two arms track each other through the early 100Hz valley at matched steps. Prediction-if-false: divergence localizes the failure — twin healthy + transformer sick = architecture problem (fall back per the note); both sick = mechanism problem (reward/anchor composition, fix before any architecture conclusion).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY, mechanism health ONLY (no behavior verdict): PASS if by 2M (1) no NaN/entropy collapse/termination explosion after 500k warmup; (2) train/bc_anchor_loss_walk falling; (3) course-income terms live (reward_walk_course_income nonzero and rising share); (4) mutual matched-step reward-band comparison with cw-walk-allheading-tf-scratch1. This arm doubles as the reference trajectory for all future all-heading scratch canaries. Healthy -> 40M acquisition with the same balanced-heading panel gate as the tf twin (eval_cmd_suite 8 headings x 0.08 m/s + stop, det+sto, every heading completion >=0.19, no falls).

**verdict**: CANARY PASS -- mechanism-health only (matched-step MLP control twin for the all-heading transformer canary, 2M steps, mesh/100Hz, identical env/reward/BC-anchor stack, same 64-frame history obs). Evidence (W&B history, cached logs/experiments/cw-walk-allheading-mlp-scratch1/): (1) no collapse -- train/std rises 0.369->0.414, zero NaN, terminations/over_current shows the same transient shared bump ~800k-1.05M steps (peak 86) fully resolving to single digits by 1.1M (tf twin: 108/44/101/59 in the same window -- shared training dynamics, not an architecture-specific blowup); truncated stays flat 15-34. (2) train/bc_anchor_loss_walk falls monotonically 0.0030->0.00047. (3) course-income mechanism live the whole run: reward_walk_course_income/support never zero (0.007-0.86 / 0.30(min ~0.28)-0.75... in range 0.30-0.93 through most of run), dips through the mid-run valley then ticks back up in the final ~100k steps (support 0.30->0.30->0.33, income 0.012->0.013->0.032) matching the tf twin's recovery shape. (4) reward trajectory closely tracks the tf twin at every matched step (quarters 73.1/87.0/122.4/172.1 vs tf's 74.4/84.8/127.7/162.3, final ep_rew_mean 173.4 vs 166.4) -- confirms this arm's own role as the reference trajectory, no divergence. Frame strips (walk_det_0-5, logs/ckpt_eval/cw_walk_allheading_mlp_scratch1_gate/) show the robot upright, six legs planted/spread, no topple -- visually matches the tf twin. Per the gate's own text: healthy canary -> 40M acquisition with the same balanced-heading eval_cmd_suite gate. Launching cw-walk-allheading-mlp-acq1 (respec --init-from-source, +40M, phase=acquisition) this cycle -- keeping the matched pair alive through acquisition preserves the twin-comparison tool for future collapse-signature checks.

