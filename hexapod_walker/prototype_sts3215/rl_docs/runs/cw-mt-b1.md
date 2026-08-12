# cw-mt-b1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T17:27:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: s0ttyuao

**hypothesis**: NARROW GENERALIST arm of the multitask A/B/C test (rl_docs/MULTITASK.md): does training stand + forward + small yaw SIMULTANEOUSLY from scratch (one command family, one coherent reward; commands change mid-episode) discover a STEERABLE gait, instead of the unsteerable forward paddle the sequential fine-tune path produced? Fresh init IS the hypothesis. Prediction-if-true: sign-correct yaw response both directions AND stands still on zero command, with forward tracking within reach of arm A. Prediction-if-false (acquisition failure): forward emerges but yaw response never does at matched budget - label acquisition/local-optimum per MULTITASK.md, NOT forgetting. Strongest alternative: command diversity starves discovery at 2M (nothing walks) - raise cohort budget, no reward tweaks.

**gate**: Video-first at 2M: (1) zero-command segments hold still (no march-in-place); (2) forward det prog med >= 0.5x arm cw-mt-a1's; (3) yaw probes at +-0.15 rad/s turn the correct way BOTH directions (eval_yaw/eval_drive probe). PASS = all three. FAIL(budget) if nothing walks; FAIL(acquisition) if forward emerges but yaw never responds. Verdict labels binding per MULTITASK.md.

