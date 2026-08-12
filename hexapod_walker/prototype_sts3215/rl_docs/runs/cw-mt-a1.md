# cw-mt-a1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T17:25:26+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**wandb_id**: y0hts56j

**hardware_ready**: False

**hypothesis**: Cohort CONTROL arm for the multitask A/B/C test (rl_docs/MULTITASK.md): a fixed-command forward specialist trained from scratch under the exact recipe the generalist arms (cw-mt-b1/c1) use, so their command-distribution effect is the only variable. Fresh init IS the cohort hypothesis (warm-start default deliberately waived). Prediction-if-true: a valid forward gait emerges by 2M as in the cw-dep-fresh1 lineage. Prediction-if-false: 2M is under-budget for this recipe and the WHOLE cohort re-runs longer before any A/B/C conclusion. Strongest alternative: the added resample/yaw-obs plumbing (on but inert in this arm) changes discovery vs fresh1.

**gate**: At 2M, own-cfg det 6 eps: recognizable forward stepping on video, gait_valid majority, 0 terminations, det prog med > 0.3. PASS = cohort budget is adequate, proceed to B/C triage. FAIL = cohort under-budget: re-queue all three arms at a matched higher budget; per-arm tweaks forbidden (MULTITASK.md).

**verdict**: FAIL(budget) per its own gate: at 2M the specialist control arm shows no recognizable walking — det prog med 0.22 (<0.3 bar), gait_valid 0/6 det (leg-3 sacrificed every det ep), video is a low-crouch splay creep. Pre-registered branch fires: cohort under-budget, all three arms re-queued at a matched higher budget (20M, the cw-dep-fresh1 donor recipe`s proven from-scratch budget); per-arm tweaks forbidden per MULTITASK.md.

