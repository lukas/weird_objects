# cw-mt-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T17:26:40+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**wandb_id**: zjp4z2gn

**hardware_ready**: False

**hypothesis**: BROADER GENERALIST arm of the multitask A/B/C test (rl_docs/MULTITASK.md): cw-mt-b1's command family plus small lateral commands (heading cap 0.34 rad = vy up to ~+-0.02 m/s at 0.06). Does the wider family still discover walking at matched budget, and does it INTERPOLATE to held-out (vx,vy,wz) command triples never sampled in training? Fresh init IS the hypothesis. Prediction-if-true: forward + yaw as in B, plus sign-correct lateral response and no falls on the three MULTITASK.md zero-shot probes. Prediction-if-false: lateral width breaks discovery at 2M - informs whether the command family needs a curriculum rather than more width. Strongest alternative: lateral response exists but only via body-yaw cheating (turn-then-walk), visible on video.

**gate**: cw-mt-b1's full gate PLUS: sign-correct lateral response on +-0.02 m/s vy probes, and the three MULTITASK.md zero-shot interpolation probes (vx=0.037 wz=0.07; vx=0.025 vy=0.012; vx=0.05 wz=-0.11) complete without falls. Verdict labels binding per MULTITASK.md.

**verdict**: FAIL(budget) per MULTITASK.md labels: nothing walks at 2M (det prog med 0.10, gait_valid 0/6, 3-leg sacrifice episodes, low-crouch creep on video) — cohort stopped by cw-mt-a1`s pre-registered under-budget branch; no independent verdict on lateral/interpolation possible at this budget. Probes deferred to the 20M re-queue (cw-mt-c2). Mild monotone ordering a1>b1>c1 in det prog (0.22/0.16/0.10) noted as budget-cost-of-diversity signal, not a verdict.

