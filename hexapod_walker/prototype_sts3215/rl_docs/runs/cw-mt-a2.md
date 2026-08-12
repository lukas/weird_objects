# cw-mt-a2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T20:32:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-mt-a1

**wandb_id**: 31war42x

**hardware_ready**: False

**hypothesis**: Give the from-scratch specialist walker enough training to actually walk, so the multitask A/B/C comparison has a working control arm. This is cw-mt-a1 unchanged (fixed forward command vx=0.05, no yaw, no stops, fresh init IS the hypothesis per MULTITASK.md) at the donor recipe's proven 20M budget instead of 2M. Prediction-if-true: a gait-valid forward walker comparable to cw-dep-fresh1 (det prog med >1.0, gait_valid majority). Prediction-if-false: recipe drift since fresh1 means even the specialist cannot walk at 20M — cohort design must be re-audited before any multitask conclusion. Strongest alternative: walks but paddles (prog fine, gait_valid failing) — still a usable control for the gait-structure comparison.

**gate**: At 20M, own-cfg det 6 eps: recognizable forward stepping on video, gait_valid majority det, 0 terminations, det prog med > 0.8. PASS = control arm valid, proceed to B/C triage per MULTITASK.md. FAIL = cohort recipe broken at proven budget: STOP the wave, re-audit recipe vs cw-dep-fresh1 before any re-queue (no third budget rung without operator).

**verdict**: PASS — control arm valid at 20M. Own-cfg(DR0.2) det: prog med 1.30 (gate >0.8), gait_valid 6/6, 0 terminations; gate(DR0) det: prog med 1.23, gait_valid 6/6, 0 terms, sto likewise clean. Video/contact-sheet confirm genuine six-leg cycling (per-leg duty_cycle 0.26-0.72, swing_count 7-25/leg, no sacrificed legs) — real stepping, not a flag/paddle exploit. Quality caveat: slow (speed ~0.06-0.14 m/s vs cmd 0.05), high slip_per_m ~1.4-1.5 (heavy foot-drag/paddle-ish; roll stays clean, tail<=1.8deg). Unlocks B/C triage per MULTITASK.md pre-registration.

