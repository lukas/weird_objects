# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T02:16:24+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**wandb_id**: hvyjdmch

**hypothesis**: Plain English: does training the V7/100Hz/hist64 recipe FROM SCRATCH with the already-built anti-leg-sacrifice reward gate (reward.walk_gait_gate=1.0) turned on from step 0 avoid ever forming the {0,2,5}-leg-sacrifice basin that the identical from-scratch recipe without the gate (cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1) converged to and FAILed on? walk_gait_gate multiplicatively collapses walk/progress/tracking income toward zero whenever any support leg hasn't completed a real swing within a 2s window -- purpose-built+bank-tested (test_walk_gait_gate_*, quadwalk precedent) but never included in this recipe until the sibling continuation cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-cont1 (a warm-start rescue of the already-collapsed scratch-s0-r1 checkpoint, owned by another cycle, still training). This arm is the from-scratch half of that same question: prevention instead of rescue, decoupled from whether the collapsed basin is too deep to escape by reward alone.

**gate**: PASS: DR-0 det gait_valid 6/6, no leg duty pinned near-zero (no [X,Y,Z]-style sacrifice), walkcurr/frontier promotes past b0. PARTIAL: gait_valid improves over the no-gate parent's 0/6 but some leg sacrifice or frontier stall remains. FAIL: identical {0,2,5} (or any 3+ leg) sacrifice signature reproduces despite the gate being active from step 0 -- would mean walk_gait_gate's income collapse is too weak/late to prevent the basin, not just too weak to rescue from it, and the leg-sacrifice root cause is NOT reward-side at all (points at the still-unclaimed structural/indexing DIG-IN, OPERATOR_QUESTIONS.md 08-24/08-25 leg-sacrifice notes).

