# cw-arch-tf64-small-joyfullcurr13-v7-hz100-canary-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T18:46:56+00:00

**pod**: hexapod-mjx-train-8

**steps**: 2000000

**parent**: cw-arch-tf64-small-joyfullcurr13-v7-hz100-canary

**wandb_id**: heklqc5l

**hypothesis**: Plain English: retry of the small-transformer 100Hz mechanism-health canary on FIXED infra -- the first attempt never trained a single step because pod train-8 was still mounting the legacy 64M /dev/shm k8s default and obs.history_frames=64 at n_envs=3072 makes the shared obs array alone ~54MB, so every worker died with SIGBUS inside env.reset() before any PPO step. This cycle recreated train-8 (and train-6) with the dshm-4Gi manifest and verified df -h /dev/shm now reports 4.0G; identical config otherwise (1L/d64/4h/ff128 transformer, obs.history_frames=64, control.hz=100, V7 certfreeze joystick recipe, 2M canary). Prediction-if-true: boots past reset, trains to 2M with no crash/NaN, reward and walkcurr b0 frontier/eval move together. Prediction-if-false: a genuine mechanism problem (crash/NaN/dead-flat reward, or reward-up-eval-flat 100Hz mismatch) now that the infra confound is removed. Strongest alternative: tf64-small trunk is under-capacity, reads as slow-but-healthy learning, answered at full budget not here.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (same as parent): PASS = boots and trains to 2M with no crash/NaN, healthy CUDA fps, reward/frontier/eval AGREEMENT. Directional gait NOT required at 2M. If PASS -> respec full 40M as cw-arch-tf64-small-joyfullcurr13-v7-hz100 (acquisition). If FAIL by misalignment -> audit; if under-capacity suspected -> escalate tf width/layers, not seeds. If it SIGBUS-crashes again identically, escalate to a pod-wide /dev/shm audit (infra, not architecture) rather than retrying a 3rd time blind.

