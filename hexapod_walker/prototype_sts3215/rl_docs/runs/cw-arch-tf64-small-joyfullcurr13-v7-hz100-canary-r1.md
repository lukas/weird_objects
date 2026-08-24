# cw-arch-tf64-small-joyfullcurr13-v7-hz100-canary-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T18:46:56+00:00

**pod**: hexapod-mjx-train-8

**steps**: 2000000

**parent**: cw-arch-tf64-small-joyfullcurr13-v7-hz100-canary

**wandb_id**: heklqc5l

**hypothesis**: Plain English: retry of the small-transformer 100Hz mechanism-health canary on FIXED infra -- the first attempt never trained a single step because pod train-8 was still mounting the legacy 64M /dev/shm k8s default and obs.history_frames=64 at n_envs=3072 makes the shared obs array alone ~54MB, so every worker died with SIGBUS inside env.reset() before any PPO step. This cycle recreated train-8 (and train-6) with the dshm-4Gi manifest and verified df -h /dev/shm now reports 4.0G; identical config otherwise (1L/d64/4h/ff128 transformer, obs.history_frames=64, control.hz=100, V7 certfreeze joystick recipe, 2M canary). Prediction-if-true: boots past reset, trains to 2M with no crash/NaN, reward and walkcurr b0 frontier/eval move together. Prediction-if-false: a genuine mechanism problem (crash/NaN/dead-flat reward, or reward-up-eval-flat 100Hz mismatch) now that the infra confound is removed. Strongest alternative: tf64-small trunk is under-capacity, reads as slow-but-healthy learning, answered at full budget not here.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (same as parent): PASS = boots and trains to 2M with no crash/NaN, healthy CUDA fps, reward/frontier/eval AGREEMENT. Directional gait NOT required at 2M. If PASS -> respec full 40M as cw-arch-tf64-small-joyfullcurr13-v7-hz100 (acquisition). If FAIL by misalignment -> audit; if under-capacity suspected -> escalate tf width/layers, not seeds. If it SIGBUS-crashes again identically, escalate to a pod-wide /dev/shm audit (infra, not architecture) rather than retrying a 3rd time blind.

**verdict**: CANARY FAIL - MECHANISM: under-capacity branch. Trained clean to 2M (no crash/NaN, checkpoint+video synced) but reward DECLINED every quarter (-20.4/-197.1/-384.1/-609.4, ep_rew_mean -734 at end, never rising) while walkcurr frontier never promoted past b0 in any of 4 cert rounds. Own-cfg diagnostics: height_err_mm grew 0->99mm, walk_loadslip_ratio grew 0.2->5.76 (cap 3.0), walk_direction_valid fell 0.94->0.63 over the run -- a real regression, not slow-but-healthy learning. Video confirms: rollout_25 (early) is an ordinary static stand; rollout_49 (late) is a lower, more asymmetric crouch. Ran the held-out joygate anyway (not required at 2M, but decisive): gait_valid_frac 0.02 (1/48), slip/m 20-22 (cap 2.9), dir_err ~79deg (cap 40), leg-0 sacrificed in 47/48 episodes -- a near-permanent single-leg lock, zero falls (it drags/circles rather than topples). Root-cause isolation: the sibling MLP arch on the IDENTICAL 100Hz/V7-curriculum/reward stack (cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1, same family, currently 38.5M/40M) shows HEALTHY rising reward (quarters -1089/121/484/384, ep_rew_mean=746) -- proving the reward/100Hz combination is learnable and isolating the failure to the tf64-small (1L/d64/4h/ff128) transformer trunk itself, not a rate-conversion or reward-alignment defect. Caveat: this canary silently ran on CPU torch (train-8's CUDA install was wiped by the pod recreation; previously flagged acceptable for a 2M canary, not a factor in the behavioral collapse since CPU vs GPU only changes wall-clock not learning dynamics). Per the run's own pre-registered gate text: FAIL, under-capacity branch -> escalate tf width/layers, not seeds. Next: launching a wider/deeper transformer canary (2L/d128/8h/ff256, matching the proven tf-r1-hard1 config that DID learn to walk at 40M) on a CUDA-verified pod, single lever.

