# smoke-logstd-anneal-v2-freeze-fix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-23T14:45:39+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000

**hypothesis**: Plain English: verify the log-std anneal no longer silently freezes PPO. The v1 smoke only checked the std schedule; both 08-23 amp anneal arms completed 2M steps with ZERO weight updates because the on_rollout_end anneal inflated first-minibatch KL past the target_kl=0.02 early-stop. After moving the set to on_rollout_start, an aggressive anneal (-2 -> -4.5 over 40k steps) must BOTH follow the schedule AND change the network weights vs the init checkpoint. Prediction-if-true: saved policy tensors differ from init and 'Early stopping at step 0' does not fire on every update. Prediction-if-false: weights byte-identical again — fix wrong, dig deeper before any relaunch.

**gate**: policy.pth non-log_std tensors differ from init checkpoint AND <100% of updates early-stop at step 0; checked by hand post-run

**verdict**: crashed at load: obs space mismatch 93!=74 — smoke env lacked the amp family's obs-widening cfg keys (yaw cmd, fault_health, amp_style_obs); relaunched as v2b with the twins' full cfg-set

