# smoke-logstd-anneal-v2b-freeze-fix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T14:49:10+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000

**hypothesis**: Plain English: verify the log-std anneal no longer silently freezes PPO (v2 retry: v2 crashed on obs-space mismatch, now using the frozen twins' full cfg-set). After moving the anneal set to on_rollout_start, an aggressive anneal (-2 -> -4.5 over 40k steps, the exact rate class that froze the 2M twins) must BOTH follow the schedule AND change the network weights vs the init checkpoint. Prediction-if-true: saved policy non-log_std tensors differ from init and 'Early stopping at step 0' does not fire on every update. Prediction-if-false: weights byte-identical again — fix wrong, dig deeper before any relaunch.

**gate**: policy.pth non-log_std tensors differ from init checkpoint AND <100% of updates early-stop at step 0; checked by hand post-run

**verdict**: PASS (mechanism smoke): the on_rollout_start anneal fix unfreezes PPO under the exact freeze-provoking condition. 40k steps, -2 -> -4.5 anneal (same 2.5-nat aggressive rate class that froze the 2M twins): all 12 non-log_std policy tensors moved vs the init checkpoint (max abs delta 0.0097; the frozen twins had literally 0), 'Early stopping at step 0' count is ZERO (twins: 31/31), log_std followed the schedule exactly (-4.048 at the last rollout start). Licenses the -r2 relaunches of stdanneal45/swinganneal45 on tag exp/logstd-anneal-rollout-start-fix.

