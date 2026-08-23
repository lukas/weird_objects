# smoke-loadslip-bootstrap-v1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T19:38:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000

**hypothesis**: Smoke: loadslip-bootstrap ramp (reward.walk_loadslip_bootstrap_steps) wires end-to-end without crashing and actually anneals.

**gate**: Trainer completes 40k steps, prints [loadslip-bootstrap] armed at step 0 and (if it finishes annealing) complete near end; no traceback.

**verdict**: Smoke confirms the new loadslip-bootstrap mechanism (reward.walk_loadslip_bootstrap_steps/_min_frac, walk_task.py + train_ppo_mjx.py) wires end-to-end: 40k-step trainer run printed '[loadslip-bootstrap] armed ... step-0 excess_scale=0.650' and '[loadslip-bootstrap] complete @ 24,576 steps', finished cleanly (0 tracebacks, ckpt+video saved). Licenses training on it.

