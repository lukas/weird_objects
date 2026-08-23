# smoke-loadslip-bootstrap-v1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T19:36:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000

**hypothesis**: Smoke: loadslip-bootstrap ramp (reward.walk_loadslip_bootstrap_steps) wires end-to-end without crashing and actually anneals.

**gate**: Trainer completes 40k steps, prints [loadslip-bootstrap] armed at step 0 and (if it finishes annealing) complete near end; no traceback.

**refused_reason**: hexapod-mjx-train-0 code marker 407687abef6d5213a14302d77e5dec9d213b93e5-dirty != local HEAD 407687abef6d5213a14302d77e5dec9d213b93e5 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

