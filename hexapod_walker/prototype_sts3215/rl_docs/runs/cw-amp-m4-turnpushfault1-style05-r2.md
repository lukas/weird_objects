# cw-amp-m4-turnpushfault1-style05-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T02:32:47+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m3-turnpush1-style05-acq1-r2

**wandb_id**: 87yq6shs

**hypothesis**: Plain English: retry of the dead-at-birth cw-amp-m4-turnpushfault1-style05 with its config bug fixed -- grafts fault onto the SOLID turn+push acquisition checkpoint (8M, PASS: prog 1.11, topples 0/6+2/6, tip errs 0.14/0.12), the sequential-composition route named as the alternative to turnfault1-style05's fresh 3-way stack (which missed its own safety bar at 2M: gait_valid 9/12 vs 10/12). The original died at birth: it copy-pasted turnfault1-style05's obs-pad-transplant=18 wiring but its own --init-from arg was mistakenly pointed at the turnfault1-style05 checkpoint (already fault-widened, 93-dim) instead of the intended turnpush1-style05-acq1-r2 checkpoint (75-dim, no fault yet) -- 0 steps trained, W&B run finished empty. This retry uses launch_run.py respec --init-from-source (guarantees the ACTUAL turnpush1-style05-acq1-r2 output checkpoint is used, not a hand-copied path) with only the fault graft added: --obs-pad-transplant=18 + dr.fault_prob=1.0 + obs.fault_health=1, matching turnfault1-style05's own wiring exactly. Prediction-if-true: mechanism-safety bar clears (gait_valid >=10/12 det+sto) where the fresh 3-way stack turnfault1-style05 missed (9/12) -- two of three axes no longer need simultaneous discovery, sequential order is the fix. Prediction-if-false: also misses the bar -- 3-way composition is hard regardless of substrate solidity, budget (not order) is the lever.

**gate**: Discovery (2M, DR-0, own cfg: fault+push+yaw all active). Mechanism-safety bar (M4 new-mechanism discipline, matching turnfault1-style05's own bar): gait_valid >=10/12 det+sto, faulted episodes limp not statue on video, no crouch, zero NaN/crash. If it clears where the fresh-stack turnfault1-style05 missed (9/12): the sequential route is confirmed as the fix for 3-way composition, extend to acquisition budget next. If it ALSO misses the bar: 3-way composition is hard regardless of substrate solidity, budget (not order) is the lever -- acquisition-continue this arm instead of respending on ordering. Compare det/sto prog+slip against turnpush1-style05-acq1-r2's own numbers (1.11/1.20 prog, no sacrificed) and run eval_yaw manually to check turn tracking survived triple-stacking.

