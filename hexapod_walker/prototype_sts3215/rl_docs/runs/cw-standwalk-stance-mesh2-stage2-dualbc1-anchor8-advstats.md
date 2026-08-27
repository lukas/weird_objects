# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor8-advstats

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T12:19:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix

**wandb_id**: mawejidv

**hypothesis**: Plain sentence: check whether PPO's own advantage normalization (not the BC anchor, not log_std, not the trunk) is quietly starving walk of its learning signal when a training batch mixes stance and walk ticks together. Neither the per-core log_std split (anchor6b) nor detaching the BC-anchor's gradient from the shared trunk (anchor7) reliably fixed the anchor4-class walk collapse -- anchor7 even REGRESSED this exact seed0's previously-clean walk. sb3-contrib's RecurrentPPO.train() normalizes advantages with ONE shared (mean,std) per minibatch (ppo_recurrent.py: advantages[mask].mean()/.std()); this stack has no VecNormalize/reward-scaling anywhere (grep-confirmed), and goal.mode_seq composes hold/rise/lower/walk INSIDE one episode, so minibatches are cross-mode by construction on essentially every update -- a previously-untested, purely-statistical (no shared weights needed) corruption path. This run adds ONLY a read-only diagnostic (train.bc_anchor_debug_adv_stats=1, zero effect on training -- unit-tested bit-exact when off) that logs the RAW pre-normalization per-mode-family advantage mean/std/share (train/adv_{loco,stance}_{mean,std,share}) on top of the EXACT anchor6b-logstdsplit-fix recipe (same seed, same init-from anchor2, no detach_trunk, no other change) so this seed's ALREADY-KNOWN clean-walk trajectory gets a first diagnostic read. Prediction-if-true (advantage-normalization-scope is a real contributor): train/adv_stance_std will be persistently much larger (several x) than train/adv_loco_std across training, most/all of the time. Prediction-if-false: the two std traces stay comparable in magnitude throughout -- ruling this mechanism out and pointing back at something else entirely (value-function coupling via the critic's own shared normalization, or PPO's max_grad_norm interacting with mixed-mode gradients).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY / DIAGNOSTIC-ONLY: this run does not change training behavior (bc_anchor_debug_adv_stats is read-only logging) and does not need a behavioral eval read to close -- the deliverable is the train/adv_loco_std vs train/adv_stance_std W&B history trace itself. WIRING CHECK FIRST: confirm the run's cached wandb_summary.json/history actually contains nonzero train/adv_loco_share + train/adv_stance_share entries (proves the diagnostic fired and the obs.mode_onehot tail was read correctly) before drawing any conclusion. Read: if stance std is persistently >>loco std (several x, most of training), the advantage-normalization-scope hypothesis is SUPPORTED -- design + build a per-mode-group advantage normalization variant next (default off, bit-exact) as the next mechanism arm, and do NOT fund another log_std/trunk-detach variant. If the two stds are comparable throughout, this hypothesis is REFUTED -- escalate to the critic/value-function coupling candidate instead. A behavioral gait_valid/gv read is NOT required to close this run (it is a training-side instrumentation probe on a matched-known-clean-seed baseline), but if the standard eval happens to run anyway, note it as a bonus.

