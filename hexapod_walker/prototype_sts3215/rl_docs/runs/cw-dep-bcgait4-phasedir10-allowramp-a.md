# cw-dep-bcgait4-phasedir10-allowramp-a

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T15:07:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun13

**wandb_id**: fxhsnig3

**hypothesis**: The 4-seed phasedir9 lineage sample (longrun17 PASS-partial 1/4, seed13/23/29 FAIL) is stuck on a regime-gap pricing bug, not a seed lottery: the det-calibrated reward.drag_stance_allow_mm=24 charges the honest gait 0.76-9.7x its own per-tick income while PPO's action-noise std is still high (0.135), because the noisy-honest stance-travel tail needs >=48mm of headroom while the det drag cheat pays zero past 36mm (logs/ckpt_eval/pd8_digin_regime/, probe_stance_slip_dist). This arm is a single-change A/B against longrun13 (seed13, FAIL 0.792x progress/1.286x slip, same 4M-step/anneal-frac-0.3 recipe): arm the newly-built, bank-tested reward.drag_stance_allow_ramp_steps=1.2M (matches the log-std-anneal end-step) so the allowance starts loose (48mm, sized above the measured noisy-honest tail) and anneals down to the same validated 24mm target exactly as PPO's exploration noise falls, instead of taxing the honest noisy gait from step 0. (Renamed -a: the plain name's snapshot tag was already pushed by a crashed prior attempt that died before the ledger write / any pod launch -- see RL_LOG, same disambiguation precedent as pd9's own stale-tag race.)

**gate**: Same clone-relative forward panel as the whole lineage (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Prediction-if-true: seed13 now clears or comes materially closer to the gate than longrun13's own 0.792x/1.286x (progress up, slip down, W&B env/reward_drag_stance trending near-zero only once the ramp completes near 1.2M, not from step 0) -- promotes the ramp as the lineage's real fix, next arm reproduces on seed17/23/29. Prediction-if-false: seed13 lands at/below longrun13's own numbers despite the looser early allowance -- the regime-gap theory is wrong or the ramp schedule doesn't reach the basin-selection window in time; next lever is a slower ramp (larger ramp_steps) before abandoning the mechanism.

