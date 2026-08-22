# cw-dep-bcgait4-phasedir10-allowramp-a

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: UNDERTRAINED

**created**: 2026-08-22T15:07:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun13

**wandb_id**: fxhsnig3

**hypothesis**: The 4-seed phasedir9 lineage sample (longrun17 PASS-partial 1/4, seed13/23/29 FAIL) is stuck on a regime-gap pricing bug, not a seed lottery: the det-calibrated reward.drag_stance_allow_mm=24 charges the honest gait 0.76-9.7x its own per-tick income while PPO's action-noise std is still high (0.135), because the noisy-honest stance-travel tail needs >=48mm of headroom while the det drag cheat pays zero past 36mm (logs/ckpt_eval/pd8_digin_regime/, probe_stance_slip_dist). This arm is a single-change A/B against longrun13 (seed13, FAIL 0.792x progress/1.286x slip, same 4M-step/anneal-frac-0.3 recipe): arm the newly-built, bank-tested reward.drag_stance_allow_ramp_steps=1.2M (matches the log-std-anneal end-step) so the allowance starts loose (48mm, sized above the measured noisy-honest tail) and anneals down to the same validated 24mm target exactly as PPO's exploration noise falls, instead of taxing the honest noisy gait from step 0. (Renamed -a: the plain name's snapshot tag was already pushed by a crashed prior attempt that died before the ledger write / any pod launch -- see RL_LOG, same disambiguation precedent as pd9's own stale-tag race.)

**gate**: Same clone-relative forward panel as the whole lineage (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Prediction-if-true: seed13 now clears or comes materially closer to the gate than longrun13's own 0.792x/1.286x (progress up, slip down, W&B env/reward_drag_stance trending near-zero only once the ramp completes near 1.2M, not from step 0) -- promotes the ramp as the lineage's real fix, next arm reproduces on seed17/23/29. Prediction-if-false: seed13 lands at/below longrun13's own numbers despite the looser early allowance -- the regime-gap theory is wrong or the ramp schedule doesn't reach the basin-selection window in time; next lever is a slower ramp (larger ramp_steps) before abandoning the mechanism.

**verdict**: Regime-gap fix (drag_stance_allow_ramp: 48mm->24mm over the same 1.2M-step window as the log-std anneal) tested on seed13. DR-0 det progress 0.64/0.77=0.831x clone (vs longrun13's 0.792x under the OLD fixed 24mm allowance -- improved, still <0.9x cap), slip 2.19/1.89=1.159x clone (vs longrun13's 1.286x -- improved, NOW inside the 1.2x cap), speed 0.047 m/s, zero falls, gait_valid 6/6 both DR-0 and own-DR0.35. This is the FIRST lever in the whole phasedir1-10 lineage to move BOTH progress and slip toward the gate simultaneously (every prior lever traded one for the other -- drag charge/allowance tightening always cost speed, std-anneal alone plateaued). W&B reward quarters [-45.6,-62.4,143.2,175.8] still rising at the 4M cutoff (not plateaued), so per the 08-21 ruling (rising reward + gate progress in the SAME direction, not a diverging reward-vs-gate story like the -9b/-cont1 continuations) this reads as UNDERTRAINED, not a refuted lever. NOT extending this exact checkpoint by continuation (the lineage's own binding rule: repairs re-init, and -9-cont1/-9b already showed low-std continuations can drift to a worse basin even with rising reward) -- the next budget question belongs to a fresh, longer-budget re-init if the seed23/seed29 generalization checks (queued this cycle + a concurrent cycle) confirm the ramp is a general fix, not seed13-specific.

