# cw-dep-bcgait4-phasedir10-allowramp-a

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T15:07:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun13

**wandb_id**: fxhsnig3

**hypothesis**: The 4-seed phasedir9 lineage sample (longrun17 PASS-partial 1/4, seed13/23/29 FAIL) is stuck on a regime-gap pricing bug, not a seed lottery: the det-calibrated reward.drag_stance_allow_mm=24 charges the honest gait 0.76-9.7x its own per-tick income while PPO's action-noise std is still high (0.135), because the noisy-honest stance-travel tail needs >=48mm of headroom while the det drag cheat pays zero past 36mm (logs/ckpt_eval/pd8_digin_regime/, probe_stance_slip_dist). This arm is a single-change A/B against longrun13 (seed13, FAIL 0.792x progress/1.286x slip, same 4M-step/anneal-frac-0.3 recipe): arm the newly-built, bank-tested reward.drag_stance_allow_ramp_steps=1.2M (matches the log-std-anneal end-step) so the allowance starts loose (48mm, sized above the measured noisy-honest tail) and anneals down to the same validated 24mm target exactly as PPO's exploration noise falls, instead of taxing the honest noisy gait from step 0. (Renamed -a: the plain name's snapshot tag was already pushed by a crashed prior attempt that died before the ledger write / any pod launch -- see RL_LOG, same disambiguation precedent as pd9's own stale-tag race.)

**gate**: Same clone-relative forward panel as the whole lineage (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Prediction-if-true: seed13 now clears or comes materially closer to the gate than longrun13's own 0.792x/1.286x (progress up, slip down, W&B env/reward_drag_stance trending near-zero only once the ramp completes near 1.2M, not from step 0) -- promotes the ramp as the lineage's real fix, next arm reproduces on seed17/23/29. Prediction-if-false: seed13 lands at/below longrun13's own numbers despite the looser early allowance -- the regime-gap theory is wrong or the ramp schedule doesn't reach the basin-selection window in time; next lever is a slower ramp (larger ramp_steps) before abandoning the mechanism.

**verdict**: FAIL vs the rung-A pass bar, but the regime-gap fix WORKS as predicted: single-change A/B vs longrun13 (identical seed13/4M-steps/anneal-frac-0.3 recipe, only reward.drag_stance_allow_ramp_steps=1.2M/ramp_mm=48 added) moved BOTH clone-relative axes in the pass direction simultaneously for the first time in the 9-arm phasedir9/longrun lineage: progress 0.792x->0.830x clone (still <0.9x cap), slip 1.284x->1.162x clone (still just above 1.15x cap, was 1.284x). Speed 0.061 in-band, zero falls, gait_valid 6/6 det+sto both DR-0/own-DR, clean 6-leg video (no sacrifice/drag pathology). Pod log confirms mechanism fired as designed: [drag-allow-ramp] armed at step0 (48mm) -> ramp complete @ 1,245,184 steps, exactly matching [log-std-anneal] complete @ 1,245,184 (std 0.041) -- the allowance tightened in lockstep with the noise anneal, not before it. This is the prediction-if-true branch (materially closer on both axes) short of full clearance -- not the prediction-if-false branch (which required landing at/below longrun13). Every earlier lineage lever (phasedir3-9) only ever traded one axis for the other; this is the first to move both together, real evidence the regime-gap diagnosis is correct, just under-dosed for seed13. Next: one more turn of the dial (slower/longer ramp) and a cross-seed check (does the same ramp rescue seed23/29's worse baselines) before promoting or abandoning.

