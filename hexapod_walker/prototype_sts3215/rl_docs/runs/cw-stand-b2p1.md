# cw-stand-b2p1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T15:48:52+00:00

**pod**: hexapod-mjx-train-1

**steps**: 18000000

**parent**: ppo_goal_cw_stance_dr10.zip

**wandb_id**: sara3uwi

**hypothesis**: RL_PLAN queue item 0 / RL_GOALS 'Standing up' plan step 3, first arm of the walkable-height rise line (operator ruling 08-10: the champion's ~70mm crouch-stand is not the deliverable, the ~142mm plant stance the walk line lives in is). Warm-starts the stance champion (ppo_goal_cw_stance_dr10, genuine feet-down rise/lower on today's post-273ebde sim) with: a plant-height target (goal.rise_height_mm=[108,114], actions.max_height_mm=115, vs the champion's native ~70mm crouch), the belly-to-plant reference track (reward.k_rise_ref_track=2.0 against rise_ref_belly2plant.npz), and the posture-pricing fixes landed for the walk-env line (reward.rise_posture_gate, rise_income_prog_gate, rise_finish_gate_signed=1 -- income/finish now require feet actually loaded, not just torso height). Also carries bus.servo_params=loaded (rise is the highest-load motion the loaded actuator fit exists for). Pre-validated by tmp_smoke_rise_ref.py: with this exact reward stack, replaying the demonstrated belly->plant path out-earns both a freeze exploit and a stilt/bridge exploit by a wide margin (+952 vs +225 vs -195) -- trying-well beats trying-badly beats not-trying, and not-trying is net negative. If-true (posture-strict rise/lower >=5/6 at the plant height): the unified deliverable's rise problem is solved by warm-start + this reward stack, no distillation needed; promote toward the crown-jewel rise/lower champions at plant height. If-false: check whether the SAME flag-leg/bridge cheat cw-stance-riseproof1 (this cycle, sibling control probe) just reproduced without these flags persists even WITH posture-gating -- if so the posture-gate mechanism itself needs a stronger geometric criterion (harness end_posture_ok uses per-foot ground clearance <20mm; training's gate may need tightening to match) before another plant-height attempt. Known risk (recorded, not yet mitigated): the height obs channel extrapolates to 2.2x its trained range (height_scale_m 0.05 unchanged for graft compatibility with the champion).

**gate**: posture-strict harness (default end-posture gate): rise AND lower success >=5/6 det each by 18M, h_err<=12mm of the +105-114mm plant target AND end_posture_ok; hold quiet (height_err_end<=8mm); VIDEO: no leg-through-floor, no flag-leg/bridge (worst pad clearance small, all feet genuinely loaded at finish); early call permitted if W&B rise/lower success flat 0 at 6M

