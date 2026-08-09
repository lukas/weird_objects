# cw-stance-lowerdense

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T05:57:51+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_stance_bellyrest.zip

**wandb_id**: 7iyhg21d

**hypothesis**: Spear-leg/allowance-riding hover survives because the end-posture charge exempts the entire descent (terminal 1.5s window) and hovering below the 60mm allowance is free; the hover is USED during the ramp (tilt guard) where it is currently unpriced. Charging foot clearance over the WHOLE lower episode (reward.end_posture_lower_dense=1, same term/k/magnitude, window only; a correct feet-planted lower pays exactly 0) prices the behavior where it is chosen. If-true: lower det summed over-allowance <80mm (bellyrest: 80-107) AND lower posture >=1/6 any pass AND frames show feet staying planted through the descent; gate pass possible. If-false shapes: (a) spears persist and PAY the dense charge (reward_end_posture integral grows, clearances unchanged) -> hover is held by a stronger opposing gradient -> tilt/current interplay diagnosis, no more shaping arms; (b) all legs converge to 40-60mm hover throughout (threshold-riding generalizes; eval would PASS while frames show a hover) -> the allowance dead zone is proven the binding defect -> operator ruling on the 60mm allowance required before any further stance arm. Strongest alternative: dense charging prices away the tilt guard and destabilizes the descent -> visible as tilt terminations/height-clause erosion (stop rule covers). Parent ppo_goal_cw_stance_bellyrest.zip md5 6212b44f (heights 24/24, hold 12/12 intact). Probe probe-stance-lowerdense-b PASS. Snapshot cafd213.

**gate**: posture-strict CPU harness @ DR 1.0, 6 eps/mode det+sto, explicit --modes lower rise hold raise track, eval cfg WITHOUT lower_belly_start_frac: lower end-posture >=5/6 sto AND >=4/6 det AND rise/lower height-only >=5/6 both AND hold sto 6/6; lower verdict must eyeball per-leg end_clear_mm for 40-60mm clustering and negatives; stop rule: rise HEIGHTS <5/6 anywhere

**verdict**: FAIL (lower posture 0/12 vs gate >=4/6 det >=5/6 sto; RISE-HEIGHTS STOP RULE TRIPPED: det 2/6 sto 2/6 vs parent 12/12, one tilt_roll fall + one over_current termination on camera; hold 12/12 and track 12/12 intact; lower heights 12/12). HYPOTHESIS REFUTED, both if-false shapes: (a) spears persist and PAY the dense charge (reward_end_posture -0.306->-0.339 over training; leg0 det 98-129mm vs bellyrest 112-145) and (b) threshold-riding generalized (leg4 det endings 54-64mm clustered AT the 60mm boundary; summed det over-allowance 68-81 vs bellyrest 80-107 fell by relabeling, not planting). Opposing-gradient diagnosis (controller probe, 3 seeds): dense charge costs 4.5-11.3/ep vs ~220 income; a planted descent pays current_hot -17.7 vs hover -4.3 and hits Imax 2.63A (hover 1.8-2.0A); support_margin pays hover +35 vs planted +24. NOT HARDWARE-READY: three feet airborne at lower end (~130mm spear), rise falls over det[0]. Champion unchanged. Checkpoint ERODED (rise) - never warm-start stance work from it. Stance line BLOCKED on operator allowance ruling per pre-registration.

