# cw-stance-raisemix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**pod**: hexapod-sweep-friction

**parent**: ppo_goal_cw_stance_dr10.zip

**wandb_id**: bl11do1r

**checkpoint**: pod friction: rl_move/sim/policies/ppo_goal_cw_stance_raisemix.zip (md5 7162ee621e56cd33c7d358a953f928b6)

**hypothesis**: raise stalls at DR 1.0 because the default goal mix under-trains it; raise=0.4,rise=0.2,lower=0.2 un-sticks it

**gate**: raise >=5/6 det+sto AND rise/lower >=5/6 all start kinds retained

**verdict**: FAIL (cycle 10): raise det 3/6 / sto 4/6 (gate >=5/6 both). Failure classification: 5/5 near-miss under-lift 6-8mm (one 18.5), zero terminations, tilt <=1.22 deg, no hot legs; legs 2/4 unloaded in ALL raise episodes. Crown jewels intact: rise det 17/18 / sto 18/18, lower 12/12, hold 6/6 at DR 1.0. Hypothesis REFUTED -> raise DEMOTED to canary status per review section 7; stance champion unchanged (cw-stance-dr10).

