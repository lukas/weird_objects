# cw-stance-bellyrest

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T04:49:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_stance_endpost_c1.zip

**wandb_id**: sfkdjvil

**hypothesis**: Spear-leg hover survives because planted-belly states are never visited (approach-from-above stalls on a redistribution manifold; std 0.197 cannot sample to contact). Starting 35% of lower eps AT the planted belly pose (height ref 0, rest quietly) makes the zero-charge basin visited and learned. If-true: env/reward_end_posture resumes descending >=0.12 over the run AND lower det over-allowance <80mm (c1: 123-156) AND lower posture >=1/6 any pass. If-false: belly-start eps rest planted but plant-start descents unchanged (~130-150mm, spear legs 0/4) -> hover preferred under descent dynamics -> descent-posture ref or operator review of 60mm allowance; terminal pricing stays closed. Strongest alt: context-split despite shared states = the if-false signature. Confound named: GPU-MJX stack switch forced by c51b3e2 (gate harness stays CPU exact-path). Budget 20M = ~305 large-batch updates, matching the c1 segment in optimizer work. Parent ppo_goal_cw_stance_endpost_c1.zip md5 d6e909af. Probe probe-stance-bellyrest PASS. Snapshot c2af9cb.

**gate**: posture-strict CPU harness @ DR 1.0, 6 eps/mode det+sto, explicit --modes: lower end-posture >=5/6 sto AND >=4/6 det AND rise/lower height-only >=5/6 both AND hold sto 6/6; stop rule: rise HEIGHTS <5/6 anywhere

**verdict**: FAIL (lower posture 0/12 vs gate >=4/6 det >=5/6 sto; heights 24/24, hold 12/12 intact). HYPOTHESIS REFUTED (if-false): frac=1.0 belly-start eval 12/12 = basin visited/learned, plant-start descents unchanged (worst_clear 139-145mm, spear leg0 + leg2/leg4 riding the 60mm allowance). NEW: threshold-riding of the allowance in both contexts; reward_end_posture WORSENED 0.08 over run. NOT HARDWARE-READY. Champion unchanged. Next: reward.end_posture_lower_dense arm; 60mm allowance to operator review.

