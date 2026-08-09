# cw-walk-phase-stance2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T21:13:47+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a, on walk pod as init_stance_dr10.zip)

**wandb_id**: wjm6lrgy

**hypothesis**: Basin-escape phase arm RELAUNCH with audited settings (operator audit 02ea8cc, binding): stance-champion init + phase tripod reward + field-standard exploration (std 1.0 via --set-log-std 0.0, ent_coef 0.01, target_kl 0.02). Predecessor cw-walk-phase-stance killed at 2M (pre-audit std 0.37/ent 1e-3 = the under-exploration taint). Probe evidence (probe-phase-agree, 96k @ DR 0): mechanism drives ALL-SIX-leg cycling - det swings 6-21/leg, gait_valid 2/3. If-true: six-foot stepping at DR 0.2 with tracking converging; anneal DR after. If-false: stepping collapses back to statics or tracking never engages despite agreement. Comparison arm: cw-walk-phase (warm dr04b, DR 0.4, deliberate low noise).

**gate**: sto walk >=4/6 gait-valid @ vel_err <=0.035 on 0.02-0.06 @ DR 0.2 AND video shows all six feet cycling contact/swing AND sto rise >=4/6 retained (canaries armed, all four groups protected from stance parent)

**verdict**: FAIL (cycle 12): walk det 0/6 gait-valid @ vel_err 0.049 / sto 0/6 @ 0.046, speed 0.030 - phase-locked TRIPOD PARK: legs 1/3/5 duty 0.83-1.0, legs 0/2/4 held airborne (0.02-0.38) in 12/12 walk eps AND in hold (height 6/6 but end_posture 0/12) AND rise (height sto 4/6 = retention gate met as written; end_posture 1/12). Parent's six-footed hold destroyed. std grew to 1.47. Video: static tripod with three legs curled in the air; NOT WALKING. Phase-contact reward REFUTED in BOTH basins (warm arm kept shuffle; basin arm at std 1.0 parked a tripod group): the park avoids stepping costs and the zero-net phase term cannot outbid that. Probe cycling at 96k was transient. NOT HARDWARE-READY. Champion unchanged. ckpt md5 d077f6942c589f7db4cf1464f3061e97.

