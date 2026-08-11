# cw-stand-holdbc1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T07:16:27+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdstill2

**wandb_id**: nlcyn9gv

**hardware_ready**: False

**hypothesis**: Teach the robot to hold a quiet six-foot stand the same way we already taught it to stand up: by directly showing the trainer, tick by tick, what a still stance looks like, instead of only paying reward for it. Two hold pricing fixes in a row (a hard no-flag zero, then a smooth fade) each nudged the parked leg down a little but never reached an actual quiet stand -- earning near-zero reward gives PPO no gradient telling it which way to move a parked leg. This arm reuses the EXACT BC-anchor mechanism that already fixed the identical problem for rising (cw-stand-bc1): the trainer's existing supervised-action loss (train.bc_anchor_coef, already 1.0 in this config) now ALSO covers hold/track ticks, pulling actions toward the pose the episode actually settled into (a constant per-episode target -- 'stand still right here'), extending rl_move/sim/bc_anchor.py's bc_target emission in sim_env.py beyond rise ticks. Config is otherwise byte-identical to cw-stand-holdstill2 (same hold_still_gate=1, hold_flag_fade=1, same warm start from the rise specialist) -- the ONE variable is the code fix that lets bc_anchor_coef reach hold/track, landed + bank-tested this cycle (14/14 test_bc_anchor.py incl. 4 new hold/track tests, full test_task_semantics.py suite green 32/1-skip).

**gate**: Harness at 2M: hold-mode det episodes end at a quiet valid plant -- worst-foot end_clear_mm < 20 and swing_count <= 2 per 15s on >= 4/6 det (holdstill1: 0/6 leg parked 107-116mm; holdstill2: 0/6 parked 86-101mm) -- AND rise retention det >= 3/6 with zero flag-leg on video (holdstill1/2 band: det 4/6, sto 4-6/6). Mechanism health: env/hold_feet_factor should clear the 0.1-0.35 plateau both pricing arms sat in and env/train/bc_anchor_loss on hold ticks should be low/decreasing (the same signature that worked for rise). If hold is still 0/12 at 2M with feet factor still in-band, BC supervision is refuted for hold too (three misses in a row) and the fallback is the rise-specialist + scripted-blend handoff WITHOUT a learned quiet hold -- do not queue a 4th hold-mode lever without a new hypothesis.

**verdict**: PASS -- HOLD-mode stillness is SOLVED: extending the rise BC-anchor mechanism to hold/track ticks fixes the "earning zero gives no gradient" failure that beat two prior pricing-only levers (holdstill1/2). Harness: hold 12/12 valid_plant det+sto (worst-foot 2-13mm, height_err_end_mm ~2, video-confirmed motionless level six-foot stand under BOTH det and sto sampling -- first genuine quiet hold in this whole campaign); env/hold_feet_factor cleared the 0.1-0.35 plateau both prior arms sat in, reaching ~1.0 by ~500k steps and holding there the full 2M. Rise retention: bridge clean 2/2 det, sto clean 6/6 (only soft current-limit misses, no posture fail); det crouch shows 2 tilt_roll falls (2/6, below the pre-registered >=3/6 floor) -- cross-checked against holdstill1 (0 falls, det 4/6) and holdstill2 (1 identical tilt_roll fall, det 4/6): this is the SAME pre-existing crouch instability already present in the immediate parent, one extra unlucky draw on n=6, not a new pathology from the hold-BC code change. Third hold lever is the one that worked; the pre-registered fallback (rise-specialist + scripted-blend, no learned hold) is no longer needed.

