# cw-uni-flag-a1-h1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T16:27:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-uni-flag-a1-r1

**hardware_ready**: no

**hypothesis**: More practice, not a new brain: the one-network joystick policy (flagship stage A) already holds still and sits down cleanly and its stand-ups are honest but unfinished at 2M from scratch — this run tests whether a 10M hardening pass finishes stand-up the way it finished the specialist's. Hardening continuation of cw-uni-flag-a1-r1: identical config/stack (hist16 + 256x256 + mode one-hot, stand-specialist reward + BC anchor, hold/rise/lower mix, n-envs 3072 shm fix), warm-started from its checkpoint, budget is the only variable. Prediction-if-true: rise all-crouch valid_plant >=4/6 det with hold/lower retained by 10M (bc1-hard1 precedent: current/footprint tails resolve with budget). Prediction-if-false: rise valid_plant still <3/6 at 10M with flat rise_plant/feet factors despite budget = genuine shared-capacity interference -> fork to MoE per the flagship pre-registration. Strongest alternative: rise converges but hold/lower erode under shared capacity — that would also argue MoE.

**gate**: 10M det harness, own stack: rise (all-crouch) valid_plant >=4/6 AND hold valid_plant >=5/6 AND lower posture-strict >=5/6, zero STABLE known-exploit fingerprints (flag-leg/tripod/park/freeze) in video; MoE fork triggers if rise valid_plant <3/6 with flat rise factors.

**verdict**: INFRA FAILURE, not a science result: 0 training steps, no checkpoint. CORRECTION (superseded by a concurrent cycle's confirmed diagnosis): my original read blamed a snapshot.sh wandb-sync contamination bug -- that bug was REAL and I fixed it (tar now also excludes the top-level prototype_sts3215/wandb/, previously only rl_move/wandb/ was excluded, ~1.9GB/560+ stale dirs were being shipped to every pod on every sync), but it was NOT the cause of this crash: the confirmed root cause (log/code-diff read, credited to the concurrent cycle handling cw-uni-flag-a1-r1) is that the trainer's plain --init-from warm-start path hard-refused any non-default --net-arch (SystemExit) even when it matched the checkpoint's own architecture -- this run's --net-arch 256,256 legitimately matched cw-uni-flag-a1-r1's checkpoint but was refused anyway. The abrupt SystemExit mid-startup, combined with stdout buffering, is what produced the misleading 'wandb backend crashed'-looking log (buffered Warp-init/vec-env-up lines flushed at process death after the unbuffered SystemExit line). Fixed in train_ppo_mjx.py (accept a matching --net-arch, refuse only a genuine mismatch), relaunched as cw-uni-flag-a1-h2 (running). My own mechanical retry (cw-uni-flag-a1-h1-r1) hit the identical still-unfixed-at-the-time bug and is verdicted separately. The snapshot.sh sync fix stands on its own merits (real waste, now fixed) but is NOT credited as fixing this failure.

**failed_reason**: trainer hard-exit at 0 steps: respec-cloned --net-arch 256,256 refused on a plain --init-from warm start (train_ppo_mjx.py old check). Root cause fixed 08-11: matching --net-arch now proceeds, only a genuine arch mismatch refuses. Superseded by cw-uni-flag-a1-h2.

