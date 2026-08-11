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

**verdict**: INFRA FAILURE, not a science result: W&B backend crashed inside wandb.init() ~1s after start (HandleAbandonedError, 'Reached EOF' with no run ID) -- 0 training steps, no checkpoint, pre-stage pullckpt found nothing. Root-caused: snapshot.sh --sync's tar excluded rl_move/wandb/ but NOT the top-level prototype_sts3215/wandb/ (the actual local wandb cache, 1.9GB/560+ stale run dirs on the controller) -- every sync shipped that whole pile onto the pod, including a controller-run wandb.init()/finish() artifact (from the concurrent cycle's r1 verdict-publish step) landing in the pod's wandb/ dir ~90s before h1's own wandb.init(). Fixed (snapshot.sh: added the missing --exclude='prototype_sts3215/wandb'; tar dry-run confirms 0 wandb files now shipped) and cleaned the confirmed-foreign run dir + stale top-level symlinks off train-0. Mechanical retry queued as cw-uni-flag-a1-h1-r1 (identical spec, respec --now on the fixed sync path); the underlying rise/hold/lower hardening question is untouched and unanswered by this run.

**failed_reason**: run never appeared as 'running' in W&B within 240s

