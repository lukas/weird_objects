# cw-uni-flag-a1-h1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T16:38:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-uni-flag-a1-h1

**hardware_ready**: no

**hypothesis**: More practice, not a new brain (mechanical retry of cw-uni-flag-a1-h1, which crashed at 0 steps to an infra bug -- W&B backend died on init because snapshot.sh's sync was shipping ~2GB of the controller's own stale wandb-cache junk onto the pod, now fixed): the one-network joystick policy (flagship stage A) already holds still and sits down cleanly and its stand-ups are honest but unfinished at 2M from scratch -- this run tests whether a 10M hardening pass finishes stand-up the way it finished the specialist's. Identical config/stack (hist16 + 256x256 + mode one-hot, stand-specialist reward + BC anchor, hold/rise/lower mix), warm-started from cw-uni-flag-a1-r1's checkpoint, budget is the only variable. Prediction-if-true: rise all-crouch valid_plant >=4/6 det with hold/lower retained by 10M (bc1-hard1 precedent: current/footprint tails resolve with budget). Prediction-if-false: rise valid_plant still <3/6 at 10M with flat rise_plant/feet factors despite budget = genuine shared-capacity interference -> fork to MoE per the flagship pre-registration.

**gate**: 10M det harness, own stack: rise (all-crouch) valid_plant >=4/6 AND hold valid_plant >=5/6 AND lower posture-strict >=5/6, zero STABLE known-exploit fingerprints (flag-leg/tripod/park/freeze) in video; MoE fork triggers if rise valid_plant <3/6 with flat rise factors.

**verdict**: INFRA FAILURE, not a science result: 0 training steps. Root cause confirmed by log/code-diff read: the trainer's plain --init-from warm-start path hard-refused --net-arch 256,256 (SystemExit "--net-arch has no effect on a plain --init-from warm start...") even though the checkpoint's own architecture IS 256,256 -- a pre-existing bug that killed every retry of this lineage (h1, h1-r1, h1-rr1), not the earlier-diagnosed wandb-cache/shm issues (those were red herrings for THIS crash; vec-env workers were already up when the exit fired, hence the leaked-shm warning as the only visible symptom). A concurrent cycle root-caused and fixed this in code (net-arch check now only refuses a genuine mismatch) and already relaunched the lineage as cw-uni-flag-a1-h2 (running on train-0, past 540k/10M steps at review time) -- no separate retry needed from this run.

**failed_reason**: 0 steps: --net-arch hard-refusal on plain --init-from warm start (fixed in code; superseded by cw-uni-flag-a1-h2)

