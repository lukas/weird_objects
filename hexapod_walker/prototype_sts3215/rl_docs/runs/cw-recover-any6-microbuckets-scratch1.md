# cw-recover-any6-microbuckets-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-15T23:20:04+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-recover-any5-mjxcert-scratch1

**wandb_id**: 78xjmlov

**hardware_ready**: False

**hypothesis**: Teach the fallen robot to stand back up by climbing a ladder of 17 baby-step difficulty buckets instead of the old coarse ones; this arm tests whether removing the two difficulty cliffs the operator found (B4->B5 and B5->B6) lets a from-scratch policy climb past where any4/any5 stalled. OPERATOR-ORDERED (fb_20260815T230538_a6f8d2, commit 3d556232 'Refine recovery curriculum and bucket scores'): genuinely FROM SCRATCH — no --init-from, no obs-pad transplant, no checkpoint load of any kind; parent is recipe provenance only. Clones any5's complete recover-only 40M Warp/MJX recipe (DR, BC anchor, safety, PPO, 1M deterministic MJX certification cadence with 8 cert envs, eval/video cadence, all MDP settings). New under 3d556232: micro-bucket ladder B0 plant_catch .. B16 flip; promotion authority is the literal latest deterministic held-out batch fraction per bucket (>=0.8 promotes, <0.2 retreats, episode minima; EMA never gates); exact stochastic training scores count EVERY terminal env as TRAIN/recover_bucket_N_success_fraction/_successes/_episodes; deterministic authority metrics are CERT/recover_bucket_N_success_fraction/_successes/_episodes. Prediction-if-true: CERT B0 fraction appears at ~1M with an 8-episode denominator, passes >=0.8, frontier promotes, and the ladder climbs smoothly past the old cliff region. Prediction-if-false: the fraction stalls at some rung with full denominators — which then names the true difficulty wall instead of a cliff artifact.

**gate**: HARD integration gate at ~1M steps: CERT/recover_bucket_0_success_fraction PRESENT with _successes and _episodes (8-episode denominator) and frontier before/after logged; W&B config carries the full recover_buckets B0-B16 number->kind map; TRAIN/recover_bucket_0_success_fraction with _successes/_episodes logging each rollout. Old CERT/*_success names and env EMA are never admission signals. Full-arm PASS bar: legitimately earned frontier (>=0.8 deterministic cert fraction per promotion) climbing past the old B4/B5-equivalent rungs by 40M, with video-verified genuine recover-to-stand (all six feet loaded, no flag/stilt/park) on the earned frontier.

**verdict**: PASS on the pre-registered hypothesis: the micro-bucket curriculum (17 baby-step buckets, CERT-gated >=0.8/<0.2 promotion) climbed CLEANLY and legitimately from B0 to B15 by 22M/40M steps -- no B4/B5/B6-equivalent cliff, the exact artifact any4/any5 were built to diagnose is GONE. It held the B15 peak (tangle+bank) for 5 straight 1M-step certs (tangle 62.5-100%, bank 75-100%) before a single bank-kind cert crashed to 12.5% at 28M, triggering the designed retreat; the remaining ~12M steps oscillated B13-B14 (tangle_mid/deep, CERT fractions ~0.25-0.75, never resettling >=0.8) and finished at B13. Video (own-pod gate eval + training-video spot checks) shows genuine six-foot-loaded stable stands at low roll/pitch (roll_tail 0.1-0.3deg, height err ~0mm) on every bucket CERT certified, incl. tangle_mild/tangle_mid and bank -- no flag-leg/stilt/park exploit anywhere. NEW WALL NAMED for the next arm: tangle-family recovery (self-tangled legs, moderate-to-deep) plus the harvested bank pose subset is the real remaining difficulty, not a curriculum artifact -- still actively contested (not flatlined at 0) when the budget ran out. Evaluator note: the generic eval_checkpoint single-episode recover probe marks some already-at-target starts (crouch_shallow/mid/deep, zero) as non-success despite a visually perfect held stance (tilt <0.5deg, height err 0mm) through the whole 16s episode -- looks like a start-already-at-goal blind spot in that harness, not a policy regression; CERT (many-episode, training's own criterion) is the gate's designated authority and shows those families were legitimately mastered earlier. No SKILLS.md row: recover is not yet reliable/hardware-ready.

