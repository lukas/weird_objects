# amp-m0-joycmd-asymcritic-smoke-v3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T07:01:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 500000

**hypothesis**: M0 checklist smoke (v3, from-scratch per AMP's from-scratch-by-design charter -- v1/v2 wrongly warm-started from a joystick-track checkpoint and hit obs-width transplant conflicts with --asym-critic): does --asym-critic (privileged critic, ported this cycle) compose end-to-end with a FRESH policy over an AMP-brief-shaped joystick command envelope (widened stress_mix cfg bundle: speed 0-0.60 m/s, full-circle heading, yaw_cmd 0-1.0 rad/s w/ 0.15 zero-frac, resample 0.5-3.0s jittered, mixed abrupt/ramped blend 0.05-1.0s) at n_envs=4096 without instability. Prediction-if-true: finite losses/rewards for the full 500k-step smoke, checkpoint loads as AsymActorCriticPolicy, no NaNs/crashes -> AMP STATUS Next item 1 CLOSED, ready for real motion-library/discriminator work on top. Prediction-if-false: NaN/crash or reward not moving at all -> a real composition bug between asym-critic and the widened envelope, dig in before M1.

**gate**: Non-scoring infra smoke (W&B disabled): PASS = process completes or is healthily still training past ~200k steps with finite reward/loss curves in the log and (if it finishes) a checkpoint that loads as AsymActorCriticPolicy; FAIL = crash/NaN/traceback.

**verdict**: PASS (infra smoke, not scoring): --asym-critic composes end-to-end with a FRESH from-scratch policy over the widened AMP-brief joystick envelope (stress_mix, speed 0-0.60 m/s, full-circle heading, yaw_cmd 0-1.0 rad/s @0.15 zero-frac, resample 0.5-3.0s jittered, mixed 0.05-1.0s blend) at n_envs=4096 -- 500k steps, 2783 env-steps/s incl. setup, finite losses/values throughout (value_loss ~29-32, explained_variance 0.64-0.76, std stable ~0.368, no NaN/crash), video reel ok on all sampled modes, checkpoint verified on-pod loads as AsymActorCriticPolicy with the expected 73-dim actor obs space. v1/v2 attempts wrongly warm-started from a joystick-track phase-clone checkpoint and hit two rounds of obs-width mismatches (--obs-pad-transplant is explicitly incompatible with --asym-critic) -- corrected by going from-scratch per AMP charter design. AMP STATUS Next item 1 (wire+confirm the joystick envelope with asym-critic) CLOSED.

