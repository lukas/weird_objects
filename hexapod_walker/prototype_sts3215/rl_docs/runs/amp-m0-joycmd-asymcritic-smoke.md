# amp-m0-joycmd-asymcritic-smoke

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-22T06:56:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 500000

**hypothesis**: M0 checklist smoke: does --asym-critic (privileged critic, ported this cycle) compose end-to-end with an AMP-brief-shaped joystick command envelope (widened stress_mix cfg bundle: speed 0-0.60 m/s, full-circle heading, yaw_cmd 0-1.0 rad/s w/ 0.15 zero-frac, resample 0.5-3.0s jittered, mixed abrupt/ramped blend 0.05-1.0s) at n_envs=4096 without instability. Prediction-if-true: finite losses/rewards for the full 500k-step smoke, checkpoint loads as AsymActorCriticPolicy, no NaNs/crashes -> AMP STATUS Next item 1 CLOSED, ready for real motion-library/discriminator work on top. Prediction-if-false: NaN/crash or reward not moving at all -> a real composition bug between asym-critic and the widened envelope, dig in before M1.

**gate**: Non-scoring infra smoke (W&B disabled): PASS = process completes or is healthily still training past ~200k steps with finite reward/loss curves in the log and (if it finishes) a checkpoint that loads as AsymActorCriticPolicy; FAIL = crash/NaN/traceback.

**verdict**: obs-space mismatch (74 vs 73): cfg-set bundle omitted goal.walk_phase_obs=1/walk_phase_hz, which the init-from phase-clone checkpoint requires (+1 obs dim); crashed at load, 0 steps trained. Not a real finding -- fixing cfg and relaunching as -v2.

