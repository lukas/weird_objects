# amp-m0-joycmd-asymcritic-smoke-v2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-22T06:59:19+00:00

**pod**: hexapod-mjx-train-0

**steps**: 500000

**hypothesis**: M0 checklist smoke (v2, fixed obs-width bug from v1: added goal.walk_phase_obs/phase_hz to match the phase-clone init-from checkpoint's obs width): does --asym-critic (privileged critic, ported this cycle) compose end-to-end with an AMP-brief-shaped joystick command envelope (widened stress_mix cfg bundle: speed 0-0.60 m/s, full-circle heading, yaw_cmd 0-1.0 rad/s w/ 0.15 zero-frac, resample 0.5-3.0s jittered, mixed abrupt/ramped blend 0.05-1.0s) at n_envs=4096 without instability. Prediction-if-true: finite losses/rewards for the full 500k-step smoke, checkpoint loads as AsymActorCriticPolicy, no NaNs/crashes -> AMP STATUS Next item 1 CLOSED. Prediction-if-false: NaN/crash or reward flat -> a real composition bug, dig in before M1.

**gate**: Non-scoring infra smoke (W&B disabled): PASS = process completes or is healthily still training past ~200k steps with finite reward/loss curves in the log and (if it finishes) a checkpoint that loads as AsymActorCriticPolicy; FAIL = crash/NaN/traceback.

**verdict**: obs-space mismatch again (75 vs 74): phase_obs adds 2 dims not 1, and the phase-clone checkpoint itself has NO yaw_cmd dim -- warm-starting a NEW obs axis (yaw command) onto an old-width checkpoint needs --obs-pad-transplant, which is explicitly incompatible with --asym-critic. Root cause of v1+v2: using an --init-from checkpoint at all for an AMP-track smoke was the wrong call -- AMP is from-scratch by design (CURRENT_TRUTHS), no warm-start obs-width constraints should apply. Relaunching v3 WITHOUT --init-from.

