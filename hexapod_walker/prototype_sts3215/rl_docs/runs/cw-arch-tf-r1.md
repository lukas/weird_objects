# cw-arch-tf-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-15T11:51:05+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-hist16-r7

**wandb_id**: m0r5qmct

**hypothesis**: Temporal-arch discovery rung: small causal transformer trunk (2 layers, d_model 128, 4 heads, ff 256, separate actor/critic trunks, 622k params vs hist16 MLP 331k) attending over the SAME 16-frame window and EXACT recipe as champion cw-arch-hist16-r7, from scratch. One variable changed vs r7: policy trunk (flatten-MLP -> causal attention). Tests a genuinely different memory mechanism after GRU-from-scratch closed on leg-sacrifice/paddle. This 2M rung answers ONLY: does the transformer+PPO stack boot on MJX/warp at 3072 envs, train stably (no NaN/collapse, usable fps), and start moving without an early cheat lock-in? The architecture verdict belongs to the 40M hardening twin (r7 itself showed no gait by 2M).

**gate**: PASS = boots and trains to 2M with no crash/NaN, fps usable (>=2000), ep_rew climbing, and the 1M/2M det walk evals free of the leg-sacrifice fingerprint (no 3-leg park). Directional gait NOT required at 2M. FAIL = crash, flat/dead reward, or leg-sacrifice/paddle already locked in det. If PASS -> respec 40M from-scratch hardening twin of hist16-r7.

**verdict**: KILLED by operator agent, no science read: pods run CPU-only torch, so the transformer PPO update ran ~240s/iter (~200 fps, 2M ETA ~3h, 40M ~2 days) while the H200 idled. Installed CUDA torch (2.11.0+cu128, --no-deps against the existing jax nvidia-cu12 stack; jax verified intact) on train-1 and benchmarked the same update at 2.0s (~120x). Relaunching the identical discovery spec as cw-arch-tf-r1b on train-1 with --device cuda.

