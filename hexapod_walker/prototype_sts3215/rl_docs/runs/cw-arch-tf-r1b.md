# cw-arch-tf-r1b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T12:12:54+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-arch-hist16-r7

**wandb_id**: kymrqt44

**hypothesis**: Temporal-arch discovery rung (relaunch of cw-arch-tf-r1, relocated to train-1 with CUDA torch + --device cuda after the CPU-torch update path measured ~200 fps): small causal transformer trunk (2 layers, d_model 128, 4 heads, ff 256, separate actor/critic trunks, 622k params vs hist16 MLP 331k) attending over the SAME 16-frame window and EXACT recipe as champion cw-arch-hist16-r7, from scratch. One variable changed vs r7: policy trunk (flatten-MLP -> causal attention). This 2M rung answers ONLY: does the transformer+PPO stack boot at 3072 envs, train stably (no NaN/collapse, healthy fps on cuda), and start moving without an early cheat lock-in? Architecture verdict belongs to the 40M hardening twin (r7 itself showed no gait by 2M).

**gate**: PASS = boots and trains to 2M with no crash/NaN, healthy cuda fps (>=2000), ep_rew climbing, and the 1M/2M det walk evals free of the leg-sacrifice fingerprint (no 3-leg park). Directional gait NOT required at 2M. FAIL = crash, flat/dead reward, or leg-sacrifice/paddle already locked in det. If PASS -> respec 40M from-scratch hardening twin of hist16-r7.

