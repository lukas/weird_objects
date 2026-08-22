# smoke-amp-asymcritic-mjx

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-22T04:44:22+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2048

**hypothesis**: Wiring smoke: does the newly-ported --asym-critic flag construct+train on the GPU/Warp trainer (train_ppo_mjx.py) without crashing, matching train_ppo_sim's proven asym_policy.AsymActorCriticPolicy behavior?

**gate**: PASS if the run reaches its step budget with a finite, moving reward and no traceback; this is a mechanism-health canary, not a behavior claim.

**verdict**: Wiring smoke PASS: --asym-critic ported cleanly to the GPU/Warp trainer. On-pod verification loads the saved checkpoint as AsymActorCriticPolicy with privileged_idx=(70,71) matching the walk tasks 2 measured-velocity obs dims (72-wide obs, 70 unmasked on the actor path, critic sees all 72); 2048/2048 steps trained with finite losses (value_loss 50.9, entropy -7.54, kl 0.0047) and video reels logged for every mode (rise/walk/raise/hold). The launcher marked this FAILED on a verify-timing race (process legitimately finished its 63s smoke before the poll found the pid, same false-flag pattern seen on other short runs) -- the pod log/checkpoint prove clean completion, no traceback.

**failed_reason**: process died; log tail:
     |
|    explained_variance   | 0.00268      |
|    learning_rate        | 0.0003       |
|    loss                 | 25.2         |
|    n_updates            | 5            |
|    policy_gradient_loss | -0.0238      |
|    std                  | 0.368        |
|    value_loss           | 50.9         |
------------------------------------------
[video] logged reel (64 steps | rise:ok walk:ok raise:ok hold:ok)
[video] logged reel (final | rise:ok walk:ok raise:ok hold:ok)
[mjx-train] done: 2,048 steps in 63s (32 env-steps/s incl. setup) -> /workspace/prototype_sts3215/rl_move/sim/policies/smoke_amp_asymcritic_mjx.zip
[mjx-train] evaluate with the C-env harness before trusting anything (MJX_PORT.md phase-2 item 4).
[wandb] checkpoint artifact ckpt-smoke_amp_asymcritic_mjx (md5 92f4d1b1)


