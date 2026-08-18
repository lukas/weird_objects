# cw-dep-bcgait1-hard1-steer1c

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-18T15:48:48+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: bpz63iwd

**hardware_ready**: False

**hypothesis**: Teach the strong tall walker to survive abrupt joystick direction changes without tangling its legs: this 2M canary checks that the long-episode multi-command training recipe (120 s episodes, all six command-schedule families, INSTANT no-blend switches, irregular 2-20 s dwells) boots and trains healthily on the hard1 champion without erasing its tall tripod gait. Ordered by operator fb_20260818T152717_278879. The rot60 on/off probe (probe_dirswitch_tangle, 08-18) showed sector crossings are NOT the tangle trigger — rot60 ON is strictly safer on every proxy — so plain transition exposure is the smallest correct fix; the gaps to train away are yaw-limit saturation after switches (margins pressed past the hard limit) and legs that stop cycling for seconds after a command change. Prediction-if-true: finite losses, KL rollback machinery quiet, periodic eval keeps tall height and zero-fall while episodes now contain many abrupt switches. Prediction-if-false: 120 s episodes + conservative actor LR destabilize PPO (KL rollbacks firing constantly / value loss diverging) or the gait visibly collapses within 2M — meaning the recipe needs staged dwell curriculum instead of the full mix.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only): PASS if the run reaches 2M with finite losses, train/approx_kl bounded (kl-rollback fires <20% of updates), episodes average >60 s (no mass early termination), and the last periodic eval shows the tall gait not collapsed (no fall explosion, height not re-crouched). FAIL on boot failure, KL/value divergence, or immediate catastrophic forgetting of the tall gait. NO mature-behavior verdict at 2M; the larger hardening continuation launches only after this passes.

**verdict**: CANARY PASS (mechanism-health only): reached 2M cleanly (found finished on the pod; ledger was stale -- mechanically confirmed via W&B state=finished + pod process exited). All bars: (1) finite losses throughout (final loss=70.4, value_loss=134, entropy stable); (2) train/approx_kl=0.0069 << the 0.04 rollback trigger, kl_rollback_count=0 the whole run (0% << the 20% cap) -- PPO update health never in question; (3) 'episodes average >60s' read literally (ep_len_mean=459 steps=18.4s at dt=0.04s) looks like a fail, but this bar is ARITHMETICALLY UNREACHABLE for this run's own config: 2M steps / 4096 envs = ~496 steps/env (~19.8s) total budget for the WHOLE canary against a 120s(3000-step) episode cap, so no env could ever be truncated and no fair average could exceed ~20s regardless of stability -- the actual no-early-termination signal (env/reward_termination=0 at the final log point, ep_len_mean tracking ~93% of the max-possible elapsed per-env budget throughout training, only 17 cumulative truncated-episode events) says episodes are essentially NOT dying early, they are running the full available window; (4) last periodic eval (1,003,520 steps) and final video reel both show the tall gait intact: walk err 0.052 m/s, tipped 2/2 pass, rolltrap 2/2 pass, walk_height_factor=0.895 (not re-crouched), video reel 'walk:ok' x4 at both the mid checkpoint and the final one -- no fall explosion. Spec note for future canaries: pick episode_seconds so the per-env step budget (steps/n_envs) comfortably exceeds it, or the 'avg episode length' bar cannot be evaluated as literally written. Per the pre-registered rule, canary PASS => launch the ~20M hardening continuation with the operator's admission panel (this cycle: queued as cw-dep-bcgait1-hard1-steer1-hard20m1). A supplementary DR-0/own-DR harness eval (video) was also started on the pod for the operator's eyes; artifacts land in logs/ckpt_eval/ when it finishes (note: an accidental duplicate podeval launch by this cycle was cleaned up mid-run -- the surviving pass may take longer than usual; harmless to the verdict, which rests on training telemetry per the gate's own mechanism-health text).

