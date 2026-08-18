# cw-dynrep-tf-liveactor-walkcurr4-canary1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-18T15:52:31+00:00

**pod**: hexapod-mjx-train-11

**steps**: 4000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: fpz5b59q

**hardware_ready**: False

**hypothesis**: Let the walking robot keep learning its physics intuition while it practices: instead of freezing the dynamics transformer that conditions its actor and critic, this canary trains that transformer LIVE on the walking the robot is actually doing (with 25% retained-corpus rehearsal and guarded boundary snapshots), while warm-starting the proven tall-gait champion's own actor AND critic across the history-stack widening. Operator-ordered (fb 20260818T153944Z) after every frozen-transformer walkcurr4 arm failed one layer deep: canB re-crouched, canC stood tall but would not walk, bridge2's fresh critic never reached usable EV in 2M. The bet: a transplanted proven critic + a live-updating predictive representation gives a usable value signal from step 0, so the imported gait survives B0 and starts climbing the sustained-joystick ladder. Prediction-if-true: pre-PPO B0 cert passes (zero falls, prog>=0.5); through 4M the source B0 is retained with zero falls, the curriculum promotes past B0, the actor transformer residual/gate goes nonzero, online heldout prediction improves on the pretrained encoder, and at least one guarded snapshot is ACCEPTED. Prediction-if-false: every snapshot is rejected (dead live-learning) or falls/rollbacks recur despite the live model - pointing at the ladder or live-replay distribution, not rollback mechanics. (-r1: first attempt SIGBUS-crashed at boot on train-4's 64M /dev/shm before any PPO step; retried once per DEAD protocol on train-11's 4G shm.)

**gate**: PRE-PPO fail-closed: cert-at-init B0 zero falls AND cmd_prog_frac>=0.5. IN-RUN fail-closed (trainer-enforced): CPU physics/Torch, encoder hash mismatch, action mismatch vs pred-gate-action-kl semantics, empty live replay, or absent predictor updates. 4M PASS requires ALL (operator bars, fb 20260818T153944Z): (1) source B0 retained ZERO falls across all cert rounds; (2) curriculum progresses (>=1 promotion past B0); (3) nonzero actor transformer residual; (4) online heldout prediction IMPROVES vs start-of-run pretrained reference; (5) >=1 guarded snapshot ACCEPTED (versions/rejections logged). PASS => operator-ordered 40M successor with this recipe. FAIL => NO 40M; name the failed bar.

**verdict**: FAIL on the operator's own 5-bar gate: bars 4 and 5 fail outright — online heldout prediction WORSENED (pred/heldout/total 2.286 at step0 -> 2.494 final, all 8 cert rounds), and zero guarded snapshots were ever accepted (pred/boundary_accepted_total=0, rejected_total=4/4) — the live transformer never actually updated the actor's world model. Bars 1/2/3 pass narrowly: B0 held zero falls all 8 cert rounds (walkcurr/b0_bridge_10s/falls=0 x8), one promotion b0->b1 at step 125 (frontier stuck at B1 pass=0 all subsequent rounds, no further promotion, no rollback), actor/predictive_residual_abs_mean=2.5e-05 (nonzero but negligible vs critic residual 15.8 -- consistent with the live encoder never being adopted). Per the pre-registered rule (FAIL on any bar => NO 40M): NO 40M successor. This is the walkcurr4 tournament's fourth and final distinct failure mode (canary rejects its own live-updated encoder every boundary) -- the whole live-transformer-actor bet is now closed pending a new operator order.

