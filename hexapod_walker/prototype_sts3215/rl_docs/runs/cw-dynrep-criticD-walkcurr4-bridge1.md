# cw-dynrep-criticD-walkcurr4-bridge1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CRASHED_BUG

**created**: 2026-08-18T11:00:40+00:00

**pod**: hexapod-mjx-train-11

**steps**: 4000000

**parent**: cw-dynrep-criticD-walkcurr4-gaitinit-hard1

**wandb_id**: ytfh9o3j

**hardware_ready**: False

**hypothesis**: Teach the frozen-dynamics-critic walker by starting from the robot's proven tall walking brain and, at first, only asking it to do exactly what it already does (walk straight ahead at its own ~0.05 m/s) while the new critic catches up — then gently widen back toward the full joystick command set. This arm tests the operator's evidence-based correction (fb_20260818T102844_116d4c) after all six walkcurr4 canaries failed in two OPPOSITE ways: scratch actors never ignite upright walking (canA-r2/canB-r1/canC-r1), and transplanted actors under the aggressive recipe either lose commanded travel (gaitinit-bcinit, prog 0.0575) or keep a real gait but fall chasing V2's non-adjacent 0.08-0.12 m/s ignition band (gaitinit-hard1, gait_valid 6/6 but falls 4/8 rounds). Recipe (the causal combo the data supports): actor-only transplant from ppo_goal_cw_dep_bcgait1_hard1 (proven: height -8..-13mm, slip 1.3, zero falls); NEW WALKCURR_BUCKETS_V3 bridge ladder (B0 straight 0.05-0.06 m/s no jitter/resample/stops DR0 = the source's own operating point, B1 0.05-0.10 straight, B2+ = V2 direction/heading ladder verbatim so the multi-direction goal is preserved); NEW --actor-freeze-steps 500000 (actor group lr=0 while the fresh condition-D critic adapts, train/actor_frozen logged); then release at 5e-5 x3 epochs, target_kl 0.01, KL-rollback 0.03 (post-promo raise to 1e-4 NOT wired - only if needed per the order); NEW --walkcurr-cert-at-init re-runs the exact pre-PPO deterministic cert in-run and refuses to train over a broken transplant (preflight already PASSED: prog 1.168, falls 0). Prediction-if-true: B0 certifies during or right after the freeze window and promotes by <=1M; frontier >=B2 at 4M with source-quality slip/height. Prediction-if-false: even a frozen proven actor + adjacent commands cannot certify B0, meaning the blocker is structural to the critic-D/walkcurr reward wiring, not actor competence, curriculum adjacency, or update aggression - that closes the actor-init lever class for this track. Strongest alternative: the critic adapts but the released actor still drifts off the gait at 5e-5 (watch post-500k cert trend vs the frozen rounds).

**gate**: PRE-PPO (in-run, fail-closed): walkcurr/pre_b0_* logged with falls==0 and cmd_prog_frac>=0.5 (preflight measured 1.168). BEHAVIORAL GATE at 4M, on cert telemetry + walkcurr panels: (1) bridge B0 promotion by <=1.0M steps; (2) at 4M frontier>=B2 (both bridge rungs certified+retained); (3) final cert round on every certified bucket: cmd_prog_frac>=0.60, height_factor>=0.80, slip_per_m<=2.0, falls==0. PASS => the triaging cycle AUTOMATICALLY launches the pre-registered 40M successor cw-dynrep-criticD-walkcurr4 with the IDENTICAL recipe (actor-only hard1 transplant, frozen critic-D md5 9df48f687967c25085ee50171e4110ff, V3 bridge curriculum, 0.5M actor freeze, 5e-5 x3 epochs, tk 0.01/rb 0.03) - operator-ordered, no new decision needed. FAIL => NO 40M; name which bar failed and compare against gaitinit-hard1's final round (prog 0.60/falls 4-8) to separate freeze-helped from freeze-irrelevant.

**verdict**: No behavioral verdict — mechanical crash at 2.007M/4M steps, NOT a training-quality result. This was the FIRST run in the whole condition-D/--actor-lr walkcurr family to reach a walkcurr promotion+rollback (B0 promoted well inside the 1M gate; the rollback crashed). Root cause: save_stock_optimizer writes every --actor-lr checkpoint's policy.optimizer as a fresh single-group stock snapshot (by design, for eval compatibility); the walkcurr in-training rollback blindly load_state_dict'ed that into the LIVE 2-group actor/critic optimizer, which torch refuses. Progress before the crash was GOOD and licenses a retry: cmd_prog_frac 0.92-1.02 on both bridge rungs (gate 0.6), slip_per_m 1.3-1.4 (gate 2.0), height_factor 0.83-0.84 (gate 0.8), B0 promoted fast. Fixed at the root this cycle: new load_optimizer_state_if_compatible() (update_health.py) gates the optimizer reload on matching param-group count, else skips it and keeps the exactly-restored policy weights (same fresh-Adam-moments contract warm starts already accept) -- 3 new unit tests, 19/19 test_value_learning.py green, full semantics bank clean. Retried as cw-dynrep-criticD-walkcurr4-bridge1-retry1 (identical recipe/seed, VERIFIED RUNNING on hexapod-mjx-train-5).

