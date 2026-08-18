# cw-dynrep-criticD-walkcurr4-bridge1-retry1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-18T11:15:18+00:00

**pod**: hexapod-mjx-train-5

**steps**: 4000000

**parent**: cw-dynrep-criticD-walkcurr4-bridge1

**wandb_id**: q623592v

**hypothesis**: Teach the frozen-dynamics-critic walker by starting from the robot's proven tall walking brain and, at first, only asking it to do exactly what it already does (walk straight ahead at its own ~0.05 m/s) while the new critic catches up, then gently widen back toward the full joystick command set. Identical recipe/seed to cw-dynrep-criticD-walkcurr4-bridge1, which crashed mid-run at 2.007M/4M steps on a mechanical bug (walkcurr in-training rollback blindly load_state_dict'ing a save_stock_optimizer single-group checkpoint into the live 2-group actor/critic optimizer -- torch's 'different number of parameter groups'), NOT a training-quality failure: progress before the crash was already inside the pre-registered gate on both bridge rungs (cmd_prog_frac 0.92-1.02, slip_per_m 1.3-1.4, height_factor 0.83-0.84, B0 promoted well inside the 1M window). Root-caused and fixed at the source this cycle (load_optimizer_state_if_compatible in update_health.py: skip the optimizer reload on a group-count mismatch, keep the exactly-restored policy weights, same fresh-Adam-moments contract warm starts already accept -- default-off elsewhere, only exercised by walkcurr's in-training rollback path), 3 new unit tests green (19/19 test_value_learning.py), full semantics bank re-run clean. This retry re-runs the exact same recipe/seed to see the run through its first promotion+rollback cycle and out to the pre-registered 4M gate. Parent: cw-dynrep-criticD-walkcurr4-gaitinit-hard1 (same lineage as the crashed run).

**gate**: PRE-PPO (in-run, fail-closed): walkcurr/pre_b0_* logged with falls==0 and cmd_prog_frac>=0.5 (preflight measured 1.168, unchanged). BEHAVIORAL GATE at 4M, on cert telemetry + walkcurr panels: (1) bridge B0 promotion by <=1.0M steps; (2) at 4M frontier>=B2 (both bridge rungs certified+retained); (3) final cert round on every certified bucket: cmd_prog_frac>=0.60, height_factor>=0.80, slip_per_m<=2.0, falls==0. PASS => the triaging cycle AUTOMATICALLY launches the pre-registered 40M successor cw-dynrep-criticD-walkcurr4 with the IDENTICAL recipe (actor-only hard1 transplant, frozen critic-D md5 9df48f687967c25085ee50171e4110ff, V3 bridge curriculum, 0.5M actor freeze, 5e-5 x3 epochs, tk 0.01/rb 0.03) -- operator-ordered, no new decision needed. FAIL => NO 40M; name which bar failed and compare against gaitinit-hard1's final round (prog 0.60/falls 4-8) to separate freeze-helped from freeze-irrelevant. Additionally: confirm no second optimizer-reload crash (walkcurr/rollbacks fires without a traceback) as evidence the mechanical fix holds.

