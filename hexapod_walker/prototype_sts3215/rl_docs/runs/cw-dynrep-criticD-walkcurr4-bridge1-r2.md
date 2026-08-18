# cw-dynrep-criticD-walkcurr4-bridge1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-18T11:38:42+00:00

**pod**: hexapod-mjx-train-11

**steps**: 3500000

**parent**: cw-dynrep-criticD-walkcurr4-bridge1

**hypothesis**: Complete the interrupted rollback and finish the bridge walking exam from the last checkpoint the run itself had certified as good: bridge1 crashed mid-rollback (optimizer-reload bug, now fixed + unit-tested 20/20) while trying to restore exactly this retention-clean B1 promotion checkpoint (step 524,288, det-cert PASS at promotion, md5 9753e3aecc6a99e44a00e1907f22cebd, CRC OK) — this run manually completes that restore and trains the remaining ~3.5M of the 4M contract. OPERATOR ORDER fb 20260818T111051Z executed with its own validity clause: the order named the 2M checkpoint, but the fail-closed pre-PPO cert REFUSED it in r1 (det B0 falls 3/8, roll 9.6deg despite prog 1.146 — it carries the KL-breached update the fatal rollback was undoing), so per 'resume only if scientifically valid / do not restart from zero unless no valid checkpoint exists' the recovery falls back to the newest VALID checkpoint, which is this one. Recorded why in r1's ledger. Recipe identical per the order: all-GPU Warp/MJX, condition-D frozen critic transformer md5 9df48f687967c25085ee50171e4110ff, V3 bridge curriculum, actor-only init (wiring contract), 0.5M actor freeze while the fresh critic re-adapts, 5e-5 x3 epochs, target_kl 0.01, KL-rollback 0.03, seed 8; the rollback optimizer-reload fix is live so the crash class is closed. Prediction-if-true: B0 re-certifies in the first cert round, B1 promotes again, frontier>=B2 by the lineage-4M end with parent-quality slip/height. Prediction-if-false: the recipe re-breaches KL past B1 exactly as the parent did (now surviving the rollback instead of crashing), pointing at the post-promo update aggression rather than the mechanics — compare against the from-zero twin retry1's trajectory to separate seed-path luck from a recipe wall.

**gate**: PRE-PPO (in-run, fail-closed): walkcurr/pre_b0_* falls==0 and cmd_prog_frac>=0.5 on the restored promotion actor (it was cert-PASS at promotion; a FAIL here means an env/obs regression, not a policy one). BEHAVIORAL GATE at r2's 3.5M end (= lineage total ~4M): (1) B0 re-promotion by <=1M of r2 (the lineage's original B0<=1M bar stands MET by bridge1 at 524,288 steps); (2) frontier>=B2 at end; (3) final cert round on every certified bucket: cmd_prog_frac>=0.60, height_factor>=0.80, slip_per_m<=2.0, falls==0; (4) mechanical: any walkcurr rollback completes without an optimizer-reload traceback. PASS => completes the operator's full-4M contract: the triaging cycle AUTOMATICALLY launches the pre-registered 40M successor cw-dynrep-criticD-walkcurr4 (identical recipe: actor-only hard1 transplant, critic-D md5 9df48f687967c25085ee50171e4110ff, V3 bridge curriculum, 0.5M actor freeze, 5e-5 x3 epochs, tk 0.01/rb 0.03). SINGLE-AUTHORITY CLAUSE: the 40M auto-launch belongs to THIS run's triage alone — exactly one 40M ever launches (launcher duplicate-refusal is the backstop); the from-zero twin bridge1-retry1 and the precert-aborted r1 are corroborating evidence only. If retry1 PASSES its 4M gate but this run FAILS, do NOT launch the 40M — flag the discrepancy for the operator. FAIL => NO 40M; name the failed bar and compare against bridge1's death telemetry (B1 cert prog .925/height .842/slip 1.31/falls 1) and retry1.

