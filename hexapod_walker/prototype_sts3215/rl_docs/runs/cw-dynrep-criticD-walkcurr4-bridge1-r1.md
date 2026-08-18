# cw-dynrep-criticD-walkcurr4-bridge1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-18T11:25:41+00:00

**pod**: hexapod-mjx-train-11

**steps**: 2000000

**parent**: cw-dynrep-criticD-walkcurr4-bridge1

**hardware_ready**: False

**hypothesis**: Finish the interrupted bridge walking exam from its saved two-million-step brain instead of starting over: the crashed attempt was walking well on command (progress 0.93 of commanded, healthy height, low slip) when a checkpoint-reload bug — now root-caused, fixed and unit-tested (20/20 green) — killed it exactly halfway; this run reloads that checkpoint's actor and completes the remaining 2M steps of the same curriculum. OPERATOR ORDER fb 20260818T111051Z executed: resume from the 2M checkpoint under an append-only -r1 name; do not restart from zero (checkpoint verified valid: zip CRC OK all members, md5 bf55d4f8244dc467a18a0b5f816ec423, num_timesteps 2,031,616, loads cleanly on train-11). Honest wiring note: the condition-D/walkcurr contract only supports --init-from-actor-only, so the 2M ACTOR resumes exactly while the critic head and curriculum state re-initialize fresh — the same recipe shape as bridge1 itself (actor-only transplant + 0.5M actor freeze while the fresh critic adapts + fail-closed pre-PPO B0 cert at prog>=0.5, which also guards against the checkpoint carrying the KL-breached update the fatal rollback was trying to undo). Everything else identical per the order: all-GPU Warp/MJX, V3 bridge curriculum, frozen critic-D transformer md5 9df48f687967c25085ee50171e4110ff, 5e-5 x3 epochs, target_kl 0.01, KL-rollback 0.03, seed 8. The rollback fix (load_optimizer_state_if_compatible group-count gate) is live in this code, so the promotion+rollback event that killed the parent now completes instead of crashing. Prediction-if-true: B0 re-certifies during the freeze window and the curriculum reaches frontier>=B2 with parent-quality slip/height by the 2M end. Prediction-if-false: the resumed actor re-walks B0 but stalls before B2 again — which, combined with the from-zero twin retry1's outcome, separates a resume artifact from a real recipe wall.

**gate**: PRE-PPO (in-run, fail-closed): walkcurr/pre_b0_* logged with falls==0 and cmd_prog_frac>=0.5 on the resumed 2M actor. BEHAVIORAL GATE at r1's 2M end (= lineage total 4M): (1) B0 re-promotion by <=1M of r1 (the lineage's original B0<=1M bar stands MET by bridge1 itself at 524,288 steps); (2) frontier>=B2 at end; (3) final cert round on every certified bucket: cmd_prog_frac>=0.60, height_factor>=0.80, slip_per_m<=2.0, falls==0; (4) mechanical: any walkcurr rollback completes without an optimizer-reload traceback (walkcurr/rollbacks>=1 with no crash is positive evidence the fix holds). PASS => completes the operator's full-4M contract: the triaging cycle AUTOMATICALLY launches the pre-registered 40M successor cw-dynrep-criticD-walkcurr4 (identical recipe: actor-only hard1 transplant, critic-D md5 9df48f687967c25085ee50171e4110ff, V3 bridge curriculum, 0.5M actor freeze, 5e-5 x3 epochs, tk 0.01/rb 0.03). SINGLE-AUTHORITY CLAUSE: the 40M auto-launch belongs to THIS run's triage alone; the from-zero twin bridge1-retry1 is corroborating evidence only (its gate was amended accordingly) — never double-launch, the launcher's duplicate refusal is the backstop. FAIL => NO 40M; name the failed bar and compare against bridge1's death telemetry (B1 cert prog .925/height .842/slip 1.31/falls 1) and retry1's outcome to separate resume-artifact from recipe wall.

**verdict**: Fail-closed at the pre-PPO precert (as designed): the RESUMED 2M actor-only transplant (the checkpoint that was mid-KL-rollback when the parent crashed) scores prog=1.15/hf=0.82/slip=1.31 but falls 38% of B0 episodes (bar: falls==0) — never trained a single PPO step. This separates a resume artifact from a real recipe wall: retry1 (same recipe, from-zero hard1 transplant) PASSED this exact precert (falls=0). Resuming a checkpoint that was itself an unstable in-flight KL-breach is not equivalent to a clean transplant.

**failed_reason**: run never appeared as 'running' in W&B within 240s

