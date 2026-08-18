# cw-dynrep-criticD-walkcurr4-bridge2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-18T11:52:26+00:00

**pod**: hexapod-mjx-train-11

**steps**: 4000000

**parent**: cw-dynrep-criticD-walkcurr4-bridge1-retry1

**wandb_id**: leuz9fcr

**hardware_ready**: False

**hypothesis**: Stop the robot's walking teacher from erasing its own progress-judge every time it protects the walk: until now, when the training curriculum rolled back a regression it restored the WHOLE brain — including the freshly-adapting value critic — so the critic kept getting reset to an old state (its accuracy went from +.30 to -.18 across three rollbacks in the last run) and the walker never consolidated; this arm rolls back ONLY the walking actor, releases the actor from its warm-up freeze only once the critic is demonstrably accurate, and restores the champion's own anti-drag/anti-park reward terms that this recipe had dropped. Operator-ordered bridge2 canary (fb_20260818T112826_9ed832) after bridge1-retry1 solved locomotion geometry (frontier B2, prog/hf/slip inside every bar) but failed ONLY safety/retention (falls .125/.375/.50 escalating with rung). Three coupled changes, per the spec (multi-variable bundle operator-ordered): (1) --walkcurr-actor-only-rollback — retention rollback restores actor trunk/action head/log_std only, never critic/value head/predictive residual/frozen encoder; critic optimizer moments preserved, actor moments reset; critic fingerprint asserted unchanged + walkcurr/rollback_critic_ev logged; (2) --actor-freeze-ev-threshold 0.2 / windows 3 / max 2M — the 0.5M actor freeze now releases only when train/explained_variance holds >=0.2 for 3 consecutive updates, fail-closed abort at 2M if the critic never converges, and the release lr is <=1e-5 x3 epochs (was 5e-5); (3) hard1's gait-retention reward stack restored during the V3 bridge curriculum: reward.walk_anchor_gate=1, anchor_tol_mm=10, k_drag_loaded=10, k_park_duty=1 (existing proven keys, zero-fall lineage). Identical otherwise: all-GPU Warp/MJX, actor-only hard1 transplant, frozen critic-D transformer md5 9df48f687967c25085ee50171e4110ff, V3 buckets, tk 0.01/rb 0.03, seed 8. Pre-PPO det certs now run on B0, B1 AND B2 (b0 keeps the abort bar). Prediction-if-true: falls stop escalating with rung (the retention terms price the parked/dragging compromises, the stable critic stops mis-valuing recoveries) and the 4M gate passes with falls==0. Prediction-if-false: falls persist even with a never-reset critic + retention terms — pointing past rollback mechanics at the V3 ladder's DR/heading rungs themselves.

**gate**: PRE-PPO (fail-closed): pre_b0 falls==0 and cmd_prog_frac>=0.5 (b1/b2 certs logged informational). IN-RUN fail-closed: readiness abort if critic EV never holds >=0.2x3 windows by 2M. BEHAVIORAL GATE at 4M (operator's exact bars, fb_20260818T112826_9ed832 item 4): frontier>=B2; B0-B2 final certs cmd_prog_frac>=0.6, height_factor>=0.8, slip_per_m<=2.0, falls==0; plus MECHANISM proof: no critic reset across any rollback (walkcurr/rollback_critic_unchanged==1 on every rollback event, critic EV does not saw-tooth to negative after rollbacks). PASS => auto-launch the pre-registered 40M successor cw-dynrep-criticD-walkcurr4 with THIS recipe (operator-ordered, no new decision). FAIL => NO 40M; name the failed bar and whether falls still escalate with rung vs retry1's .125/.375/.50 signature to separate rollback-mechanics from curriculum-ladder causes.

**verdict**: FAIL — the run correctly self-aborted at its own pre-registered IN-RUN fail-closed gate at 2.007M/4M steps: critic explained-variance never held >=0.2 for 3 consecutive updates within the 2M actor-freeze cap (final EV=-0.345, streak=0), so the actor was never unfrozen. A NEW failure mode vs retry1 (falls escalating 12.5%->37.5%->50% with curriculum depth): even with the critic protected from any rollback reset (actor-only rollback) and hard1's anti-drag/anti-park terms restored, the fresh condition-D critic (frozen predictive-transformer encoder) never learned a usable value function in 2M steps. Per the single-authority gate: NO 40M. This was the operator's own bridge2 correction (fb_20260818T112826_9ed832, ratified 20260818T114821Z) and it also failed, one layer deeper than retry1's (critic competence, not curriculum falls). The walkcurr4 tournament (bridge1-r1 precert-refused, retry1 falls-FAIL, bridge2 critic-EV-FAIL = 3 corrections) has spent ~11 arms today without beating its own parent cw-dep-bcgait1-hard1; SIM-SPRINT fit stays an open operator question (q_20260818T1035Z/1040Z/1103Z/1125Z). No bridge3 without an explicit new operator order.

