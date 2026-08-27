# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor7-detachtrunk-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-27T09:58:54+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix-s1

**wandb_id**: l4odj0pq

**hypothesis**: Seed1 half of the anchor7-detachtrunk joint 2-seed call (see seed0 for full hypothesis text). Same detach_trunk=1 lever on top of the identical anchor6b-logstdsplit-fix-s1 recipe (log_std split, this exact seed catastrophically froze walk despite wiring-confirmed-correct exploration noise) -- this is the seed that most needs rescuing if trunk-bleed is the true root cause.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate text as anchor7-detachtrunk (seed0): WIRING CHECK FIRST (detach_trunk active, near-zero anchor grad-norm on trunk params), then joint 2-seed call: FULL PASS = WALK-SURVIVES both seeds; PARTIAL = rescues only one seed; FAIL = anchor4-class catastrophe persists on either seed with wiring confirmed -> escalate to critic-level dig-in.

**verdict**: CANARY FAIL - MECHANISM: detaching the stance-anchor gradient from the shared trunk partially rescues THIS seed's total walk freeze, but not to a passing level, and the joint call with seed0 (which detach_trunk made WORSE) refutes trunk-bleed as the sole root cause. Evidence (report.json, DR-0 gate + own-DR owncfg, vs fix-s1 parent): fix-s1 (no detach) was a TOTAL catastrophe at DR-0 (gait_valid 0/6, sacrificed_legs 3-5 every episode, prog_ratio 0.004-0.009, fwd ~0.03m -- essentially static). detach_trunk (this run) improves that to a milder single-leg persistent drag (gait_valid 0/6 still, but sacrificed_legs=[0] or [1] only, prog_ratio 0.10-0.11, fwd 0.27-0.29m -- real continuous translation on the contact sheet, not a freeze) at DR-0, and own-DR(0.5) reaches gait_valid 5/6 (sacrificed_legs=[] in 5/6 episodes, prog_ratio 0.10-0.23, fwd 0.34-0.53m) -- the best walk result anywhere in this pair. Hold roughly unchanged vs fix-s1 (hold/sto terms 2/6 both). JOINT CALL (with seed0): detach_trunk does NOT deliver the pre-registered FULL PASS (neither seed reaches det gait_valid>=5/6 at DR-0) and does not cleanly match the anticipated PARTIAL shape either (one seed clean/one broken) -- instead BOTH seeds converge toward a similar milder 1-3-leg persistent-drag basin, one rescued from total freeze (this seed), one regressed from a clean pass (seed0). This is real signal that detach_trunk changes SOMETHING (a shared critic/value-function coupling or a seed-homogenizing effect), just not the hypothesized trunk-gradient-bleed mechanism in the simple additive way predicted. VERDICT: FAIL against the promotion bar on both seeds. NEXT: per this arm's own pre-registered escalation clause, dig into the shared critic/value head (not action-noise, not trunk-gradient-bleed -- both now refuted as sole causes) before funding another anchor-recipe variant; do not fund a further trunk-detach dose/variant arm on this evidence alone.

