# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor7-detachtrunk-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T09:58:54+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix-s1

**wandb_id**: l4odj0pq

**hypothesis**: Seed1 half of the anchor7-detachtrunk joint 2-seed call (see seed0 for full hypothesis text). Same detach_trunk=1 lever on top of the identical anchor6b-logstdsplit-fix-s1 recipe (log_std split, this exact seed catastrophically froze walk despite wiring-confirmed-correct exploration noise) -- this is the seed that most needs rescuing if trunk-bleed is the true root cause.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate text as anchor7-detachtrunk (seed0): WIRING CHECK FIRST (detach_trunk active, near-zero anchor grad-norm on trunk params), then joint 2-seed call: FULL PASS = WALK-SURVIVES both seeds; PARTIAL = rescues only one seed; FAIL = anchor4-class catastrophe persists on either seed with wiring confirmed -> escalate to critic-level dig-in.

