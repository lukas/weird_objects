# cw-walk-step0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T23:56:30+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: none (fresh init — operator-directed exception to warm-start default)

**wandb_id**: wfcg6ues

**hypothesis**: Operator queue item 0 (binding): walk-only from-scratch DR0 baseline with step-event reward package — park priced by construction (k_park_duty -0.6/tick for a standing park, test-measured), drag-while-loaded charged, completed lift->swing->touchdown paid per leg. Audited exploration std 1.0 / ent 0.01 / target_kl 0.02. If-true: reward_step_event rises from zero and harness shows >=10cm forward, six legs cycling, duty [0.2,0.9], >=2 swings/leg, det+sto. If-false: park/shuffle persists with duty charge absorbed as constant — refutes pricing-suffices-from-scratch, promotes history-8 on this exact reward. Strongest alt: exploration never completes one step so the credit stays invisible (reward_step_event pinned at exactly 0). Probe probe-walk-step0 PASSED. Snapshot f117fc5.

**gate**: DR 0, det AND sto: from normal stance move forward >=10 cm with ALL SIX legs repeatedly cycling lift/swing/touchdown; per-leg duty ~[0.2,0.9]; >=2 swings/leg; no drag, no parked leg; video verdict pathology-first

**verdict**: LAUNCH ABORTED (cycle 14): launcher crashed passing unquoted --notes to remote shell (>= parsed as redirect); no process ever started on pod (verified /proc scan + no log file). Relaunched successfully same cycle (see RUNNING entry, wandb wfcg6ues).

