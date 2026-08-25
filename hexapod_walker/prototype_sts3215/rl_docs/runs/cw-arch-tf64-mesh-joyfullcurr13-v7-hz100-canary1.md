# cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-canary1

**What this run is trying to learn (plain English):** does the proven
transformer walking recipe still train healthily when the simulated robot
is the corrected, heavier as-built model (mesh family, 3.50 kg) instead of
the legacy 2.10 kg primitive model?

- **Operator order (08-25 focus note):** clean FROM-SCRATCH restart of the
  `cw-arch-tf64-joyfullcurr13-v7-hz100` lineage from its beginning — the
  exact 2L/d128/8h/ff256 causal-transformer-over-hist64 canary recipe that
  acq1 continued — on current main.
- **Model family:** `env.model_source=mesh` (explicit `--cfg-set`, visible
  in the ledger command). Pod log confirms: `[servo_model] ... using the
  checked-in primitive-collision twin hexapod_mesh_mjx.xml` (full mesh
  assets not generated on pods — expected). Corrected kinematics (+38 mm
  coxa hip-pitch anchor, 150 mm knee→foot), as-built 3.50 kg masses, real
  +4 mm boots on legs 0/4 modeled.
- **Lineage:** NEW mesh-family lineage. NO warm-start / `--init-from-source`
  — every pre-2026-08-24 checkpoint is primitive-family and does not
  transfer. Ledger `parent` = canary2 is config provenance only.
- **Phase/budget:** canary, 2M steps, train-6 (4Gi shm verified).
- **Judging rule (binding, 08-24):** from-scratch 100 Hz canaries are NOT
  judged on reward level/sign at 2M (architecture-independent valley,
  ~−739 @ 2M on primitive). No mesh-family control trajectory exists yet;
  the sibling `cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-canary1`
  (train-7, same cycle) is being established as that matched-step control.
- **If both canaries healthy:** respec both to 40M from-scratch
  acquisitions; tf64-mesh judged against the MLP-mesh matched-step
  trajectory (acq1 gate structure), with per-leg duty tracked for the
  legs-3/5 sacrifice fingerprint (3rd-lineage question) on the mesh model.
