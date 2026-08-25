# cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-canary1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T00:49:43+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-arch-tf64-joyfullcurr13-v7-hz100-canary2

**wandb_id**: rgsmea0q

**hypothesis**: Plain English: does the proven transformer walking recipe still train healthily when the simulated robot is replaced by the corrected, heavier as-built model (mesh family, 3.50 kg, true +38mm hip-pitch anchor, +4mm boots on legs 0/4) instead of the legacy 2.10 kg primitive model? OPERATOR ORDER (08-25 focus note): clean FROM-SCRATCH restart of the cw-arch-tf64-joyfullcurr13-v7-hz100 lineage from its beginning -- the exact 2L/d128/8h/ff256 causal-transformer-over-hist64 canary recipe that acq1 continued -- on current main with env.model_source=mesh (pods auto-load the checked-in primitive-collision twin mesh_mujoco/hexapod_mesh_mjx.xml; same kinematics/masses/inertia, MJX-ready). This is a NEW mesh-family lineage: NO warm-start from any pre-2026-08-24 checkpoint (those are primitive-family; the families do not transfer). 2M canary answers mechanism health only: boots on the mesh twin under warp, trains stably at usable CUDA fps, no NaN/SIGBUS. Prediction-if-true: reaches 2M crash-free with finite diagnostics; reward will sit deep in the from-scratch 100Hz valley (primitive-family control was ~-739 at 2M) and its level/sign is NOT a health signal on a new dynamics family with no matched control yet. Prediction-if-false: crash/NaN/degenerate contacts from the mesh twin under warp. Strongest alternative: the +66% mass deepens/lengthens the valley without being unhealthy -- only resolvable at acquisition against the MLP-mesh matched-step control launched alongside (cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-canary1).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (08-24 valley ruling: from-scratch 100Hz canaries are never judged on reward level/direction at 2M, and NO mesh-family matched-step control trajectory exists yet -- the sibling cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-canary1 is being established as that control). PASS = trains to 2M with no crash/NaN/SIGBUS, healthy CUDA fps (same order as the ~5000 fps primitive tf64 runs), finite reward/diagnostics. If PASS and the MLP-mesh sibling canary is also healthy -> respec BOTH to 40M from-scratch acquisitions, tf64-mesh judged against the MLP-mesh matched-step trajectory (the exact acq1 gate structure), with explicit per-leg duty tracking for the legs-3/5 sacrifice fingerprint on the mesh model (which also models the real +4mm boots on legs 0/4). If FAIL by crash/instability -> audit mesh_mjx contacts/warp before any further mesh-family budget. NO gait/skill judgment and no reward-class closure at 2M.

