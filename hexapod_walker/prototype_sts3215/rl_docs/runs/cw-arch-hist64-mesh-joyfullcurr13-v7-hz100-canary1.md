# cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-canary1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T00:53:35+00:00

**pod**: hexapod-mjx-train-7

**steps**: 2000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1

**wandb_id**: 97f06to7

**hypothesis**: Plain English: establish the healthy-MLP reference trajectory on the NEW as-built 3.50 kg mesh robot model, so the operator-ordered tf64-mesh lineage restart can be judged fairly. The 08-24 canary ruling is binding: from-scratch 100Hz runs pass through an architecture-independent reward valley (~-739 at 2M, zero-crossing ~12-14M on the primitive family) and must be judged against a MATCHED-STEP control trajectory, never absolute reward -- on the primitive family that control was this run's own source (cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1); NO such trajectory exists for the mesh family (corrected +38mm hip-pitch anchor, 3.50 kg as-built masses, real +4mm boots on legs 0/4). This clones the from-scratch hist64 MLP V7-certfreeze 100Hz recipe onto env.model_source=mesh at 2M canary budget, from scratch (no pre-08-24 checkpoint touched; families do not transfer). Secondary purpose: second-architecture per-leg-duty evidence for the open cross-lineage leg-sacrifice fingerprint question (primitive MLP sacrificed legs {0,2,5}; primitive tf64 sacrificed {3,5}) -- if the fingerprint is a property of the legacy sim model rather than architecture/diet, the mesh family's corrected kinematics and modeled boots should shift or remove it at acquisition. Prediction-if-true: boots and trains to 2M, no crash/NaN, valley-shaped finite reward. Prediction-if-false: crash/NaN/degenerate mesh-twin contacts under warp. Strongest alternative: mesh dynamics reshape the valley so primitive-era waypoints mislead -- which is precisely why this control run exists.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: PASS = trains to 2M with no crash/NaN/SIGBUS, healthy CUDA fps, finite reward/diagnostics; reward level/sign at 2M is explicitly NOT judged (08-24 valley ruling -- this run IS the future matched-step control for the mesh family). If PASS and the tf64-mesh sibling canary is also healthy -> respec to 40M from-scratch acquisition alongside cw-arch-tf64-mesh-joyfullcurr13-v7-hz100 acquisition as its matched-step control, and record its 2M/7M/12M reward waypoints as the mesh-family valley reference in the run doc. If FAIL by crash/instability -> mesh_mjx contact/warp audit before any further mesh-family budget. No gait/skill verdict and no reward-class closure at 2M.

**verdict**: CANARY PASS (gate text is explicit: no gait/skill judgment at 2M, this run IS the future matched-step control for the mesh family). Plain English: the from-scratch hist64 MLP clone (the required MLP-mesh matched-step control for the tf64-mesh transformer canary) boots and trains cleanly on the mesh-family model. Evidence: reached the full 2,015,232/2,000,000-step budget with 0 tracebacks/NaN/SIGBUS (train log clean, wandb state=finished), fps 4740 (healthy, in fact higher than its tf64 sibling's 3725 and the ~5000fps primitive baseline order), finite diagnostics throughout (reward quarters -45.5/-361.2/-503.3/-512.1, ep_rew_mean -558.2 -- nearly identical valley shape to the tf64-mesh sibling's -45.1/-340.5/-494.3/-471.0, as expected for two architectures on the same new dynamics). Why: this answers the gate's only question (does the mesh twin train stably under warp on the MLP too) YES, and gives the tf64-mesh canary its required matched-step control. What's next: BOTH mesh-family canaries are now healthy PASS -> respec both to 40M from-scratch acquisitions this cycle (the pre-registered next step), tf64-mesh judged against this MLP-mesh trajectory exactly as acq1 was judged against the primitive-family MLP control.

