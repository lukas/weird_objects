# cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-canary1

**What this run is trying to learn (plain English):** establish the
healthy-MLP reference trajectory on the new as-built 3.50 kg mesh robot
model, so the operator-ordered tf64-mesh lineage restart can be judged
fairly (08-24 valley ruling: matched-step control, never absolute reward).

- **Model family:** `env.model_source=mesh` (explicit `--cfg-set`). Pod log
  confirms the mesh_mjx twin loaded on train-7.
- **Recipe:** clone of `cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1`
  (from-scratch hist64 MLP, V7 certfreeze joystick recipe, control.hz=100),
  from scratch — no pre-08-24 primitive-family checkpoint touched.
- **Phase/budget:** canary, 2M steps, train-7 (4Gi shm verified).
- **Secondary purpose:** second-architecture per-leg-duty evidence for the
  cross-lineage leg-sacrifice fingerprint (primitive MLP sacrificed
  {0,2,5}; primitive tf64 sacrificed {3,5}). If the fingerprint is a
  property of the legacy sim model, the mesh family's corrected kinematics
  and modeled +4 mm boots (legs 0/4) should shift or remove it at
  acquisition.
- **If PASS:** respec to 40M acquisition as the matched-step control for
  the tf64-mesh acquisition; record 2M/7M/12M reward waypoints as the
  mesh-family valley reference.
