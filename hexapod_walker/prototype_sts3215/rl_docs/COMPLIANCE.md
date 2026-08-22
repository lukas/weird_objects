# Structural compliance — measurement protocol + sim hook

Status: WIRED IN, FIRST-PASS ESTIMATES ACTIVE.
Source directive: operator RISE_WALK_NEXT_48H (08-13), P3 "Prepare
structural-compliance modeling"; activated after the 2026-08-21 bench
geometry session. Companion code:
`rl_move/sim/struct_compliance.py` (+ unit tests
`rl_move/tests/test_struct_compliance.py`).

## Why

The physical robot shows visible load-dependent deformation of
structural parts while standing (printed brackets/tibias; BOM already
flags the printed tibia stand-in as softer than CF). MuJoCo assumes
rigid links, so on hardware

    actual leg pose = servo-reported pose + load-dependent deflection

and neither the policy obs nor the sim reward see the sag. The v1
model is a per-axis torsional spring in series with each servo
(quasi-static: deflection = tau / k).

Current `rl_move/config.yaml` first-pass values:

    struct_comp:
      enabled: 1
      k_yaw_nm_rad:  300.0
      k_hip_nm_rad:  180.0
      k_knee_nm_rad: 120.0
      dr_scale_lo: 0.5
      dr_scale_hi: 2.0

These are deliberately conservative estimates, not final measurements.
The DR band means `--dr-scale 0` uses the nominal values, while
`--dr-scale 1` samples each axis class over 0.5x..2.0x.

## What the operator should measure next

All of this is camera + ruler + the existing sysid runner; no new
sensing. Use the bench_blast camera workflow (`zero_check.jpg` style
frames) so every measurement has a photo.

1. **Unloaded geometry.** Robot on the stand (feet free), servos
   holding the plant pose. Photo front + side; record encoder
   positions (`GET /api/pose`). This is the zero-deflection reference.
2. **Geometry under known standing load.** Same commanded pose,
   robot standing on the floor (full ~X kg on 6 legs), then on 4 legs
   (lift middle pair via pose command) to change per-leg load. Photo
   from the same tripod positions; record encoder positions. The
   difference between VISUAL joint angles (photo) and ENCODER angles
   at the same command isolates structural deflection from servo
   tracking error.
3. **Angular/linear deflection per axis.** From the photos: chassis
   drop (mm) at known load (the sim stance sag note says ~15–25 mm —
   confirm and attribute), knee/hip angle delta (deg) between loaded
   and unloaded at identical encoder readings. If one axis dominates
   visually (expected: knee bracket), say so.
4. **Recovery after unloading.** Lift the robot back onto the stand,
   re-photo: does the geometry return exactly (elastic) or is there a
   set (plastic/backlash)? Elastic-only justifies the spring model.
5. **Symmetry.** Repeat the loaded photo for left vs right legs.
   Asymmetric sag => per-leg stiffness scales (the hook currently
   shares one scale per axis class; per-leg is a one-line extension).
6. **Hysteresis (if observable).** Load → unload → load cycle,
   photographing the same joint: a visible loop width (deg) bounds
   the damping/friction term; if invisible, skip damping in v1.
7. **Loaded servo speed (P2 item, same session).** Re-run the sysid
   `steps` protocol ON GROUND (`champion_stand_ground` protocol
   already exists in `sysid/`) and fit with
   `rl_move/sim/fit_loaded_actuator.py`: the directive explicitly
   warns not to assume unloaded servo speed equals achievable speed
   under load. The shipped `sim_model_sysid.json` is the AIR fit;
   `vel_max_deg_s` under load is the number the takeoff/feasibility
   analyses (probe_walk_income feasibility block, transition gate)
   depend on.

## Turning measurements into cfg

From (2)/(3): per axis, k = tau / deflection, with tau estimated from
the measured per-leg load (weight distribution from foot count) times
the moment arm at the measured pose — a spreadsheet-level fit, no new
code. Enter:

    struct_comp:
      enabled: 1
      k_yaw_nm_rad:  <fit>
      k_hip_nm_rad:  <fit>
      k_knee_nm_rad: <fit>
      dr_scale_lo: <e.g. 0.5>   # DR band around the fit; set from the
      dr_scale_hi: <e.g. 2.0>   # measurement spread + hysteresis width

## Wiring

Two integration points are active:

- `sim_env._read_state`: `q_reported = comp.reported_q(q_phys, tau)`
  with `tau = data.qfrc_actuator[dof_adr]` — obs/encoder side.
- model prep after DR kp_scale: `comp.apply_effective_kp(model, rows)`
  — physics side ("equivalent compliant mount").
- per-episode DR: `comp.sample(rng, scale=dr_scale)` stores
  `_struct_comp_k`, which rides `mjx_host.SNAP_ATTRS` so reset pools keep
  the same stiffness vector their model rows were minted with.
- MJX: `prepare_shared_model` applies nominal compliance, and
  `ModelDrScratch.rows_for` applies the episode stiffness before
  uploading `actuator_gainprm` / `actuator_biasprm` rows.
