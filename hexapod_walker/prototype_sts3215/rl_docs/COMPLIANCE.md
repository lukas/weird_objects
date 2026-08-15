# Structural compliance — measurement protocol + sim hook

Status: HOOK READY, MEASUREMENTS PENDING (hardware down ~2 days).
Source directive: operator RISE_WALK_NEXT_48H (08-13), P3 "Prepare
structural-compliance modeling". Companion code:
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
(quasi-static: deflection = tau / k). **No stiffness values are
invented**: `struct_comp.enabled=1` hard-fails until the measured
`k_yaw/hip/knee_nm_rad` are entered in `rl_move/config.yaml`.

## What the operator should measure (once the robot is repaired)

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
      dr_scale_lo: <e.g. 0.7>   # DR band around the fit; set from the
      dr_scale_hi: <e.g. 1.5>   # measurement spread + hysteresis width

## Wiring (follow-up commit; call sites are hot right now)

Two integration points, both documented in the module docstring:

- `sim_env._read_state`: `q_reported = comp.reported_q(q_phys, tau)`
  with `tau = data.qfrc_actuator[dof_adr]` — obs/encoder side.
- model prep after DR kp_scale: `comp.apply_effective_kp(model, rows)`
  — physics side ("equivalent compliant mount").
- per-episode DR: `comp.sample(rng)` next to the other `_ep_rand`
  draws; the stiffness vector must ride `SNAP_ATTRS` (pool-restore
  lesson) once wired.

Deliberately NOT wired in this change: `sim_env.py` is mid-flight with
the mode-seq canonical-frame fix (08-13/14); the hook is inert,
tested, and 3 lines per call site once that lands.
