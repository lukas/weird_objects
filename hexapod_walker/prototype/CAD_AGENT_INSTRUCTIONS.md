# CAD agent instructions — hexapod prototype

These rules apply to any LLM coding agent editing the parametric CAD
in `hexapod_walker/prototype/`.  Read them in full before touching
geometry; paste them into context for any helper / sub-agent you spawn
to make a change.

The pipeline they refer to is documented in `CAD_WORKFLOW.md`.

## Rules

1. **Before editing CAD, list every functional clearance and keep-out
   volume that the change might intrude into.**  The canonical
   inventory is `design_spec.yaml` (per-part `keep_out_volumes`,
   `wire_channels`, `clearances`) plus the `KEEP_OUT_VOLUMES`
   registry in `keepout_volumes.py`.  Skipping this step is how every
   recurring regression has gotten in; see the long comments in
   `_verify_prototype.py` for examples.

2. **Never add material inside a keep-out volume.**  This is a hard
   rule.  Common offenders:
   - `pad_sweep_clear` (a cylinder along Y inside the coxa link's
     pedestal + hub; the femur hip pad sweeps through it).
   - `yaw_horn_sweep` / `link_hub_horn_sweep` (cylinders above /
     below the horn-bolt circle; the plastic horn rotates inside
     them).
   - `*_servo_body` (rectangular pockets that hold each servo body —
     the body must drop in cleanly).
   - The L-shaped wire-exit corridor at the +X bottom-outboard
     corner of every cradle (see `_wire_exit_slot` in
     `hexapod_prototype.py`).

3. **Never remove or shrink a named hole or channel in
   `design_spec.yaml` unless the user explicitly requests it.**  Each
   entry in `parts.<name>.holes` and `parts.<name>.wire_channels`
   represents a functional invariant (mounting bolt, wire-exit path,
   bolt-circle hole) somebody downstream depends on.  If you think a
   hole is dead, ASK before deleting it.

4. **Every new support / gusset / pad must be checked against
   rotation and wire clearances.**  Anything that adds material near a
   joint axis can clip an actively rotating part (femur hip pad,
   plastic horn) or block a wire-exit corridor.  Run
   `make -C hexapod_walker/prototype check-cad` after the edit and
   look specifically at the `check_workspace_self_collision` and
   `check_wire_slot` lines.

5. **Prefer parametric dimensions over hardcoded magic numbers.**  If
   the change needs a new length / radius / clearance, add a constant
   to the constants block in `hexapod_prototype.py` (or pull from one
   that exists — there are over 80 already) instead of typing the
   literal value into the body of a `make_*` function.  Mirror the
   constant in `design_spec.yaml` with a `# = hexapod_prototype.<NAME>`
   comment so the spec tracks the change.

6. **After every geometry edit, run
   `make -C hexapod_walker/prototype check-cad` and fix failures
   before committing.**  The check writes
   `artifacts/cad_report.md` — the "Failure details" section is
   designed to be pasted back into an LLM as a diagnostic prompt.  Do
   NOT commit a CAD edit while any check is failing; if a check is
   wrong (e.g. you've intentionally rerouted a wire), update the
   spec / check first and commit the spec change together with the
   CAD change.

7. **If a requested change is ambiguous, encode the assumption in
   `design_spec.yaml`, not in a code comment.**  Example: the user
   asks "make the foot bigger".  Add (or update) the `foot_pad`
   entry's `bounds_mm` / `holes` to capture exactly how much bigger,
   in which axes, with what hinge clearance; the validator can then
   detect drift on future runs.  A comment in `make_foot_pad()` does
   not survive a refactor; a `design_spec.yaml` entry does, and it
   shows up in the report's "Discovered dimensions" table.

## Workflow at a glance

```
$ make -C hexapod_walker/prototype check-cad-fast    # inner-loop ~30 s
$ make -C hexapod_walker/prototype check-cad         # full check ~2 min
$ open hexapod_walker/prototype/artifacts/cad_report.md
```

If a single failure dominates: drop into the matching
`_verify_prototype.check_*` function and read the long comments — they
typically explain the constants involved and the prior failure history.

## Useful entry points

- `hexapod_prototype.py` constants block (lines ~80 to ~700): every
  geometric constant + docstring on what depends on it.
- `_verify_prototype.py` (~2.5 k LOC) — every validity check.
- `keepout_volumes.py` — code-driven registry of keep-out / clearance
  meshes; the validator + the render overlays both consume it.
- `design_spec.yaml` — bounding boxes, named holes / channels /
  keep-outs, print orientations.

Happy designing.
