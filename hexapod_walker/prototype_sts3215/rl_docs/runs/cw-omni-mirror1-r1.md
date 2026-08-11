# cw-omni-mirror1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T23:08:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-omni-mirror1

**wandb_id**: 4bv1y07m

**hardware_ready**: False

**hypothesis**: THE omnidirectional run (operator, 08-10: one run that ends with a hexapod that walks in any commanded direction). Continues the cw-omni-mirror1 probe checkpoint (+40M, same recipe — the r7-c1/c3 continuation pattern) on the full omni stack: hist16-dep1 boot recipe, full-circle headings (uniform +-180deg incl. backward), turn-in-place 30pct, signed yaw income + drift charge, deployment contract (meas:=ref, 25deg tilt), k_current=0, and the mirror-symmetry regularizer (coef 1.0) that the probe proved healthy. The structural bet: the command-invariant left-yaw drift that swallowed every turn command across 8 reward-side arms is baked into asymmetric limb phasing; a policy regularized toward pi(mirror(obs))=mirror(pi(obs)) cannot host a fixed chirality, so turn commands finally act on a symmetric substrate — and full-circle command exposure teaches every heading, not just the +-45deg cone all prior lineages saw. If-true: eval_yaw turn tracking in BOTH directions + zero-command drift <0.05 rad/s + the joystick panel extended to backward/lateral — the deliverable policy. If-false (drift persists under an enforced-symmetric policy): the drift is NOT policy chirality — look at sim physics/contact asymmetry next, mirror line closes.

**gate**: Behavioral, at 40M vs the no-mirror same-recipe baseline cw-arch-hist16-dep1 under the IDENTICAL panel: (1) eval_yaw both signs turn |wz_err| med toward the 0.10 rad/s gate AND hold/zero-command |wz| med < 0.05 (the drift number; dep1-lineage sits ~0.09); (2) joystick gate 0 falls on the full direction panel INCLUDING backward and lateral legs at own DR and DR0.2; (3) gait_valid 6/6 det; (4) video: real swing/stance in forward, backward, lateral, and turn-in-place clips — no flag leg, no dragging, no belly shuffle; (5) train/mirror_sym_loss stays low (regularizer not overpowered late). Slip/tracking economy read against dep1 own-band per hist16 precedent (exposure gap tolerated, structural defect not.

**verdict**: STOP — reward/eval specification bug (known exploit: walk-mode park/freeze). Own-DR + DR0 harness vs frozen hist16-dep1 baseline: fwd med 0.68m->0.01m, gait_valid 6/6->3/6, slip med 1.48->3.85, prog med 1.10->0.22 (sto worse: 0.05, slip 17.6). Per-episode returns show freezing (gv False, fwd~0.004m) scoring ~1130 vs walking (gv True, fwd~0.02m) scoring 500-860 — standing still pays MORE than walking under this arms stack (walk_speed 0.05-0.06 m/s + k_yaw_still=50 + turn_in_place_frac 0.30 shrank walk-progress income below parking/height/alive terms). train/std climbed monotonically 0.39->1.69 over 40M (health alarm); ep_rew_mean peaked ~640 near 8-10M then fell to ~320-350 by 40M. Mirror-symmetry yaw-drift hypothesis is UNTESTED by this run (gait broke before any turn-tracking claim is meaningful) — not evidence against mirror-symmetry. Next: fix the walk-mode park attractor under the full omni+turn-still stack (income must beat parking by construction) and add a stall/freeze-vs-walk ordering check at this speed band + k_yaw_still to the OMNI bank before re-hardening.

