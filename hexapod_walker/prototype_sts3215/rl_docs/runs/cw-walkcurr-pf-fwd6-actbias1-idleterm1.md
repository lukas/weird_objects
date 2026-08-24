# cw-walkcurr-pf-fwd6-actbias1-idleterm1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-24T02:45:06+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1

**hypothesis**: Plain English: does the qvel-based idle-termination mechanism (verified working end-to-end, but so far only tested on the raw-a=0 lineage where it likely just cut short the SAME zero-point settle transient actbias1 already fixed) finally unstick walking once stacked on TOP of actbias1's clean, non-collapsing, already-diagnosed static PARK-STAND (height stays ~15mm all episode, zero collapse, the exact absorbing state idle-terminate was designed for)? Single lever vs actbias1 (the last checkpoint before the park_duty-class dose chain, since that class is closed at every bank-legal dose): safety.walk_idle_terminate_s=3.0 / walk_idle_terminate_grace_s=3.0 / walk_idle_terminate_qvel_deg_s=2.0 / reward.walk_idle_terminate_penalty=3.0 (x0.02-scaled, matched to this stack's own term_penalty=24, same ratio as idleterm1/2). Prediction-if-true: env/walk_freeprog_score leaves the [-0.10,-0.05] dead band, episodes stop running the full 15s from the clean park-stand, det gate shows real stepping on video. Prediction-if-false (same clean park-stand recurs, now bounded by the termination but never escaped): idle-termination is the 10th-and-11th (with idleterm1/2) refuted mechanism on the CLEAN static-stand target specifically, and BC-kickstart (flagged to the operator, q_20260824T0233Z) becomes the only unexplored escalation left in the track's own pinned fork order.

**gate**: Rung-1 gate: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health check: env/walk_freeprog_score trend vs the dead band every prior rung-1 arm sat in; mean episode length should not sit flat at ~6s the whole run; env/height_err_mm should stay in actbias1's healthy ~15-22mm band (no collapse re-introduced); clip_fraction stays healthy.

