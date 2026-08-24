# cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T13:20:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**wandb_id**: ffhhv474

**hypothesis**: Plain English: the robot got worse at not-falling because we forced its stop-hold during TRAINING, which corrupts the learning data (the action it proposes on frozen ticks is never executed, so it can drift unpunished and then jerk at resume); this run keeps the hold ONLY in the certification assay -- where it legitimately unlocked the never-practiced side/rear buckets -- and trains on clean on-policy data. Warm-start from stopcur2 (the clean 1/48-joygate parent), identical reward cfg (k_walk_stop_current=2.0), goal.walk_stop_freeze_s=0.0 in training, new --walkcurr-cert-cfg-set goal.walk_stop_freeze_s=0.4 applied to the cert/precert env only (default-off flag, 24/24 tests, tag exp/cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze). freeze40's dig-in 2x2 (08-24): eval-time freeze on parent weights costs ~1 fall (1/48->2/48) while training-time freeze cost the WEIGHTS 1/48->4/48 (freeze-off eval; same det episodes fall either way) and 7/48 combined -- so removing it from training should keep the promotion win without the safety regression.

**gate**: walkcurr/frontier promotes past b1 (>=b2) at/near init via the freeze-assisted cert (precert b1 PASS expected at step 0) and b2+ get real practice; held-out 60s joygate stays in the parent band: falls <=2/48 (parent 1/48, freeze40 7/48), dir_err med dr0 <=~36deg (parent 33.9), slip med <=2.9, no new leg-sacrifice signature. NOTE for triage: the joygate prestage forwards training cfg, so it will correctly eval freeze-OFF. If-true: on-policy corruption confirmed as freeze40's damage mechanism; cert-only freeze becomes the standard ladder assist. If-false (joygate still regresses to ~4+/48 with training freeze off): the damage is the b2+ practice diet itself trading off stress-mix robustness -- next lever is mixing joygate-style stress_mix commands into bucket training, not freeze mechanics.

**verdict**: FAIL, exactly the gate's own pre-registered if-false branch. Plain
English: making the freeze cert-time-only (training freeze OFF,
cert-env freeze ON) does NOT rescue the joygate -- the b2+ heading-
widening practice diet itself is the damage source, not freeze
mechanics.

Evidence:
- Frontier: promoted b0->b5 again (walkcurr/frontier climbed cleanly),
  so the cert-only-freeze DOES unstick the ladder as designed.
- Held-out 60s joygate: falls 6/48 (dr0 4/24, dr0p5 2/24) vs parent
  stopcur2's 1/48 and the gate's own cap of <=2/48 -- lands in the
  "if it STILL regresses ~4+/48" branch almost exactly. All 6 falls
  are over_current (dr0 det[3,11], dr0 sto[0,9], dr0p5 sto[2,7]) --
  the SAME dominant signature as freeze40 (7/48) and freeze40-stopcur6
  (6/48), now reproduced a THIRD time with training-time freeze fully
  OFF. dir_err dr0 34.8deg (near parent's ~34, inside the ~36 bar) but
  dr0p5 50.26deg (bad); slip/m 2.21 (ok, cap 2.9).
- NEW finding this run's own DR-0/own-DR gate (not just the joygate)
  also regressed hard, and shows why: det gait_valid crashed 6/6
  (stopcur2 parent, clean) -> 3/6, with leg-3 SACRIFICED in 3 of 6 det
  episodes (progress_ratio ~0.53 vs ~1.0-1.6 on the clean episodes) --
  a leg-3 lock reappearing at DR-0 from a lineage (stopcur2) that was
  previously clean at DR-0. Own-DR(0.5) det also down to 1/6 pass,
  gait_valid 2/6, leg 3 again. sto gait_valid 4/6, one episode
  (sto/4) sacrifices legs [2,4] with slip/m 42 (near-total thrash).
  This means the b2+ side/rear/full-circle heading-widening curriculum
  can INDUCE the leg-3 lock fresh from a clean k=2.0 base, independent
  of the current-charge dose that the earlier stopcur6 story blamed --
  a genuinely new mechanism candidate (heading-widening itself, not
  just current pricing), flagged for dig-in below.

Net: 3/3 independent arms now show the same over_current joygate
regression regardless of freeze mechanics (full training freeze,
cert-only freeze) or current-charge dose (k=2.0, k=6.0) -- decisively
points at the b2+ command-distribution diet itself, matching the
gate's own pre-registered explanation. Champion unchanged (stotight45-
seed13); joystick DONE gate stays met per 08-23 via that champion,
this V6/V7 ladder remains operator-ordered hardening
(fb_20260823T220651_5c66e3).

REPAIR BUILT + LANDED this cycle (tag exp/cw-arch-hist16-dep1-c1-
joyfullcurr12-certfreeze-v7, snapshot 8a4261ba): new
WALKCURR_BUCKETS_V7 (`--walk-curriculum-version 7`) -- byte-identical
ladder to V6 (same names/heading bands/durations/DR/gates) except
every non-bridge bucket now also draws in-place turning (wz, +-0.3
rad/s, 50% zero) and a 15% chance a resampled segment is a full
INSTANT REVERSAL of the current command -- the bucket-diet analogue of
the held-out joygate's stress_mix family (sweep_circle/square give
turning, flip_180 gives reversals), which the plain V6 sampler never
drew at all. V1-V6 tables lack the new keys so `.get(..., 0.0)`
defaults make this a true no-op for every existing lineage (unit-
tested: `test_v6_sampler_bit_exact_when_stress_fields_absent`). New
bank: `rl_move/tests/test_walk_curriculum.py` +6 tests (49 total, all
pass), `test_walkcurr_mjx.py` 19/19 unchanged.

Evidence: `logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr12_
certfreeze_{gate,owncfg,joygate}/`, W&B run ffhhv474.

