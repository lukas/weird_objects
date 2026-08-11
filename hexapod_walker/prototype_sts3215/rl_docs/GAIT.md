# GAIT.md — kill the paddle: walking that lifts its feet

Operator directive 2026-08-11 ~13:05. Owner: agent cycles; operator
reviews verdicts. Status of each item lives in the ledger/RL_LOG, not
here.

## The problem

Every walking champion travels by PADDLING: feet stay loaded and
skate across the floor (slip/m 1.1–1.5 in its own band; the
hist16-dep1 eval degenerate hits 7–11). On hardware this scrapes the
leg tips against the ground and the robot can catch/stall. We want a
gait that LIFTS and PLACES feet — swing legs clear the ground, stance
feet do not translate while loaded.

## What we know (don't re-derive)

- **Pricing already prefers stepping.** probe_walk_income.py (08-11):
  an honest stepping gait out-earns the paddle 2–4x under the
  champion stack, all four directions, DR 0 and 0.5. The paddle is
  not a paid basin — it is a strong LOCAL optimum PPO finds first and
  never leaves.
- **Sim slip is not free.** calibrate_slip.py (08-10): sim is
  slightly conservative vs the real floor (travel ratio 0.35–0.41 sim
  vs 0.50–0.51 real, speed-invariant). This is not a sim exploit.
- **Anti-slip income shaping is CLOSED (10+ arms).** Gates/shaping
  retrofitted onto a trained paddler either starve income (policy
  parks and earns nothing — the park-and-earn failure) or get farmed.
  That closure is about INCOME SHAPING ON A FORMED HABIT. It does not
  close the two things below (operator 08-11): a structural drag
  CHARGE banked from scratch, and CURRICULUM that changes the task.
- **The scripted tripod gait steps.** It walks the real robot
  (tape-proven), and bc_anchor.py already emits it as a
  command-conditioned per-tick target on walk ticks
  (`cw-omni-transbc1` built the machinery; that arm failed on
  from-scratch omni TRANSLATION, which rot-60 later solved — the
  anchor itself converged cleanly. It was never tried for GAIT
  CLEANUP on a policy that already travels).

## Priority 1 — BC-anchor gait cleanup (launch-ready, zero code)

`cw-walk-gaitbc1`: warm-start THE hardware walker (`cw-dep-vref1-r1`)
with `train.bc_anchor_coef=1.0` on walk ticks (existing TripodGait
target), discovery 2M, everything else byte-identical to the parent's
recipe. Rationale: rise and hold both proved the anchor breaks
entrenched habits that pricing cannot; here the skill (traveling)
already exists and only the HABIT (dragging) needs breaking, which is
the anchor's demonstrated strength.

Gate: slip/m must drop DECISIVELY below the parent band (target
<0.6 at DR0 and own-DR vs parent 1.1–1.5) while keeping the
joystick gate (zero falls, fwd distance, vel-err in band) and
gait_valid. Kill signature: fwd distance collapses toward the
scripted gait's slower band with no slip win, or park.
Follow-up if PASS: 10M hardening + tape-replay style hardware check
before any promotion talk.

## Priority 2 — structural stance-slip / swing-clearance terms
   [CODE, bank first]

Two structural terms (HARDWARE.md queued them long ago), landed
behind cfg flags with an MDP_PREFLIGHT bank BEFORE any launch:

- **Stance-slip charge**: foot-XY translation WHILE IN CONTACT,
  charged per tick (this is literally the scrape). A charge, not an
  income gate — gates are the closed move.
- **Swing-clearance term**: swing feet must clear >= X mm at
  mid-swing (pay tiny clearance income or charge sub-clearance
  swings; pick in SPECIFICATION).

Bank requirements (scripted fingerprints, full champion stack):
step-gait >> drag-gait, AND drag-gait > freeze/park (the penalty must
never make parking the optimum — the exact failure of the closed
arms), AND the honest gait's own small placement slip is not
bankrupted (tape-proven gait must stay net-positive).

## Priority 3 — LEARN it, no anchor (operator: the goal state)

Operator 08-11: "if we really gave dragging on the ground a big
penalty, it should eventually learn how to move — maybe we need to
make the task initially easier in some other way." The BC anchor is a
crutch; this line exists to retire it. Design principles: dragging
expensive AND the task easy enough early that clean travel is
DISCOVERABLE before the charge matters. From scratch (no paddle habit
to break), one easing lever at a time:

1. **Terrain-as-teacher (first arm — physics does the pricing).**
   Train ON bumpy ground (existing `env.terrain_amp` hfield, bumps
   ~20–36 mm). On bumps a dragging foot physically catches: paddling
   stops traveling, so plain progress income selects stepping with NO
   reward surgery. Anneal terrain toward flat late (or test transfer
   to flat directly).
   ZERO-TRAINING PROBE FIRST (cheap, decisive): eval the current
   champion on terrain_amp>0 and read slip/m. Champion already
   survives 36 mm bumps — if its slip DROPS there, physics already
   forces stepping and the premise is confirmed; if it paddles over
   bumps at the same slip, this lever is refuted and we skip the arm.
2. **Drag charge annealed UP** (with Priority-2 machinery): charge
   starts near zero (travel discoverable, sloppy is fine), ramps to
   full by mid-run so the FINAL optimum cannot include dragging.
   Anneal-up avoids both failure modes of the closed arms: no
   retrofit onto a formed habit, no early starvation.
3. **Physics easing early, annealed to nominal**: servo velocity
   ceiling up (lifting is cheap while learning) and/or slight gravity
   reduction during discovery — standard dynamics-curriculum practice
   in legged RL. Anneal to measured params before any gate counts.
4. **RSI-for-walk (state crutch, NOT action supervision)**: spawn
   walk episodes mid-stride of the scripted tripod gait (machinery
   exists from the rise work, `goal.rise_rsi_frac` pattern +
   pool-restore lesson) so lifted-leg states are experienced from
   step 0.
5. **Slow-speed-first commands** if 1–4 underdeliver (note: the old
   speed-band closure was about chasing speed TRACKING, not gait
   curriculum — distinct).

Success = a from-scratch, no-anchor policy whose slip/m beats the
BC-anchored one at matched travel, surviving DR hardening. If the
line stalls after levers 1–3 are honestly tried, the anchor stays
and this doc records why.

## Order of operations

P1 launches now (zero code). P2 spec+bank next (its charge is also
P3's lever 2). P3 starts with the zero-training terrain probe, then
one arm per lever. Every arm: one variable, matched-parent control,
pre-registered kill signature, slip/m + gait_valid + joystick gate
as the panel.
