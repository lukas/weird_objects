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
   **REFUTED (08-11, `cw-dep-vref1-r1` on freshly-synced code —
   the amp>1 clamp fix, 434a6e0, must actually be on the eval pod;
   an unsynced free pod silently re-ran the OLD clamped code and
   returned bit-identical reports across amps, a false negative,
   caught + documented as COMMANDS.md gotcha 16):
   true amp 1.0 (18mm, the model's actual base bump) leaves slip/m
   at the champion's own band (det/sto med 1.07/1.17); amp 2.0 (36mm)
   makes it WORSE, not better (1.34/1.54); amp 3.0 (54mm) is worse
   again (1.43/1.85) AND unsafe — 6/6 det and 4/6 sto episodes
   terminate `over_current` (the dragging foot catches/strains
   against a bump it never lifts over). Slip never drops with
   amplitude — the champion pays MORE for dragging into bumps, it
   never discovers stepping. Lever 1 is refuted at every amplitude
   tried, well past the point of any therapeutic effect and into
   outright failure; skip the terrain-as-teacher training arm
   entirely (no `cw-gait-terrain2`) — `cw-gait-terrain1`'s INVALID
   verdict stands but its suggested successor is now superseded,
   not just re-run at a corrected amplitude. Surviving P3 levers:
   3 (physics easing), 4 (RSI-for-walk), 5 (slow-speed-first); P2's
   structural charge is still SPECIFICATION-gated (bank first).
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

## Session log 08-11 ~13:15-14:15 (operator) — first wave results

- **TERRAIN CLAMP BUG found+fixed (434a6e0):** servo_model.build_model
  clipped `env.terrain_amp` to [0,1], and HFIELD_MAX_Z is 18 mm — so
  EVERY historical terrain run/eval ran peak bumps <=18 mm regardless
  of the requested amp (docs said 36). Exposed when the champion probe
  returned bit-identical results at amp 1.5/2.0/3.0. Amps >1 now scale
  the hfield z-extent (data stays [0,1]); verified amp 4.0 = 72 mm.
- **Champion-on-terrain probe (post-fix, train-5
  logs/terrain_probe2):** amp 2.0 (36 mm): 6/6, prog 0.85, slip 1.31 —
  paddle degraded but passes. amp 4.0 (72 mm): 0/6, prog 0.80, slip
  1.46. amp 6.0 (108 mm): 0/6, prog 0.80, slip 1.48. The paddle FAILS
  THE WALK GATE at 72 mm — teaching ground confirmed; the flat spawn
  disk + fade-in is a built-in curriculum.
- `cw-gait-terrain1` INVALID (trained on the clamped 18 mm rung).
  Superseded by **`cw-gait-terrain2`** (train-3): from scratch at true
  72 mm, no reward surgery.
- **P1 `cw-walk-gaitbc1` finished, verdict pending** — training-time
  read is kill-signature-shaped: bc_anchor_loss converged 5.16->0.03
  but walk_loadslip_factor collapsed 1.0->0.06. Suspect: the anchor
  target (scripted tripod) is itself drifty-footed in sim (travel
  ratio 0.35-0.41). Await gate eval before ruling.
- **P3 lever 2 `cw-gait-dragstep1` FAILED** (agent triage, kill (b)):
  paddle formed anyway from scratch at k_drag_loaded=40 + step-event
  income; gate eval slip det 6.36. IMPORTANT CAVEAT before closing
  pricing forever: env/reward_drag never exceeded ~0.09/tick in
  magnitude even at k=40 (0.5 mm/tick deadband + per-mm scale keeps
  the term tiny vs ~1/tick progress income), and reward_step_event
  peaked at 0.03/tick. The "big penalty" was effectively small. A
  charge-magnitude AUDIT (what does k=40 actually cost a typical
  paddle tick, in fraction of income?) is the honest next step on
  this lever, not another blind coef rung.
- **P3 lever 1 `cw-gait-terrain2` FAILED** (operator gate-eval,
  logs/terrain2_gate on train-3): from scratch at TRUE 72 mm the
  policy learned the LEG-SACRIFICE drag degenerate — own-terrain det
  0/6, prog 0.25, slip/m 10.2, sacrificed legs [3,4]; flat retention
  identical. Ground that defeats the champion's paddle did not force
  stepping; PPO settled for 25% progress dragged over the bumps.
  Physics-as-teacher REFUTED standalone at this amp/budget.

## Where this leaves the no-anchor line (08-11 evening)

Four from-scratch discovery arms (omni trans1, gru-r3, dragstep1,
terrain2) all land on paddle/sacrifice no matter the ground or the
charge coef. Two readings: (1) the effective drag price has NEVER
been big — see the magnitude caveat above — so the operator's "really
big penalty" hypothesis is still UNTESTED, not refuted; (2) discovery
lacks lifted-leg state coverage, which RSI-for-walk (lever 4)
addresses without action supervision. Next arms in order: (a) the
charge-magnitude audit, then ONE from-scratch arm with the charge set
so a typical paddle tick costs 2-3x a progress tick (audit-derived k,
not a guess); (b) RSI-for-walk mid-stride spawns; (c) annealed-up
charge once the P2 bank lands. Lever 3 (physics easing) unstarted.
