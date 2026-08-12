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

## Priority 1 — BC-anchor gait cleanup — CLOSED 08-11 (froze instead
of stepping; see session log)

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

P1 CLOSED (froze, see session log). P2 spec+bank next (its charge is also
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
- **P1 `cw-walk-gaitbc1` FAILED (08-11, gate eval) — worst-case
  version of the pre-registered kill signature: total FREEZE, not
  partial.** Video (det+sto, DR0 + own-DR0.35, all 6 episodes each)
  shows an IDENTICAL static tripod-like pose every episode — 3 legs
  held aloft motionless, 3 planted, zero gait cycling — fwd travel
  ~0.00 m/15s vs the parent's 0.28-0.34 m, gait_valid 1/6 det.
  Training-time read confirmed the mechanism: bc_anchor_loss
  converged to ~0 while walk_loadslip_factor collapsed 1.0->0.06 —
  the anchor was satisfied by NOT MOVING (close enough to the
  moving reference at a single frozen instant), not by lifting feet.
  Unlike rise/hold (the anchor's two prior wins, both STATIONARY
  end-states where freeze-toward-reference IS success), walk's
  reference target is itself in continuous motion; anchoring to a
  moving target plus the existing income-gated walk reward found a
  degenerate joint optimum instead of reshaping the gait. Per its own
  pre-registered gate text this is not a coefficient-variant
  situation — P1 (BC-anchor gait cleanup) is CLOSED; move to P2
  (structural stance-slip charge + swing-clearance, bank first) or
  P3 lever 4 (RSI-for-walk).
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

- **08-11 18:1x (agent cycle) — independent re-check + lever 1's
  pre-registered retry.** Re-ran the zero-training champion probe
  myself (det+sto, matched seed, per-mode 6) across the FULL
  amplitude range at the fixed clamp: flat 1.02, 18mm 1.09, 36mm
  1.33, 54mm 1.50, 72mm 1.74 slip/m — a clean MONOTONIC INCREASE, not
  a drop, confirming probe2's finding independently and closing any
  doubt that a coarser probe grid missed a sweet spot. Also
  independently re-evaled `cw-gait-terrain2`'s own checkpoint
  (own-terrain det slip 8.58 med / sto 12.38; flat retention det 6.83
  / sto 10.78, gait_valid 2/6 det both, leg[3] sacrificed in 4/6
  episodes) — same qualitative fingerprint as the operator's pass
  (numbers differ slightly by eval seed draw, conclusion identical:
  FAIL, worse than the closed paddle band). Launched the run's own
  pre-registered single retry, **`cw-gait-terrain2-r1`** (env.
  terrain_amp=3.0, 54mm, VERIFIED RUNNING train-3) to close out lever
  1 with the two-miss rule before moving on — but per the ordering
  above, the CHARGE-MAGNITUDE AUDIT is still the sharper next lever
  regardless of how -r1 lands; do not let -r1 delay it.
- **08-11 ~18:2x — `cw-gait-terrain2-r1` FAILED, lever 1 (terrain-
  as-teacher) CLOSED for good (two-miss rule).** One rung down at
  54mm avoided terrain2's leg-sacrifice (gait_valid 6/6 det+sto, all
  six legs stay engaged, video-confirmed) but landed on neither
  pre-registered pass branch: own-terrain slip/m med 6.86 det / 8.90
  sto is 4-6x the closed paddle band (1.1-1.5), not the <0.6 win and
  not a match to the champion band; one det episode terminated
  over_current (a dragged foot straining against a bump).   Gentler
  terrain does not force stepping either — physics-as-teacher is
  refuted across the amplitude range tried, from scratch, twice.
  Do not requeue terrain-as-teacher at any amplitude. Next: the
  charge-magnitude audit (P3 lever 2 prerequisite, unstarted) or
  RSI-for-walk (lever 4).

## Charge-magnitude AUDIT — DONE 08-11 eve (operator session);
## per-tick charge form REFUTED, structural per-stance charge LANDED

`rl_move/sim/probe_drag_audit.py` (+ `logs/probe_drag_audit*.json`).
Measured per-foot loaded slip on the honest scripted gait vs the two
learned skaters (`longdist_r2`, `dep_vref1_r1`) under the trans1
stack, 375 ticks each:

- **The 0.5 mm/tick deadband was the hole**: 53–63% of the skaters'
  slip ticks ride UNDER it (slow constant slide), so at dragstep1's
  k=40 the effective charge was ~0.001–0.09/tick vs ~2–2.6/tick
  income. The "big penalty" never existed.
- **The per-tick FORM is unfixable, not just the coefficient**:
  per-tick slip medians overlap (gait 0.31 mm — touchdown scuff — vs
  skaters 0.40–0.47 mm). At ANY (k, deadband) that prices the skate,
  the honest gait pays ≥2.5x its own income too. This is why 10+
  coefficient arms failed; do not run another per-tick k rung.
- **Per-STANCE accumulated travel separates them 3.3x**: scripted
  gait median 2.9 mm/stance (95% of stances under 6 mm) vs skater
  median 9.8 mm (only ~25–30% under 6 mm). Stance duration is the
  discriminator the per-tick form throws away.
- **Contact-solver micro-jitter (~0.2 mm/tick) integrates** on long
  stances (a 2 s quiet stance accrues ~10 mm/foot of pure jitter), so
  the accumulator only counts ticks sliding >0.25 mm (floor; skater
  drag runs 0.4–0.5 mm/tick).

Landed in `walk_task.py` (default OFF, legacy bit-exact):
`reward.k_drag_stance` (charge per metre of over-allowance stance
travel), `reward.drag_stance_allow_mm` (default 6),
`reward.drag_stance_tick_floor_mm` (default 0.25). Accumulator resets
at touchdown, pays incrementally (a never-lifting foot cannot defer).
**Audit-derived operating point: k=8000, allow 6 mm, floor 0.25 mm**
— skaters pay ~2.5x their income, honest gait ~23%, motionless foot
~0. Launch gate added and PASSING
(`test_drag_stance_stack_prices_skating_below_stepping`: stepping >
zero-lift skate of the same gait by >50 return, gait > stall > park
survives, forward + crab).

Calibration side-note (same session): re-ran `calibrate_slip.py` full
mu sweep + loaded-servo point. Travel ratio saturates ~0.38–0.40 (air
fit) at mu ≥1.2 and the 08-10 loaded fit makes it WORSE (0.25–0.31)
— friction is NOT the knob for the sim-vs-tape travel gap, and sim is
PESSIMISTIC (slipperier than the real floor), which is conservative
in the right direction for anti-skate training. Contact-stiffness
class stays open as operator calibration; it does NOT block the
charge arm.

## P0 reward-accuracy diagnostic — DONE 08-11 late (idle-kick cycle);
## penalty-side suspect REFUTED, paddle is a sim-EFFECTIVENESS optimum

`probe_walk_income.py --stack vref1` (new stack = cw-dep-vref1-r1's
exact ledger cfg; artifacts `logs/probe_walk_income/vref1_p0_dr0.json`
/ `vref1_p0_dr035.json`, run on train-1 at code 29f3706): scripted
plant-height tape-proven gait vs THE champion checkpoint under the
champion's own reward, forward, 3 seeds.

- **Totals are near parity, not gross mispricing**: gait 656 vs ckpt
  739 at DR0 (−11%); gait 713 vs ckpt 661 at the champion's own
  DR 0.35 (+8%). Neither branch of "reprice-then-train" fires.
- **The operator's tilt/rocking suspect is REFUTED**: the scripted
  gait's k_gyro+k_roll+k_pitch cost is ≤1.5/ep combined (vs ~650
  totals) and ZERO episodes terminate at either DR. Repricing the
  rocking terms would change nothing.
- **The stack already pays tall**: plant-height policies collect the
  base stance kernel (reward_task ~271-319/ep vs the crouched
  champion's ~30) and the champion pays −139/ep base k_height for its
  crouch — a combined ~380/ep pro-tall margin, already bigger than
  dep-hgt1/hgt2 assumed.
- **Why the paddle still wins**: it genuinely TRACKS the command in
  sim (progress_ratio 1.06 vs the scripted gait's 0.35 — the
  calibrated conservative slip physics), collecting ~495/ep more
  walk+prog kernel income. That is real locomotion income, not a
  pricing hole. Corollary: any near-champion policy that lifts to
  plant height immediately loses walk income (its early honest steps
  realize less progress) long before stance/height income arrives —
  a local-optimum MOAT, which is why income shaping kept failing and
  why the per-STANCE structural charge (audit section above) and/or
  curriculum are the levers with teeth.

Same cycle, the moat got a direct measurement:
`cw-walk-dragstance1` (warm champion + the audit charge, discovery
2M) **FAILED its slip gate but proved the charge's safety properties**:
reward_drag_stance sat at −7/tick the entire run (never resolved) and
the policy NEITHER parked NOR restructured — full travel retained
(prog 1.00, gait_valid 6/6, zero falls), slip only edged 1.1–1.3 →
0.95–1.15. A static fine on the FORMED habit is closed (no k rung,
no longer budget on the warm retrofit). Remaining routes: from
scratch under the charge (`cw-gait-dragstance1-r1`, 40M, running —
another cycle's) and the anneal-up curriculum (P3 lever 2).

## Structural stance-slip charge, FROM SCRATCH — `cw-gait-dragstance1`
## FAILED 08-11 (agent triage) — parked, not stepping

Companion arm to `cw-walk-dragstance1` (that one warm-starts the
champion; this one trains the identical audit-derived charge
(k=8000/m, 6mm allowance, 0.25mm floor) from scratch on the trans1
stack, to see if the paddle basin is avoidable from step 0 when it's
priced out from the start). Pre-registered false branch was "parks,
or paddles while paying" — that is exactly what happened, worst case
first tried:

- `env/reward_drag_stance` engages immediately (step 19: -9/tick) and
  **never trends toward zero** — it sits at -6 to -9/tick for the
  whole run, i.e. the charge never gets resolved, only endured.
- `env/walk_loadslip_factor` collapses 0.62 -> ~0.05-0.08 by step 49
  and stays floored — the policy minimizes loaded-slip *exposure* by
  going nearly still, not by cleanly lifting and placing feet.
  `env/reward_step_event` (the stepping income) stays tiny (~0.015),
  i.e. it earns almost no stepping credit either.
  Meanwhile the reward-quarters trend gets MORE negative over
  training (-441 -> -1558 -> -2354 -> -2381) simply because it
  survives the full episode paying a constant charge rather than
  falling early — not a sign of a worsening gait, a sign of a
  stable stillness habit.
  - Harness (gate DR0 + own-DR0.35, det+sto, 6 eps each = 24 clips):
  forward_dist 0.002-0.02 m vs cmd_dist 0.5-0.7 m over a 15 s episode
  (progress_ratio ~0.00, success 0/6-1/6). ALL 24 videos show an
  IDENTICAL static, splayed pose held for the entire clip — no
  leg-cycling, no net translation. slip_per_m reads 7.3-18.7
  (nonsensical-looking, but that's the near-zero-travel denominator,
  not real sliding distance — the real read is duty_cycle 0.94-0.98
  with 5-7 tiny swings/leg over 15s, i.e. a shuffle, not a gait).
- **Verdict: STOP - known exploit (FREEZE/PARK), matches the run's
  own pre-registered false branch verbatim.** The structural
  per-stance charge, even at the audit-tuned operating point, is
  *exonerated as a solo fix* for the paddle: from scratch it does not
  make stepping worth discovering, it just makes standing-still-while-
  paying preferable to the alternatives it tried. **CROSS-TRACK
  INSIGHT: this is also nobc's queued "drag-charge magnitude audit"
  (STATUS.md Next item 1) — the same charge, same conclusion, applies
  to both tracks' gait-from-scratch line.** Next lever per GAIT.md
  P3: RSI-for-walk (mid-stride spawns) or the charge combined with
  income-shaping, not another coefficient rung. Await
  `cw-walk-dragstance1` (warm-start variant) separately — a policy
  that already knows how to travel may resolve the charge by cleaning
  up its gait instead of freezing, since freezing there costs the
  large pre-existing walk-progress income it would otherwise keep
  earning; this from-scratch result does not by itself predict that
  one's outcome.

## P3 lever 4 (RSI-for-walk) — FAILED 08-12, lever CLOSED

`cw-gait-rsi1`: same dragstance1 charge stack + `goal.walk_gait_start_frac
=0.5` (episodes spawn mid-stride in the scripted tripod gait instead of
always standing still). Gate 0/6 det+sto both DR0 and own-DR0.5,
prog_ratio 0.00, slip/m 6-18 (well above the 1.1-1.5 paddle band the
PASS branch needed); all 24 video clips show uniform marching-in-place
(no leg sacrificed — `gait_valid` 6/6 — but zero net floor travel).
Training curves show the IDENTICAL mechanism as dragstance1:
`env/walk_loadslip_factor` collapses 0.51→0.03-0.07 by step 49 and
stays floored, `env/reward_drag_stance` sits at -7..-8.5/tick
unresolved the whole run, `env/reward_step_event` stays ~0.013-0.019.
The mid-stride state injection gave the policy nothing to build on —
it drifts back to a minimal-motion habit within the first ~1% of
training regardless of start state. **RSI-for-walk (P3 lever 4) is
CLOSED for nobc's gait-from-scratch line**, matching hw's tall-rsi1
null exactly as pre-registered. **08-12: lever 5 (slow-speed-first,
`cw-gait-slowfirst1`, target speed 60% below the champion band) also
FAILS** — det fwd travel 0.00m every episode (prog_ratio ~0.00,
slip/m 7.31), sto is worse (slip/m 17.8, negative progress);
video-confirmed legs shuffle in place, zero net displacement, the
identical fingerprint regardless of how slow the command is. Every
nobc-legal from-scratch gait lever launchable WITHOUT new code
(2, 4, 5) is now closed on the SAME mechanism
(`env/walk_loadslip_factor` floors early and never resolves,
independent of pricing, state-injection, or target speed).
**Re-opened same cycle, still no new code:** lever 2 retried as a
warm-start curriculum instead of an in-run anneal —
`cw-gait-anneal1` (warm from `cw-gait-dragstep1`, a genuine
from-scratch RL-only paddler that already translates by skating,
with its refuted per-tick charge swapped for the audit-correct
structural per-stance charge that froze dragstance1 from a random
init). Hypothesis: an already-mobile prior gives the optimizer
something to reshape instead of nothing to build from. If this also
floors, every no-new-code form of lever 2 is closed too and the only
remaining options are a true in-run coefficient scheduler (CODE,
unqueued, spec first) or BC-INIT (the lever that broke the hw
track's tall-wall, `cw-dep-bcgait1`) — NOT available here, nobc bans
imitation losses of any kind by charter.

## TALL LADDER — height-ref rungs on the dep line (operator session 08-11 eve)

Operator directive: "make a deployable tall smooth walker." Mechanism:
`goal.walk_height_off_mm` as a warm-start ladder (the lowgait line's
proven trick, inverted to climb UP), NOT the hgt1 income gate (refuted
one-shot). All rungs 2M warm, full dep contract + tipped starts
retained; eval numbers are `eval/walk/*` end-of-episode.

| run | parent | ref | height_err_end | speed m/s | slip | note |
|---|---|---|---|---|---|---|
| cw-dep-tip1 (baseline) | — | 0 | 59.9 mm | 0.0388 | 1.65 | eval-ends ~−60 mm |
| cw-dep-tall30 | tip1 | −30 | **15.2 mm** | 0.0295 | 1.74 | + k_drag_stance 8000/6/0.25 |
| cw-dep-tall30h | tip1 | −30 | 17.5 mm | 0.0287 | 1.80 | isolation: NO charge |
| cw-dep-tall15 | tall30 | −15 | 29.0 mm | 0.0278 | 1.64 | STALL: body ≈ −44 mm |
| cw-dep-tall15-h1 (T1, 6M) | tall15 | −15 | 51-58 mm (eval med) | 0.0545 | 1.16-1.35 | FAIL: worse+flat, not step-bound |

Findings (three runs, one evening):

- **Height-as-commanded-ref works on the dep line**: 60→15 mm in one
  rung. The hgt1 gate failure was about the unreachable one-shot 50 mm
  jump, not about height being untrainable.
- **The structural drag charge is FREE on a warm walker** (tall30 vs
  tall30h isolation: identical speed/slip, marginally better WITH the
  charge) — it rides along in the ladder but did not cut slip either
  (1.74 vs baseline 1.65). Consistent with cw-walk-dragstance1:
  absorbed, not resolved.
- **The free climb ends at ~−44 mm**: rung 2 (ref −15) left the body
  at the same ~−44 mm as rung 1 did. ~16 mm taller than baseline,
  ~44 mm short of plant height. Speed cost so far ~25-28%, flat
  across rungs (one-time, not per-rung).

Follow-up campaign T1-T5 pre-registered in RL_PLAN.md queue -0.5
(P2.5): budget rerun, k_height crank, gate-at-reachable-ref, speed
trade, and the −44 mm workspace probe that decides whether the wall
is kinematic (stop) or habitual (keep pushing).

**T1 (budget, 08-11) FAIL — the wall is not step-bound, and 3x budget
made it slightly worse.** `cw-dep-tall15-h1` (6M, identical respec of
the stalled ref −15 rung). `env/height_err_mm` (training-env metric)
plateaus by ~1M steps (~36-39mm) and sits flat the remaining 5M —
not the monotonic-improvement shape the PARTIAL branch needed.
Harness eval end-state is worse than the 2M parent's own 29mm: gate
median 51-58mm, own-DR median 57-58mm, close to the tip1 ref-0
baseline (59.9mm) despite 6M steps at ref −15. Speed improved
(0.0278→0.0545 m/s, now mid-band) but execution got noisier: one
gate/det episode spins in place (fwd 0.20m, slip/m 2.66) and
own-DR/sto gait_valid dropped to 4/6 (two episodes sacrifice 3 legs,
slip/m 2.6-5.1). No park/flag-leg/falls in any of the 24
video-checked episodes (terms 0, safety_flags 0) — still the honest
paddle gait, just at the old crouch depth, now with a bit more
instability at the height-vs-speed tradeoff. Verdict: budget is not
the lever; do not schedule further step-count variants on this rung.
Per the run's own gate, next is **T5** (kinematic/stability probe:
does a SCRIPTED gait, physically commanded to a shallower stance,
actually hold −15mm without tipping/losing contact, or does it also
settle back near −44mm?) before T2/T3 — T5 is NOT YET BUILT (needs a
small script using `info["height_mm"]`, already exposed per-tick by
`sim_env.py`, plus an IK-adjusted scripted stance; see
`linux_control/geometry_plant.py:knee_for_foot_z` for the exact FK/IK
this robot uses). Flagged as the next concrete task on this line,
not attempted this cycle to avoid a rushed/wrong physical-limit claim.

## BC-INIT — the wall BREAKS (08-12)

Per RL_PLAN's T6 conclusion (neither pricing nor state injection moves
mid-gait posture across 7+ arms), the next lever was BC-INIT: pure
action pretraining on the scripted tall gait (`bc_init_gait.py`,
DART-noise supervised cloning of `TripodGait` into a fresh policy
net), THEN RL fine-tune — disjoint from every closed lever because it
fixes the ACTIONS from tall states directly, not prices or states.

**`cw-dep-bcgait1` (2M fine-tune on the tip1 dep stack + walk_height_gate
sigma30 ref0) PASSES the primary/binding metric.** `probe_tall_wall`
steady-state height is **-10 to +6mm** across 3 seeds (every prior
RL-bred arm: -72 to -75mm) with leg-yaw margin **+17 to +18deg**
(every prior arm: pinned negative at the 35deg splay limit) — the
crouch+splay stability habit is gone. Harness confirms real travel:
det prog_ratio 0.77, speed 0.067 m/s, gait_valid 6/6, roll settles
clean (tail 0.4-1.4deg, recovers from transients up to 17.5deg, zero
falls), video visibly taller and genuinely walking. Not yet polished:
the run's own secondary bar (slip<=1.8) misses (det 2.12, sto 12.39
with one sacrificed leg in 1/6 sto episodes) — existence-proof win,
not a deployable candidate yet. Next: harden (more steps + DR/tipped
retention) to see if slip/robustness clean up with budget, matching
the bc1->bc1-hard1 and holdbc1->holdbc1-hard1 pattern elsewhere.

**`cw-dep-bcgait1-hard1` (10M hardening) PASSES decisively, same
cycle.** Height stays in-band (-8.5..-9.8mm, tighter than the 2M
parent), yaw margin stays positive (+2.1..+2.3deg, smaller than the
parent's +17..+18deg but still not pinned negative). BOTH secondary
misses fixed: det slip/m 2.12->1.43 (under the 1.8 bar), sto
12.39->1.51 with the sacrificed-leg episode gone (gait_valid 6/6).
det prog_ratio 1.05, sto 0.91, roll settles clean, zero falls. This
is now the strongest tall-walking candidate in the campaign; next is
the standard dep-line DR/tipped-start retention panel before any
Gate 0 consideration -- NOT yet run.

**08-12: retention panel items 1+2 (friction, floor slope) PASS, both
free.** `cw-dep-bcgait1-hard1-fric` (0.4-1.6x friction DR, 8M) and
`cw-dep-bcgait1-hard1-groundtilt5` (5deg floor tilt DR, 8M), both
respec'd from hard1's own checkpoint. Both: gait_valid 6/6 all four
eval slices (gate DR0 + own-DR, det+sto), zero falls/terminations,
slip/m 1.27-1.54 (within bar), probe_tall_wall height fric -7.9mm /
groundtilt5 -8.5mm -- matches or slightly beats the frozen parent's
own matched-probe (-12.7mm), no crouch re-drift. Video clean six-leg
gait both arms, no flag-leg. Watch item (not gate-breaking): leg-yaw
limit margin narrows further under DR (fric 0.82-1.45deg, groundtilt5
0.16-1.22deg vs the parent's matched-probe 1.37-1.90deg) -- continues
the lineage-wide narrowing trend (17deg->2.3deg->under 1.5deg); still
positive/not pinned but worth a combined-axis check before Gate 0.
CORRECTION (same cycle): tipped-start dose and command-side
DR-compose retests are NOT queued further here -- both are already
CLOSED generically (tipped-start dose saturates with zero separation,
3 arms; DR pair-compose on a dep-line checkpoint is a proven-free
closed class per RL_PLAN "CLOSED moves" -- a 3rd axis here would just
reconfirm it, not new evidence). The one open, non-generic question
for this lineage is whether it shares the dep-line's walk-TAKEOFF
roll-transient vulnerability (the still-open hw blocker, mechanism
diagnosed as contact/pinning geometry via replay_trace, not DR-fixable)
-- that needs either a bench tape replay against this checkpoint or a
probe_walk_push-style matched-parent injection ADAPTED to the
bcgait1-hard1 reward stack (probe_walk_push.py is hardcoded to the
plain vref1 stack today; needs a small generalization first). Spec
that adaptation before any further training respec on this lineage.

**08-12: the generalization landed and was run.** Turns out no new
reward stack was needed: `cw-dep-bcgait1-hard1`'s train command uses
the IDENTICAL physically-relevant cfg as `VREF1_STACK`
(goal.walk_speed_min/max_m_s, walk_obs_body_vel, safety.max_roll/
pitch_deg, walk_park_start_frac, anchor_tol_mm — verified against both
lineages' own launch commands) and only ADDS reward-only terms
(walk_height_gate/sigma_mm) that this probe never touches (it reads
physical info fields, never reward). `probe_walk_push.py` gained
CKPTS entries for the bcgait1 lineage; everything else (stack, CMD_V)
was already a match. Ran the diagnostic comparison it was built for
(forced 2.6N·m/1.5s injection, n=12 seeds, det, neither checkpoint
ever push-trained): `cw-dep-bcgait1-hard1` falls **6/12** vs frozen
`cw-dep-tip1` **9/12** under the identical injection — the tall gait
is LESS vulnerable to the takeoff-roll transient than the crouch
lineage, not more, and already lands close to the push-TRAINED
tip1_push1(-hard1)'s 5/12 fall rate without any push exposure at all.
Plausible mechanism (untested): a taller stance changes the torque
pulse's effective lever arm / CoM height, or the tall gait's less
knife-edge support polygon (SIM.md gap 4 diagnosed the crouch
lineage's 3-foot near-threshold margin as the root cause) is simply
less fragile to begin with. **Conclusion: no push-training respec is
warranted on this lineage** (the torque-DR family is closed for good
regardless, RL_PLAN "CLOSED moves") — this was a pure diagnostic use
of the probe, not a new training arm. The remaining path to Gate 0 for
`cw-dep-bcgait1-hard1` is hardware bench evidence (tape replay against
a real bench session), not another sim DR axis. Raw data:
`logs/probe_walk_push/bcgait1_hard1_vs_tip1.json` (train-1).
