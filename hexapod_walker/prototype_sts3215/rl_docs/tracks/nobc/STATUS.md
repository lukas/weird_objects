# nobc — Learn without BC anchor

W&B: tag `track:nobc`. Excess-capacity research.

**Goal:** learn standing — and honest skill discovery generally,
clean non-scraping gait included — by RL alone: structural rewards,
curricula, terrain, RSI. NO imitation losses of any kind. hw ships
whatever works; this track exists to retire the crutch.

## Now

- Stand-from-scratch: six reward-side arms collapsed identically; the
  collapse traced to warm-start OOD drift, and the Warp pool-restore
  bug contaminated early verdicts — income-shaping/RSI verdicts are
  REOPENED for from-scratch designs. RSI machinery works.
- Gait-from-scratch: terrain clamp bug fixed (all past terrain was
  really <=18mm); terrain2 (true 72mm) FAILED into leg-sacrifice;
  dragstep1 FAILED — but its effective charge never exceeded
  ~0.09/tick vs ~1/tick income, so "really big penalty" was UNTESTED.
  **CROSS-TRACK: item 1 below is now ANSWERED, by an hw-tracked run**
  (`cw-gait-dragstance1`, GAIT.md) — the audit-derived structural
  per-stance charge (k=8000/m, correctly sized this time) trained
  from scratch and FAILED into freeze/park (charge paid the whole
  episode, never resolved, near-zero travel), matching its own
  pre-registered false branch. Solo structural charge does not
  induce stepping from scratch; move to item 2 (RSI-for-walk).
  **08-12: item 2 (RSI-for-walk mid-stride spawns, `cw-gait-rsi1`)
  also FAILED — CLOSED.** Identical training mechanism to
  dragstance1 (loadslip factor floored by step 49, drag charge never
  resolved); gate 0/6, slip/m 6-18, video is marching-in-place. Mid-
  stride state injection gives the from-scratch policy nothing to
  build on. **08-12: lever 5 (`cw-gait-slowfirst1`, target speed
  0.02-0.025 vs the champion's 0.05-0.06, same charge, no RSI) ALSO
  FAILED** — det fwd travel 0.00m every episode, video-confirmed legs
  shuffle in place with zero net displacement, same fingerprint
  regardless of command speed. Every no-new-code lever (2, 4, 5) is
  now closed on the identical mechanism. **Re-opened same cycle, a
  different no-new-code angle on lever 2:** `cw-gait-anneal1` —
  instead of an in-run anneal (needs a scheduler, CODE), warm-start
  the structural charge onto `cw-gait-dragstep1`, a genuine
  from-scratch RL-only paddler that already translates (by skating)
  before the charge ever turns on, testing whether an already-mobile
  prior reshapes into stepping instead of freezing like a random
  init did. If this also floors: every no-new-code form of lever 2
  is closed too, and only a true in-run coefficient scheduler (CODE,
  unqueued, spec first) or accepting BC-anchor (out of charter here)
  remains — that would be a genuine WAIT on unwritten code for this
  track's mainline lever.

- CROSS-TRACK INSIGHT (hw P0 probe, 08-11 late, GAIT.md bottom): the
  crouch-paddle is a sim-EFFECTIVENESS optimum, not a paid basin —
  it out-earns the plant-height gait ~495/ep on genuine progress
  income while tilt penalties are negligible (≤1.5/ep). Pricing
  magnitude alone must beat that moat without paying park; favors
  the per-stance charge + curriculum levers below over any further
  income shaping.

## Next

1. ~~Drag-charge magnitude audit~~ DONE 08-11 (see above) — solo
   structural charge does not induce stepping from scratch.
2. ~~RSI-for-walk mid-stride spawns~~ DONE 08-12 (see above) — CLOSED,
   same freeze/near-still mechanism as the drag charge alone.
3. Physics easing (unstarted) or annealed-up charge once the P2 bank
   lands, whichever is spec-ready first; then stand-from-scratch
   resumes with whatever levers moved gait discovery.

Detail: GAIT.md P3 · RISE.md forensic ladder.
