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
  init did. **08-12 ~08:30 verdict: `cw-gait-anneal1` FAILED,
  informatively.** The mobile prior survives the charge (det fwd
  0.37-0.46 m — NOT the frozen fingerprint) but never cleans up:
  leg-3 flag-leg skate (gait_valid 0/6, sac [3] all det episodes),
  slip/m 4.3-5.1 vs the 1.5 bar, reward diving -335 → -4744 by
  quarters as the charge goes unresolved for 2M steps. Every
  no-new-code form of lever 2 is now closed. ~~The track is now a
  genuine WAIT on unwritten code~~ **08-13: the code-wait CLEARED —
  the in-run coefficient scheduler LANDED** (`sched.*` cfg keys,
  applied in `sim_env._step_begin`, so it runs on both trainer
  stacks by construction — the MJX vec envs call the same hook;
  default off = bit-exact, 10 new tests `test_coef_sched.py` + full
  semantics bank green, REWARD.md row; monotone per-process clock,
  never rewound by pool-restores). First arm `cw-gait-sched1`
  (2M discovery, from-scratch on the dragstance1 stack, charge
  ramped 0→8000 over global steps 0.5M→1.5M) is the LAST untried
  form of lever 2: let the paddle form first, then price the skate
  out from under a policy that already knows how to move. Gate
  pre-registered (mobility retained AND skate resolving). If it
  fails the same way as its siblings (freeze OR unresolved-charge
  skate), the from-scratch gait line has no levers left and closing
  it is the recommendation to the operator.

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
