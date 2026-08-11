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
  ~0.09/tick vs ~1/tick income, so "really big penalty" is UNTESTED.

## Next

1. Drag-charge magnitude audit → one arm with the charge sized so a
   paddle tick costs 2-3x a progress tick (audit-derived k).
2. RSI-for-walk mid-stride spawns.
3. Annealed-up charge once the P2 bank lands; then stand-from-scratch
   resumes with whatever levers moved gait discovery.

Detail: GAIT.md P3 · RISE.md forensic ladder.
