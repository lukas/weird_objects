# QUADWALK reference-acceptance gate (pre-registered, 08-13)

Plain English: before any four-leg-walking policy can become the
"honest reference" that the quadwalk reward bank measures cheats
against, it must pass every check below. This file is the binding
gate spec required by the operator ruling of 08-13 ~12:4x UTC
(quad route (2): a feedback/RL rear-four-stepping policy MAY serve
as the bank reference, WITH an explicit pre-registered robustness
gate written down BEFORE the first arm launches). This is that
document; it was committed before `cw-quadwalk1` (the first arm)
launched. Changing these bars after a candidate exists is
forbidden — a sloppy reference poisons every downstream BC/anchor
use of the bank.

## Scope

Two different gates, do not confuse them:

1. **Discovery gate (per-arm):** did genuine rear-four stepping
   emerge at all? Judged on each discovery run (its ledger gate).
   Passing discovery does NOT make a checkpoint the bank reference.
2. **Reference-acceptance gate (this spec):** the full robustness
   bar a candidate must pass before the quadwalk ordering tests in
   `rl_move/tests/test_task_semantics.py` may un-skip and cite it.
   Until some candidate passes ALL clauses, the ordering tests keep
   skipping (`QUADWALK_REFERENCE_BLOCKED`) and no scripted-bank
   claim may cite an RL reference.

## What counts as "genuine rear-four stepping" (ruling condition a)

Evidence standard = the same as every gait claim in this campaign:

- Video/contact-sheet review (det pass) shows all FOUR support legs
  cycling ground-contact/swing — no pinned or dragged mid leg, no
  outrigger, no march-in-place.
- Positive net FORWARD translation along the commanded heading — a
  backward-rectified shuffle (the probe_quad_crawl failure mode,
  −0.02..−0.10 m) is an automatic FAIL regardless of other numbers.
- Both commanded lift legs (goal.quad_lift_legs, default fronts
  0+5) genuinely lifted: harness `fronts_lifted` criterion, lift
  duty over the post-grace tail < 0.15.

## Reference-acceptance clauses (ALL must pass)

Eval tool: `rl_move/sim/eval_checkpoint.py --modes quadwalk` (mode
assert live since the 08-13 fix; pre-08-13 quad rows were walk).
Episode = the mode's own 15 s protocol (2 s hold, 1 s ramp,
commanded 0.02–0.05 m/s).

1. **Multi-seed det:** >= 2 eval seeds x >= 6 det episodes each.
   In EVERY episode: gait_valid (quadwalk variant: lift legs
   excluded from the stepping requirement but required OFF ground;
   no sacrificed support leg), fronts lifted (tail duty < 0.15),
   net forward displacement >= +0.05 m; per-seed MEDIAN net forward
   >= 40% of commanded distance; NO episode net displacement
   < −0.02 m (backward rectification bar).
2. **DR panel at the walk bank's standard axes:** own-DR at
   dr-scale 0.5, plus the friction axis (0.4–1.6x, the
   bcgait1-hard1 panel axis). >= 6 episodes per axis: zero falls,
   positive net forward translation in >= 5/6, fronts lifted in
   all. Matched-parent control applies (same injection, same seed,
   `--baseline ppo_goal_cw_quad_hold2.zip`) for any judgment call.
3. **Zero falls anywhere:** no termination in any episode of
   clauses 1–2 (det, sto, or DR).
4. **Exemption verification in eval telemetry:** the two audited
   reward exemptions verified ON THE CANDIDATE's own eval rollouts —
   (a) lift legs earned ZERO step/swing credit (per-leg credit
   telemetry), (b) k_park_duty window spanned only the four support
   legs. A candidate that scores by exploiting either exemption is
   rejected even if clauses 1–3 pass.
5. **Stance stillness:** because k_quad_still is new in this
   lineage, the candidate's QUAD (hold) mode must show planar creep
   <= 0.05 m per 15 s episode (median, det; max <= 0.10 m) — the
   measured defect being fixed is 0.33 m/15 s. Also: during the 2 s
   zero-command head of quadwalk episodes, no measurable forward
   creep (> 0.02 m) before the ramp (creeping through "stillness"
   to farm progress is a cheat).
6. **Visual-quality report:** verdict quotes roll_tail_deg /
   roll_settled, drag_m / slip_per_m, height vs plant alongside
   success counts, per the 08-11 night directive. No numeric bar
   beyond clause 1, but a reference whose drag/roll numbers are
   grossly worse than the quad-hold parent's stance is a judgment
   REJECT, stated in plain words.

## Process notes

- Normal one-variable / matched-parent / phase rules all bind; the
  gate spec does not relax them.
- On the first PASS: the accepted checkpoint's det rollout becomes
  the bank's reference trajectory; un-skip the ordering fixture by
  replacing the `quadgait` branch with a rollout of the accepted
  policy (record run name + checkpoint checksum in the fixture
  docstring); REWARD.md quadwalk row updated in the same cycle.
- Until then: ordering tests SKIP loudly, and quadwalk training
  arms are permitted ONLY under the 08-13 ruling terms (excess
  capacity, hw keeps pod priority).
