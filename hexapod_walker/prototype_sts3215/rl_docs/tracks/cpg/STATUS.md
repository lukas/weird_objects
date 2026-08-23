# cpg - Berkeley-style parameter gait search

Last updated: 2026-08-23 ~05:5x UTC (**TEACHER-FORK A/B READ: CPG
library WALKS as an AMP style source but isn't yet better than the
scripted teacher.** `cw-cpg-teacherfork-ab-cpgv1` (Next item 3, single
lever vs the PASSED `cw-amp-m2-bcinit-sec5-style05`: `--amp-motion-lib`
teacher_v2.npz -> cpg_v1.npz, same BC-clone init/reward/2M budget)
VERDICTED INFORMATIVE (WORSE-BUT-WALKING, the pre-registered middle
branch): DR-0 gate det+sto gait_valid 6/6, zero falls/sacrificed legs,
no crouch (height_err 1.7-19.1mm, tighter than style05's own healthy
18-31mm range), video-clean six-leg cycling both modes — the CPG
clip's obs_style distribution does NOT collapse this init, refuting
that specific worry. But det margins run ~15-20% softer than style05's
own numbers (progress_ratio med 0.99 vs 1.16, fwd dist med 0.59 vs
0.69m/15s); sto is matched on progress (0.58 vs 0.58) and slightly
better on raw distance (0.30 vs 0.23m). Answer to the track's own
adoption question: the CPG motion library is a VIABLE but not yet
SUPERIOR AMP style source on this one test — no teacher_v1/teacher_v2
swap, no joystick slip-bar recalibration off this single result (per
the item's own pre-registered caveat). A second data point (CPG-clone
init instead of BC-clone init, or matched larger budget) would be
needed before any real adoption fork; not funded this cycle — M4
composition work on the amp track has priority for GPU budget right
now. Evidence: `logs/ckpt_eval/cw_cpg_teacherfork_ab_cpgv1_gate_det/`.
Prior banner below.)

Previous entry (2026-08-23 ~05:2x UTC (**ROBUST GATE FULL PASS — closed-
loop yaw trim built + verified, all 5 panels green.** The 120-iter
robustness-refinement search (Next item 1) converged but could NOT
fix the mu0.8 turn overshoot (best-found params still read 1.33/1.35,
identical to the contextual-250 winner) -- confirming Next item 2's
prediction that this is a friction-dependent GAIN error an open-loop
parameter search structurally cannot reach. Built
`rl_move/sim/yaw_trim.py` (`update_trim`, 8/8 unit tests, pure numeric,
no sim) -- a proportional MULTIPLICATIVE trim on commanded omega from
measured-vs-commanded yaw rate over 1.0s windows, wired into
`eval_cpg_gate.py` as `--yaw-trim` (default off, bit-exact when off,
scoped to turn segments only). Re-ran the SAME robust120-winner
params with `--yaw-trim`: **overall pass=True, all 5 panels
(dr0/dr0_script2/mu1.2/mu0.8/loaded) PASS** -- mu0.8 turn ratio
1.33/1.35 -> 1.07/1.07 (inside [0.70,1.30]), every other panel's turn
tracking also tightened (dr0 1.03/1.05 -> 1.01/1.02) since the trim
corrects small biases everywhere a turn is commanded, not just the
adversarial-friction case; zero falls, zero sacrificed legs, slip/m
0.69-1.36 across all panels (well inside the 1.4-2.9 teacher band),
video-reviewed (all 5 contact sheets: clean six-leg cycling, upright,
no pathologies). Exported
`rl_move/sim/policies/cpg_controller_robust120_yawtrim.json`
(first artifact export on this track). SKILLS.md row added.
**This is the FIRST FULL PASS of the track's behavioral DONE gate**
(zero falls, no sacrificed legs, correct headings, both-direction
turns, stops/restarts, slip inside the teacher band, DR-0 + friction +
servo-profile sweep, video-reviewed, saved parameter artifact) -- but
the gate's own text also asks for a web-UI / teacher-library-generator
loader for the artifact schema, which does NOT exist yet (confirmed,
grep-checked), and the track's own Next item 3 (a controlled A/B
adoption fork vs the current teacher) is unstarted. Track is NOT
declared closed on this evidence alone -- champions/gates are
append-only and adoption needs its own measured fork, same discipline
as the joystick track's stotight45 promotion-pending-integration
precedent. Evidence: `logs/cpg_gate/robust120-winner{,-yawtrim}/`,
`logs/paper_cpg_search/paper-cpg-robust120-20260823.{json,log}`.
Prior banner below.

Previous entry (2026-08-23 ~04:1x UTC (gate harness built + first
held-out gate run, see "Gate results 08-23". Track created same day:
**TRACK CREATED by operator direction:
make the Berkeley/Levine CPG result a third first-class path.** This
track is not PPO and should not be converted into seed sweeps. It
owns deterministic/black-box search over low-dimensional gait
parameters, direct behavioral scoring, and any controlled adoption of
the resulting gait as a controller or teacher source.)

## Why this is working better

The CPG search removes the two failure modes that keep hurting the RL
lines: the controller is a small interpretable parameter vector, and
the score is the eval. There is no hidden learned reward for the
policy to exploit, no stochastic policy noise floor, and no long run
where training reward rises while behavior gets worse. Each trial is a
real MuJoCo walk/turn attempt scored on commanded progress,
cross-track error, loaded-foot slip, falls, tilt/height, and effort.

## Current best

- Straight-50 winner verified real: tetrapod, period 2.0,
  swing_frac 0.18494, lift 0.035 m, cmd_tau 0.65225,
  workspace_margin 0.92865; progress_frac 0.9028, slip/m 0.5369,
  zero falls.
- Contextual-250 winner: tetrapod, period 2.0, swing_frac 0.2961,
  lift 0.0299 m, cmd_tau 0.1, workspace_margin 0.7759.
  Score 0.166 vs the straight winner's 0.033 under the fixed
  contextual scorer. Across 5 headings + 2 turns: heading progress
  0.84-0.89, cross <=0.043, slip roughly 0.56-0.75, yaw tracking
  0.993/1.012 of target, zero falls/terminations.

## Known fixes already made

- `sim_gait_compat.SE2FootGait` keeps the knee-frame boundary correct
  for MuJoCo.
- `paper_cpg_search` now supports replay and warm-started GP search.
- The contextual scorer integrates unwrapped yaw, so turns beyond pi
  no longer look sign-inverted.
- Pure-turn slip is normalized by progress plus a yaw foot-arc term
  instead of dividing by near-zero translation.

## DONE gate

The track is green when a CPG controller passes a held-out contextual
session gate with zero falls, no sacrificed legs, correct headings,
turns in both directions, stops/restarts, and slip better than or at
least competitive with the current teacher band. Required panels:
DR-0 plus a modest own-DR/friction/servo-profile sweep, 60 s command
scripts, video review, and a saved parameter artifact that the web UI
and teacher-library generator can load.

## Gate results 08-23 (`rl_move/sim/eval_cpg_gate.py`, built this cycle)

Held-out 60 s scripted session (seeded held-out headings +/-87/-49/+88/
+57/-20 deg, turns both ways, 3 stops/restarts), panels dr0 /
dr0-script2 / mu1.2 / mu0.8 / loaded-servo. Contextual-250 winner:
**DR-0 PASS decisively** (heading progress mean 0.899, turns yaw_along
1.03/1.05, stop drift <=3 mm, slip/m 0.70, zero falls, 6/6 legs
cycling duty ~0.70, roll/pitch peaks <0.6 deg; video reviewed, no
pathologies). Robustness 3/4: script2 0.899 PASS, mu1.2 PASS, loaded
PASS (0.923, best), **mu0.8 FAIL — open-loop turn overshoot 1.33/1.35
vs the pre-registered [0.70,1.30] band**, plus contact chatter (swing
counts 39-55 vs ~30). Everything else at mu0.8 passes (prog 0.827,
slip 1.35, zero falls). Artifact NOT exported (gate is strict; no
post-hoc threshold loosening). Verdict + videos:
`logs/cpg_gate/contextual250-winner/`. Root cause: the controller is
open-loop, so yaw-per-stride scales with ground friction; the search
never priced friction variation.

**UPDATE 08-23 ~05:2x: 120-iter robustness search done, still FAILS
mu0.8 identically (1.33/1.35, `logs/cpg_gate/robust120-winner/`,
confirms the root cause is a search-unreachable gain error) —
`--yaw-trim` closed-loop fix built and verified: SAME params,
`--robust --yaw-trim`, ALL 5 PANELS PASS (mu0.8 turn ratio ->
1.07/1.07; every panel's turn tracking tightened; zero falls/
sacrificed legs; slip/m 0.69-1.36). Artifact exported:
`rl_move/sim/policies/cpg_controller_robust120_yawtrim.json`. See
banner above for full detail. This closes Next items 1 and 2 below
(kept for the record).**

## Next

1. **DONE 08-23**: TRIAGED the robustness-refinement search
   `paper-cpg-robust120-20260823` (120 GP iters, warm-started from the
   contextual-250 winner) — best-found params still FAIL mu0.8
   identically to the contextual-250 winner (1.33/1.35): confirms the
   defect is not reachable by more open-loop search.
2. **DONE 08-23**: closed-loop yaw trim
   (`rl_move/sim/yaw_trim.py` + `eval_cpg_gate.py --yaw-trim`) built,
   unit-tested (8/8), and verified to fix mu0.8 without regressing any
   other panel — full robust-gate PASS, artifact exported. Not yet
   wired into the web UI (no loader exists for `cpg_controller_*.json`
   anywhere in the repo — grep-confirmed 08-23) or into
   `linux_control/se2_foot_gait.py`/`drive_controller.py` for a real
   hardware path (sim-only fix so far; hardware adoption is an
   [operator] physical-robot item per the standing rules, not blocked
   on anything code-side).
3. **DONE 08-23**: `rl_move/sim/motion_library/cpg_v1.npz` built
   (`build_motion_library.py --controller se2cpg`, new/default-off/
   bit-exact-when-tripod, 3 tests) from the yaw-trim-verified
   `robust120_yawtrim` params, same 15-family command suite as
   `teacher_v1`/`teacher_v2` — 45/45 clips accepted, slip/m 0.27-2.17.
   `cw-cpg-teacherfork-ab-cpgv1` (respec of the PASSED
   `cw-amp-m2-bcinit-sec5-style05`, single lever:
   `--amp-motion-lib` swapped) VERDICTED INFORMATIVE
   (WORSE-BUT-WALKING, see banner): CPG library sustains real walking
   as an AMP style source but det margins run ~15-20% softer than
   teacher_v2 (sto matched/better). ANSWER: viable, not yet superior;
   no teacher swap, no bar recalibration off one result. A second data
   point (CPG-clone init, or matched larger budget) would be needed
   before any real adoption fork — not funded this cycle, amp M4
   composition has GPU priority.
4. **DONE 08-23**: a loader for `cpg_controller_*.json` in the web UI,
   per the gate's own text. `linux_control/cpg_controller_loader.py`
   (10/10 tests, pure file I/O, no hardware) parses/validates the
   artifact into `SE2FootGait(**gait_kw)`; wired additively into
   `DriveController` as gait id 6 ("SE2 CPG (loaded)") plus new
   `CPGLIST`/`CPGLOAD <name>` commands (swap refused until a
   controller is loaded, refused while walking — same discipline as
   every other gait swap; verified end-to-end via `dry_run=True`, zero
   hardware contact) and a small picker in `linux_control/webui`
   (gait select + controller dropdown showing gate pass/slip + Load
   button, both reusing the existing generic `/cmd` channel — no new
   HTTP route needed). Copied `cpg_controller_robust120_yawtrim.json`
   into `linux_control/policies/` (the deployable location) so it's
   discoverable there too, not just in `rl_move/sim/policies`. This
   was the last named gap in the track's own DONE-gate text. Actually
   driving the physical robot on it is still an [operator] hardware
   decision (no HTTP/SSH/firmware touched by this change). No further
   named Next items remain on this track besides a real A/B adoption
   decision (a second data point beyond the one INFORMATIVE
   teacherfork-ab-cpgv1 read above) whenever amp/joystick GPU budget
   allows it.
