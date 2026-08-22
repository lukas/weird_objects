# hw - hardware joystick mainline

Last updated: 2026-08-22 UTC. Current mainline status, not a run history; details in `rl_docs/runs/`, W&B, `RL_LOG.md`.

## Goal

Physical hexapod stands, sits, turns, and walks under joystick control by
any reliable means (specialists, scripted blends, wrappers, anchored
policies all acceptable).

## Current Sprint

SIM SPRINT binding: protect or improve the download-ready rise+walk
answer. Bench items parked.

## Current Download Answer

Unchanged: `footlow2_hard1` + `bcgait1_hard1` + session controller
(`rl_docs/DOWNLOAD_ANSWER.md`; n=600 gate det 0.967 / sto 0.853 —
OLD-PLANT numbers, see Live Runs for the tibia-150 break).

## Live Runs

None. 08-22 finding: the shipped download pair HARD-FAILS eval_session
at the measured tibia-150 plant (falls: sit tilt_pitch, back
tilt_roll; fwd yaw -21.8 deg) while the matched 128 mm control (HEAD
harness, only a4beb8af reverted) PASSES —
`logs/ckpt_eval/plantgate_tibia150_session/`. Two warm-started fix
arms are spec'd (below) but LAUNCH-BLOCKED: rise/lower/walk semantics
banks are RED at HEAD (4 tibia-caused incl. the walk gait-gate
honest/flag-leg orderings; 10 both-plant, onset 08-21/22 window).
Bank bisect/repair is the topmost agent-doable item. Fast-gait fork
still sits with the operator (zero-RL phase clone is the candidate).

## Recently Finished

Fast-gait chain, 5 refuted RL levers in sequence (newest first):

- `cw-dep-bcgait4-phasedir1` (fb_20260822T000627; first fast-gait arm
  on tibia-150): RL FAIL per pre-registered mode (b) — 2M PPO degraded
  every gated axis vs the matched un-RL'd clone control (dir_err med
  35.6->67.3, rear headings collapse, speed 0.068->0.139, slip/m
  1.81->4.17) but ZERO falls + gait_valid 12/12 (phase input =
  anti-collapse anchor). The clone itself passes the whole curriculum
  with zero RL (SKILLS). dir_err has a ~35 deg tick floor — judge
  deltas. Evidence: logs/probe_phasedir/. Scope: refutes
  full-heading-from-rung-1 phase RL only (fb_20260822T003132).
- `cw-dep-bcgait3-speedbc1-cont1` (+4M, operator override of a STOP):
  FAILED WORSE — 48/48 falls (parent 34/48), sacrificed-leg pathology.
- `cw-dep-bcgait3-speedbc1` (speed obs + charges): FAILED every axis.
- `cw-dep-bcgait2-fastbc1-track1` (tracking price): FAILED, overspeed
  WORSE (det 1.88x->2.10x).
- Teacher-level faster cadence: REFUTED by its own preflight grid.

Refuted: faster cadence, tracking price, speed-obs+charges, more
steps, full-heading-from-rung-1 phase RL (staged untried). fastbc1
(2x overspeed) still exists; no download change. Plant note:
bcgait2/3 evals were old-plant; phasedir1 + clone are tibia-150.

## Current Evidence

- `footlow2_hard1` + `bcgait1_hard1` pass the full stance/session/walk gates ON THE OLD 128 mm PLANT and remain the hierarchy; at tibia-150 they fail the session gate (see Live Runs).
- `postlower4` only beats the parent under remaining-rise semantics; adopting that semantics and promoting it are operator gates.
- Raised servo profile destabilizes warm-started policies regardless of onset (steer5/6/7, ramp-in + train-through: all fail identically; details CURRENT_TRUTHS + rl_docs/runs/).
- Recover/tangle sim/deploy-ready: s13 packaged, runner contract test-locked, ladder-through-runner = training-path gate (DR-0 21/23, own-DR 22/23; flip 0/6 out of envelope). Additional mode only; details `rl_docs/RECOVER_DEPLOY.md`.
- Coxa length sweep is advisory only: yaw-margin/scrub lever, not walking speed; no sim pivot follows.

## Next Agent Actions

No fast-gait launches without an operator-chosen lever. If the
operator adopts the phase clone: pre-registered next rungs are a DR
panel on the clone + board-side command-gated phase-clock CODE.
PRE-REGISTERED (operator-permission required, fb_20260822T003132):
`cw-dep-bcgait4-phasedir2-staged` — phase clone init as phasedir1,
fixed 0.08 cmd, vel:=ref, no charges, STAGED headings (rung A
forward-only `walk_heading_max_rad=0`, B +/-45 deg, C full fixed, D
irregular changes); each rung gates on clone-vs-child DELTA (dir_err
med within +5 deg of the matched un-RL'd clone, zero falls, slip/m
<=2.2, speed 0.06-0.09); optional gait-preservation anchor if rung A
drifts. Hypothesis: full-heading conditioning from step 0 pushes the
clone off-manifold; staged exposure keeps it on.
Plant flag RESOLVED TO A FINDING 08-22 (see Live Runs). AGENT QUEUE,
in order:
1. [code] Bank bisect/repair: split is measured (4 tibia-150-caused:
   rise_rock, trans_drag_honest_rise, walk_gait_gate keeps-honest +
   collapses-flag-leg; 10 both-plant: rise_valid_plant, lower x2,
   slipwalk x2, walkcurr4, hold_load_hover, walk_kick_pulse,
   quadwalk_start, walk_gait_gate_collapses_quadwalk_midpin). Slipwalk
   bank was green 08-21 ~13Z → bisect 08-21/22 commits (suspects:
   quad rear-support 14cbf0de/ac22d050). Repair = fix stale test
   plant-constants OR fix reward mis-pricing, per root cause; REWARD.md
   + snapshot rules apply. Banks green = launch precondition met.
2. [precondition: banks green] Launch fix arms, warm-started,
   hardening phase, ~10M each, evidence = plantgate session FAIL +
   champion lineage: `cw-stand-footlow2-plant150-1` (respec --from
   cw-stand-footlow2-hard1, --init-from its zip; GATE at tibia-150:
   det session (stance seat vs bcgait1_hard1) zero falls +
   sit_descends PASS + rise 12/12 zero-fall; control = parent's
   tibia-150 session fall on sit) and `cw-dep-bcgait1-plant150-1`
   (respec --from cw-dep-bcgait1-hard1, --init-from its zip; GATE at
   tibia-150: det session drive segments zero falls incl. reverse,
   fwd_heading soft PASS, gait_valid 6/6, slip/m <= 2.0; control =
   parent's back-fall + fwd yaw -21.8). Promotion of any passer is
   [operator].
3. [triage] After bank repair, re-read phasedir1's RL degradation in
   light of the walk gait-gate mis-ordering at tibia-150 (it trained
   on the possibly mis-ordered stack).

## Operator Gates

- Promote `postlower4` and/or adopt remaining-rise semantics.
- Fast-gait fork: adopt the zero-RL phase clone / permit the staged phase-RL arm above / new RL pricing / park.
- Measured plant: PROMOTION of any plant-150 fix-arm passer (arms themselves are agent queue items above).
- Bench-promote the download hierarchy when the robot returns.
- Recover mode: flip handling; bench items parked (`rl_docs/RECOVER_DEPLOY.md`).
- Reopen geometry/CAD only by explicit operator direction.

## Closed For Now

Single-policy distills; generic speed-band/coefficient sweeps; bench
measurements while the robot is away.

Keep this file under 120 lines; replace stale bullets, don't append a log.
