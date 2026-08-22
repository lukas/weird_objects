# hw - hardware joystick mainline

Last updated: 2026-08-22 UTC. Current mainline status, not a run history; details in `rl_docs/runs/`, W&B, `RL_LOG.md`.

## Goal

Physical hexapod stands, sits, turns, and walks under joystick control by
any reliable means (specialists, scripted blends, wrappers, anchored
policies all acceptable).

## Current Sprint

SIM SPRINT binding while the robot is off the bench: protect or improve the
download-ready rise+walk answer. Bench items parked.

## Current Download Answer

Unchanged: the stance/walk hierarchy in `rl_docs/DOWNLOAD_ANSWER.md`
(`footlow2_hard1` + `bcgait1_hard1` + session controller; held-out
session gate n=600, det 0.967, sto 0.853). Still the product baseline.

## Live Runs

None. All five fast-gait RL levers are refuted; the fork sits with the
operator (see Operator Gates) — now WITH a zero-RL candidate: the
phase-conditioned BC clone (below).

## Recently Finished

Fast-gait chain, 5 refuted RL levers in sequence (newest first):

- `cw-dep-bcgait4-phasedir1` (operator 08-22 phase/direction-first
  order fb_20260822T000627; first fast-gait arm on the MEASURED
  tibia-150 plant): RL FAIL per pre-registered mode (b) — vs the
  matched un-RL'd clone control, 2M PPO degraded every gated axis
  (dir_err med 35.6->67.3 deg, rear headings collapse, speed
  0.068->0.139 = the fastbc1 overspeed attractor, slip/m 1.81->4.17)
  while keeping ZERO falls + gait_valid 12/12 (phase input = effective
  anti-collapse anchor; speedbc1 fell 34/48 without it). THE CLONE
  ITSELF (`ppo_goal_cw_bcgait_init_fullprof_phase1`, committed) PASSES
  the whole direction-first curriculum with zero RL: all fixed
  headings incl. rear 12/12 AND irregular heading changes 12/12 at
  fixed 0.08 (SKILLS row). Metric note: direction_err_mean_deg has a
  ~35 deg tick-level floor from stride sway — judge clone-vs-child
  deltas, not the raw <=30 bar. Evidence: logs/probe_phasedir/.
  SCOPE (operator-keyed fb_20260822T003132): this FAIL refutes
  phase-RL with FULL +/-180 deg random headings from rung 1
  (`goal.walk_heading_max_rad=3.1416`), NOT the phase input and NOT
  the operator's intended STAGED curriculum, which is untried (spec
  in "Next Agent Actions").
- `cw-dep-bcgait3-speedbc1-cont1` (+4M continuation, operator order
  fb 20260822T000318Z overriding a pre-registered STOP): FAILED WORSE
  than parent — reward "recovery" was purely shrinking episodes
  (per-tick reward stayed net-negative); pinned-speed panel 48/48
  falls (parent 34/48), new sacrificed-leg pathology, dir err
  flat-to-worse, speed still command-invariant. No download change.
- `cw-dep-bcgait3-speedbc1` (speed-conditioned BC + leg-odometry vel
  obs + overspeed/heading charges): FAILED every axis — 34/48 falls,
  dir err 58-80 deg, speed command-invariant. RL under the charges
  destabilized a previously stable clone.
- `cw-dep-bcgait2-fastbc1-track1` (`reward.k_walk_cmd_track=1.0`):
  FAILED — tracking price made overspeed WORSE (det 1.88x->2.10x).
- Teacher-level faster cadence (`--tripod-period-scale`): REFUTED by
  its own preflight grid — every faster rung strictly worse than 1.0
  at every profile dose.

Refuted so far: faster cadence, command-tracking price, speed-obs +
overspeed/heading charges, more training steps, and phase-obs with
full-heading-from-rung-1 (staged variant untried). Fast walking itself
still exists (fastbc1, 2x overspeed); no download change from any of
this chain. Plant note: bcgait2/3 chain evals used the old 128 mm
plant; phasedir1 + the phase clone are the first fast-gait artifacts
on the measured tibia-150 plant (a4beb8af).

## Current Evidence

- `footlow2_hard1` stance passes the full sim stance/session gate and remains the stance half of the hierarchy.
- `bcgait1_hard1` tall walk broke the crouch-splay wall and remains the walk half of the hierarchy.
- `postlower4` only beats the parent under remaining-rise semantics; adopting that semantics and promoting it are operator gates.
- `steer5/6/7` profile-dose arms all missed speed/tracking/slip bars (full and half dose); V5 anti-skate curriculum + `reward.k_loadslip_excess` are implemented and tested; step-0 V5 raised-profile canaries failed before PPO — the dose destabilizes the warm start.
- Profile ramp-in AND train-through both FAILED their pre-registered gates at both doses (`cw-dep-bcgait1-{mid,fast}ramp1`, `cw-dep-bcgait1-{mid,fast}thru1`): all 4 canaries (2 onset styles x 2 doses) fail identically — spinning/skating or progressive leg-splay instead of stepping. The raised actuator dose itself destabilizes warm-started policies regardless of how or how gradually it is introduced (details: rl_docs/runs/, CURRENT_TRUTHS).
- Recover/tangle REOPENED by operator order 08-20 and productionized in sim the same cycle: champion s13 packaged (policy + frozen encoder + relocatable loader), deployment-runner obs contract implemented + test-locked, 23-rung ladder through the runner path = DR-0 21/23 (matches the training-path gate; both misses known), own-DR 22/23; flip out of envelope (0/6 isolation). Recovery is an ADDITIONAL mode; rise+walk download answer unchanged. Evidence + blocker list: `rl_docs/RECOVER_DEPLOY.md`.
- Coxa length sweep is advisory only: geometry helps yaw margin/scrub, not straight walking speed; no sim pivot follows.

## Next Agent Actions

No fast-gait launches without an operator-chosen lever (5 refuted so
far). If the operator adopts the phase clone: pre-registered next
rungs are a DR hardening panel on the clone and board-side support for
the command-gated phase clock in the walk runner (CODE).
PRE-REGISTERED (operator-permission required, per fb_20260822T003132):
`cw-dep-bcgait4-phasedir2-staged` — phase clone init as phasedir1,
fixed 0.08 cmd, vel:=ref, no charges, but STAGED headings: rung A
forward-only (`walk_heading_max_rad=0`), rung B +/-45 deg, rung C
full fixed headings, rung D irregular changes; each rung gates on
clone-vs-child DELTA (dir_err med within +5 deg of the matched
un-RL'd clone, zero falls, slip/m <=2.2, speed 0.06-0.09) before the
next; optional gait-preservation anchor if rung A already drifts.
Hypothesis: RL degrades the clone because full-heading conditioning
from step 0 pushes it off-manifold; staged exposure keeps it on.
Otherwise: protect the download hierarchy, attack named session gaps
if a concrete lever exists. Also flag: pre-08-22 lineages trained on
the old 128 mm tibia plant; re-gating the download hierarchy on the
measured plant is an open operator call.

## Operator Gates

- Promote `postlower4` and/or change the runner/eval contract to remaining-rise semantics.
- Fast-gait fork: OPEN, operator's choice — adopt the zero-RL phase clone (then DR panel + board phase-clock CODE), permit the staged-curriculum phase RL arm pre-registered above (fb_20260822T003132), set a new RL pricing, or park. Five RL levers refuted (list under "Recently Finished").
- Bench-promote the download hierarchy when the robot returns.
- Recover mode (reopened 08-20, sim-ready): decide flip handling (ship unsupported vs order a flip-hardening arm) and, when the robot is back, the recover-mode hardware safety contract (185 deg tilt envelope inside recover only) + on-robot transformer compute check. See `rl_docs/RECOVER_DEPLOY.md` blockers.
- Reopen geometry/CAD only by explicit operator direction.

## Closed For Now

- Single-policy distills are not the download answer.
- Generic speed-band or coefficient sweeps are not justified by current evidence.
- Bench measurements are paused while the robot is unavailable.

Keep this file under 120 lines; replace stale bullets, don't append a log.
