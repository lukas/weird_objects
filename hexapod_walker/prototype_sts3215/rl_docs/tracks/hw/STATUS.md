# hw - hardware joystick mainline

Last updated: 2026-08-22 UTC. This is the current mainline status, not a
run history. Details live in `rl_docs/runs/`, W&B, `RL_LOG.md`, and topic docs.

## Goal

Make the physical hexapod stand, sit, turn, and walk under joystick control
by any reliable means: specialists, scripted blends, wrappers, and anchored
policies are all acceptable.

## Current Sprint

SIM SPRINT is binding while the robot is off the bench for repair. Mainline
work should protect or improve the download-ready rise+walk answer. Bench
items are parked.

## Current Download Answer

Use the hierarchy in `rl_docs/DOWNLOAD_ANSWER.md`:

- stance: `ppo_goal_cw_stand_footlow2_hard1`
- walk: `ppo_goal_cw_dep_bcgait1_hard1`
- session controller: per-mode re-anchor, entry slew, STOP->stance hold, rot60 wrapper

Gate: held-out session n=600, det 0.967, sto 0.853. This is still the product
baseline.

## Live Runs

None. All five fast-gait RL levers tried so far are refuted; the fork
sits with the operator (see Operator Gates) — now WITH a zero-RL
candidate: the phase-conditioned BC clone (below).

## Recently Finished

Fast-gait chain, 5 refuted RL levers in sequence (newest first):

- `cw-dep-bcgait4-phasedir1` (operator 08-22 gait-phase/direction-first
  order fb_20260822T000627: +2 phase obs sin/cos at the teacher clock,
  fixed 0.08 cmd, fixed-heading diet, NO charges, vel:=ref as fastbc1;
  first fast-gait arm on the MEASURED plant, tibia 150/a4beb8af):
  RL FAIL per pre-registered mode (b) — vs the matched un-RL'd clone
  control on the identical fixed-heading panel, 2M PPO degraded every
  axis (dir_err med 35.6->67.3 deg, rear headings collapse to prog
  0.01-0.07, speed 0.068->0.139 = the fastbc1 overspeed attractor,
  slip/m 1.81->4.17, roll_settled 12/12->5/12) while keeping ZERO
  falls + gait_valid 12/12 (phase input = effective anti-collapse
  anchor; speedbc1 fell 34/48 without it). THE CLONE ITSELF
  (`ppo_goal_cw_bcgait_init_fullprof_phase1`, committed, holdout err
  0.0040) PASSES the whole direction-first curriculum with zero RL:
  teacher grid 0.06-0.10 x 4 headings clean at the new plant; uniform
  random fixed headings 12/12 (prog 0.65-0.76, slip 1.6-2.0, in-band
  speed); irregular heading changes + stops 12/12 (prog 0.70-0.78).
  Metric note: direction_err_mean_deg has a ~35 deg tick-level floor
  from stride sway (the clean clone reads 35.6) — judge deltas, not
  the raw <=30 bar. Evidence: logs/probe_phasedir/ (train-0..2).
- `cw-dep-bcgait3-speedbc1-cont1` (+4M continuation, operator order
  fb 20260822T000318Z overriding a pre-registered STOP): FAILED WORSE
  than its parent. Rollout reward "recovered" (-137 -> -18) purely
  because episodes got shorter (ep_len 44.5 -> 22.6); per-tick reward
  stayed net-negative — the predicted artifact, confirmed. Our own
  pinned-speed panel (0.06/0.08/0.10 m/s, det+sto, DR-0) on the final
  checkpoint: 48/48 falls (parent 34/48 — worse), a NEW sacrificed-leg
  pathology (legs [1,3,5] det / [0,3,4] sto unused), dir err
  flat-to-worse (82-115 vs 58-80 deg), roll_tail unsettled at 9.2 deg,
  speed still command-invariant (0.12-0.17 m/s). No download change.
- `cw-dep-bcgait3-speedbc1` (speed-conditioned BC + leg-odometry vel
  obs + overspeed/heading charges): FAILED every axis — 34/48 falls,
  dir err 58-80 deg, slip det 3.0-3.5/sto 8-11, speed 0.12-0.14 m/s
  command-invariant. RL under the charges destabilized a previously
  stable clone instead of buying obedience.
- `cw-dep-bcgait2-fastbc1-track1` (`reward.k_walk_cmd_track=1.0`):
  FAILED — the tracking price made overspeed WORSE (DR-0 prog_ratio
  det 1.88x->2.10x, sto 1.20x->1.76x). Zero falls, no exploit, wrong
  lever.
- Teacher-level faster cadence (`--tripod-period-scale`): REFUTED by
  its own preflight grid — every faster rung strictly worse than 1.0
  at every profile dose. The native-cadence full-profile cell is what
  seeded `bcgait2-fastbc1`.

Refuted so far: faster cadence, command-tracking price, speed-obs +
overspeed/heading charges, more training steps. Fast walking itself
still exists (fastbc1, 2x overspeed); no download change from any of
this chain. Teacher/clone preflights and gate evals used the
repo-nominal sysid plant at cfe8160a (08-21 calibration commits are
bench tooling only — no calibrated plant values in repo yet, per MCP
addendum fb_20260821T224209).

## Current Evidence

- `footlow2_hard1` stance passes the full sim stance/session gate and remains the stance half of the hierarchy.
- `bcgait1_hard1` tall walk broke the crouch-splay wall and remains the walk half of the hierarchy.
- `postlower4` only beats the parent under remaining-rise semantics; adopting that semantics and promoting it are operator gates.
- `steer5-fastprof1` showed raised servo profile buys raw speed but breaks tracking/slip at the tested dose.
- `steer6-fasttrack1` full dose failed its acquisition gate: speed improved, direction and slip did not.
- `steer7-middose1` half dose improved over the matched parent but still missed in-band speed/tracking/slip bars.
- V5 fast anti-skate curriculum and `reward.k_loadslip_excess` are implemented and tested.
- Step-0 V5 raised-profile canaries failed before PPO, so the profile dose itself destabilizes the warm start.
- Profile ramp-in FAILED its own pre-registered gate at BOTH tested doses (`cw-dep-bcgait1-midramp1` mid, `cw-dep-bcgait1-fastramp1` full): the B0 precert already fails at the ramp's fitted START profile (falsifying "stable at the low end"), and by 1M at target dose both show ~44-52 deg direction/heading error and slip well over budget (fastramp1 also terminates walk_low_height 6/6 at the DR-0 gate) - the robot spins in place instead of tracking command, the same steer6-style skating pattern as the non-ramped attempts. Ramping the actuator-profile onset in slowly does not fix the raised-speed destabilization; the dose itself is the problem, not its abruptness.
- Train-through FAILED its own pre-registered gate at BOTH tested doses (`cw-dep-bcgait1-fastthru1` full, `cw-dep-bcgait1-midthru1` mid): periodic B0 certs showed falls trending WORSE not toward zero (midthru1: 25%->37.5% falls across the two 500k rounds; fastthru1 similar), the pre-registered kill trigger. Post-train eval at both doses: 0/6 det + 0/6 sto walk success, all TERM walk_low_height/fell, dir_err 35-78 deg, slip/m 2.1-11.0 vs a <=1.6 budget, video shows progressive leg-splay/kick instead of stepping. All 4 fast-gait canaries (2 onset styles x 2 doses) now fail identically: the raised actuator dose itself destabilizes direction-holding and footing, regardless of how or how much it is introduced.
- Recover/tangle REOPENED by operator order 08-20 and productionized in sim the same cycle: champion s13 packaged (policy + frozen encoder + relocatable loader), deployment-runner obs contract implemented + test-locked, 23-rung ladder through the runner path = DR-0 21/23 (matches the training-path gate; both misses known), own-DR 22/23; flip out of envelope (0/6 isolation). Recovery is an ADDITIONAL mode; rise+walk download answer unchanged. Evidence + blocker list: `rl_docs/RECOVER_DEPLOY.md`.
- Coxa length sweep is advisory only: geometry helps yaw margin/scrub, not straight walking speed; no sim pivot follows.

## Next Agent Actions

No fast-gait launches without an operator-chosen lever (5 refuted so
far). If the operator adopts the phase clone: pre-registered next
rungs are a DR hardening panel on the clone and board-side support for
the command-gated phase clock in the walk runner (CODE). Otherwise:
protect the download hierarchy, attack named session gaps if a
concrete lever exists. Also flag: pre-08-22 lineages trained on the
old 128 mm tibia plant; re-gating the download hierarchy on the
measured plant is an open operator call.

## Operator Gates

- Promote `postlower4` and/or change the runner/eval contract to remaining-rise semantics.
- Fast-gait fork: operator chose "just keep training" (fb 20260822T000318Z) — `cw-dep-bcgait3-speedbc1-cont1` FINISHED and FAILED worse than parent (48/48 falls, new sacrificed-leg pathology, no obedience gain — reward rise was the predicted episode-length artifact). Refuted levers now: faster cadence, k_walk_cmd_track, speed-obs + overspeed/heading charges, more training steps. Next lever is again an operator choice.
- Bench-promote the download hierarchy when the robot returns.
- Recover mode (reopened 08-20, sim-ready): decide flip handling (ship unsupported vs order a flip-hardening arm) and, when the robot is back, the recover-mode hardware safety contract (185 deg tilt envelope inside recover only) + on-robot transformer compute check. See `rl_docs/RECOVER_DEPLOY.md` blockers.
- Reopen geometry/CAD only by explicit operator direction.

## Closed For Now

- Single-policy distills are not the download answer.
- Generic speed-band or coefficient sweeps are not justified by current evidence.
- Bench measurements are paused while the robot is unavailable.

Keep this file under 120 lines. Replace stale bullets instead of appending a
chronological log.
