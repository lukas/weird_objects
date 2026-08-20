# hw - hardware joystick mainline

Last compacted: 2026-08-20 UTC. This is the current mainline status, not a
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

Operator asked to test both fast-gait options A and B. The active ledger rows
as of 2026-08-20 19:27 UTC are:

- `cw-dep-bcgait1-fastthru1` - RUNNING, waived full-profile B0 pre-cert so PPO can train through the wobble.
- `cw-dep-bcgait1-midthru1` - INTENT, waived mid-profile B0 pre-cert.
- `cw-dep-bcgait1-midramp1` - RUNNING, profile ramp-in to 750/40/3 deg.
- `cw-dep-bcgait1-fastramp1` - RUNNING, profile ramp-in to 1500/80/5 deg.

Do not launch duplicates. Wait for these verdicts or poll the ledger.

## Current Evidence

- `footlow2_hard1` stance passes the full sim stance/session gate and remains the stance half of the hierarchy.
- `bcgait1_hard1` tall walk broke the crouch-splay wall and remains the walk half of the hierarchy.
- `postlower4` only beats the parent under remaining-rise semantics; adopting that semantics and promoting it are operator gates.
- `steer5-fastprof1` showed raised servo profile buys raw speed but breaks tracking/slip at the tested dose.
- `steer6-fasttrack1` full dose failed its acquisition gate: speed improved, direction and slip did not.
- `steer7-middose1` half dose improved over the matched parent but still missed in-band speed/tracking/slip bars.
- V5 fast anti-skate curriculum and `reward.k_loadslip_excess` are implemented and tested.
- Step-0 V5 raised-profile canaries failed before PPO, so the profile dose itself destabilizes the warm start.
- Profile ramp-in is built and now being tested by live canaries.
- Recover/tangle results improved scientifically, but recovery is not in the current download answer.
- Coxa length sweep is advisory only: geometry helps yaw margin/scrub, not straight walking speed; no sim pivot follows.

## Next Agent Actions

When live fast-profile runs finish:

1. Triage each run against its own pre-registered gate.
2. Update ledger, W&B note, run doc, `RL_LOG.md`, this file, and `STATUS.md` only if the story changes.
3. If one arm passes, propose or launch only the pre-registered successor allowed by its gate.
4. If all arms fail, stop the sub-line and report the operator choice: respec from evidence or park fast gait.

Do not open a new fast-gait dose sweep without operator authorization.

## Operator Gates

- Promote `postlower4` and/or change the runner/eval contract to remaining-rise semantics.
- Decide fast-gait continuation after `fastthru1`/`midthru1`/`midramp1`/`fastramp1` verdicts.
- Bench-promote the download hierarchy when the robot returns.
- Reopen recover/tangle or geometry/CAD only by explicit operator direction.

## Closed For Now

- Single-policy distills are not the download answer.
- Generic speed-band or coefficient sweeps are not justified by current evidence.
- Bench measurements are paused while the robot is unavailable.

Keep this file under 120 lines. Replace stale bullets instead of appending a
chronological log.
