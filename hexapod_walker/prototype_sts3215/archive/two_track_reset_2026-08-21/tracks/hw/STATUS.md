# hw - hardware joystick mainline

Last updated: 2026-08-22 ~04:4x UTC. Current mainline status, not a run history; details in `rl_docs/runs/`, W&B, `RL_LOG.md`.

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

`cw-dep-bcgait1-plant150-1` (walk fix arm, launched 08-22 after the
bank repair below; gate in queue item 2).
`cw-dep-bcgait4-phasedir2-staged-fwd` (staged phase-RL rung A, queued
08-22 per operator order fb_20260822T032514 — see Next Agent
Actions). 08-22 finding: the shipped
download pair HARD-FAILS eval_session at tibia-150 while the matched
128 mm control PASSES — `logs/ckpt_eval/plantgate_tibia150_session/`
(differential stands; absolute numbers predate the harness repair).
BANK REPAIR LANDED 08-22: the 08-21/22 bank collapse was bisected to
30660b51's absolute-tibia knee convention + new default stand home
leaking into the femur-relative sim (NOT the quad commits); fixed by
`linux_control/sim_gait_compat.py` + `_default_plant_deg` guard.
Walk banks GREEN; rise family retains 7 true tibia-150 residue FAILs
=> stance fix arm still launch-blocked. phasedir1 trained on the corrupted
sim — its RL verdict is env-confounded (queue item 3); the fork was
then resolved by operator order fb_20260822T032514 (Next Agent
Actions).

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
- `speedbc1` / `speedbc1-cont1` / `fastbc1-track1` / faster-cadence
  grid: all FAILED (details in their run docs / CURRENT_TRUTHS).

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

FAST-GAIT LEVER CHOSEN by operator order fb_20260822T032514 (08-22
~03:2x): staged phase RL with ALIGNED reward. Rung A
`cw-dep-bcgait4-phasedir2-staged-fwd` QUEUED at 1f0eadbd:
forward-only (`walk_heading_max_rad=0`), fixed 0.08, phase clone
init; NEW pricing per the order (stride-averaged course charge
`k_walk_course`, EMA overspeed band `k_walk_course_overspeed`,
clone-banded `walk_loadslip_gate`/`k_loadslip_excess` at ok=2.2/
max=4.0, travel floor `k_walk_idle_charge`) + STRONG gait
preservation (`bc_anchor_coef=1` with NEW `bc_anchor_phase_lock=1`
command-gated anchor clock and `bc_anchor_knee_abs=1` clone-dialect
teacher). Preflight `tests/test_phasedir_semantics.py` 20/20 GREEN:
clone out-earns every measured phasedir1 attractor per heading bin
incl. rear, margins attributable to the new terms (deviation filed
q_20260822T0430Z: ordered slip pricing inverts the generic
stall>park tail; obey buries both >=100). Gate is CLONE-RELATIVE per
bin (progress >=0.9x, slip <=1.15x, dir_err <= clone+5deg, speed
0.06-0.096, zero falls) + reward-vs-behavior table mandatory (order
item 6). Rungs B `walk_heading_set=[0,+-0.7854]`, C full fixed
headings, D irregular changes: each a respec of the prior PASS,
same clone-relative per-bin gate.
Plant flag RESOLVED TO A FINDING 08-22 (see Live Runs). AGENT QUEUE,
in order:
1. [code] DONE 08-22 (convention repair, this cycle) — residue: 7
   true tibia-150 recalibration FAILs (rise_valid_plant +
   score_replay = stale 128 mm `rise_ref_belly2plant.npz`, re-mint
   via extract_rise_ref at the measured plant; rise_rock; trans_drag;
   getup_honest_ordering; recover_floor_rungs; fastprof_obeying —
   ROOT-CAUSE HINT from the 08-22 phasedir2 preflight: at tibia-150
   the overdriven teacher SATURATES near the commanded speed, so the
   instant k_walk_overspeed band cannot separate the attractor;
   re-band or adopt the EMA k_walk_course_overspeed form).
   Repair each per root cause (reward vs test/ref); REWARD.md +
   snapshot rules apply. Rise/lower family green = stance-arm
   precondition met.
2. [precondition: rise banks green] `cw-stand-footlow2-plant150-1`
   (respec --from cw-stand-footlow2-hard1, --init-from its zip; GATE
   at tibia-150: det session (stance seat vs bcgait1_hard1) zero
   falls + sit_descends PASS + rise 12/12 zero-fall; control =
   parent's tibia-150 session fall on sit). The walk twin
   `cw-dep-bcgait1-plant150-1` LAUNCHED 08-22 (walk banks green;
   GATE at tibia-150: det session drive segments zero falls incl.
   reverse, fwd_heading soft PASS, gait_valid 6/6, slip/m <= 2.0;
   control = parent's back-fall + fwd yaw -21.8). Promotion of any
   passer is [operator].
3. [triage, upgraded] phasedir1 trained ON the corrupted sim (its
   snapshot 061dfe69 contains 30660b51: walk spawn poses + BC anchor
   knee-convention-broken). Its "RL degrades everything" verdict is
   env-confounded: before any staged-curriculum decision, re-run the
   SAME phasedir1 spec on the repaired sim (operator permission per
   fb_20260822T003132 still applies to the staged variant; a
   straight re-run of the already-approved spec on a fixed harness
   is repair, not a new lever).

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
