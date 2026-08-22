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

`cw-dep-bcgait3-speedbc1-cont1` (acquisition, +4M warm from the failed
speedbc1 ckpt, train-7, launched 08-22 ~00:2x UTC by operator order fb
20260822T000318Z overriding the pre-registered STOP): tests whether
more training escapes the charge basin. Pre-launch decomposition of
the parent (4yitv3cc) says the late reward "recovery" was an
episode-length artifact — per-tick reward worsened (-2.95 -> -3.13)
while ep_len fell 317 -> 249 and pitch rose 4.7 -> 6.3 deg (falling
earlier truncates the net-negative heading+overspeed tax); direction
error (~78 deg) and speed (0.12 m/s) stayed command-invariant. Gate
forces per-tick-vs-ep-len decomposition + pinned-speed panels at every
1M snapshot (ckpts every 0.5M via --save-every); reward-only rise =
MISALIGNED verdict, fork back to operator. No download change.

`cw-dep-bcgait3-speedbc1` (discovery, 2M, operator order
20260821T224150Z — speed-conditioned BC + leg-odometry vel obs +
tested overspeed/heading charges) FAILED its gate on EVERY axis
(pinned-speed panel 0.06/0.08/0.10 det+sto DR-0, triaged 08-21 ~23:5x
UTC): 34/48 episodes end in tilt_pitch FALLS (gate: zero; parent
fastbc1 had zero), dir err med 58-80 deg (gate <=30), slip/m det
3.0-3.5 / sto 8-11 (gate 2.2/3.0), and raw speed 0.12-0.14 m/s
COMMAND-INVARIANT across the band — overspeed persists even though the
policy could see its own speed. RL under the charges destabilized a
previously stable clone instead of buying obedience. Pre-registered
FAIL mode -> STOP; the fast-gait speed-obedience fork returns to the
operator. Now refuted for making the fast walker obey a speed band:
faster cadence (teacher-level), k_walk_cmd_track scalar, and
speed-obs + overspeed/heading charges. Fast walking itself still
exists (fastbc1: zero falls, straight, ~2x overspeed). No-BC scratch
walking stays closed. Teacher/clone preflights and the gate eval all
used the repo-nominal sysid plant at cfe8160a (08-21 calibration
commits are bench tooling only; no calibrated plant values in repo —
noted per MCP addendum fb_20260821T224209).

`cw-dep-bcgait2-fastbc1-track1` (warm from `cw-dep-bcgait2-fastbc1`,
adds `reward.k_walk_cmd_track=1.0`) FAILED: the added command-tracking
price made the overspeed WORSE, not better — DR-0 prog_ratio det
1.88x->2.10x, sto 1.20x->1.76x; own-DR sto went from already-in-band
(1.12x) to overspeeding (1.92x). Zero falls, gait_valid 6/6, video
still tall/clean six-leg (no exploit); slip and heading error improved
slightly but that isn't the gate. This is the gate's own pre-registered
"wrong lever, STOP" outcome — no further respec of this reward line;
the fork returns to the operator (q_20260820T2330Z / STATUS.md).

`cw-dep-bcgait2-fastbc1` itself finished earlier: PASS as discovery canary
(fresh BC-INIT from the scripted TripodGait teacher under the FULL
servo profile at NATIVE cadence survives RL fine-tune — tall, clean
6-leg gait, zero falls, direction correct, det slip/m 1.76, realized
0.117 m/s ~2x deployed) but overspeeds the 0.05-0.08 command band 2x
(prog ratio med 1.95); the hardening rung above is the pre-authorized
fix attempt.

Context: the ordered faster-cadence knob (`--tripod-period-scale`,
landed 08-20, default-off) was REFUTED by its own teacher preflight —
period_scale 0.9/0.75/0.6 strictly worse than 1.0 in every cell of a
3-cadence x 3-profile x 3-speed grid (progress collapses, slip
explodes; stride auto-scales with period so a faster clock just scrubs).
The same grid proved the full profile SAFE for the native-cadence
teacher (prog 0.73-0.76 ≈ 0.073 m/s realized, slip/m 1.6-3.0, 147 mm,
clean 6-leg tripod, zero falls) — the order's higher-profile branch.
Prior A/B FAILs (fastthru/fastramp, mid+full) remain valid for
transplanted policies; this arm is the fresh-clone counterexample test.
No further fast-gait dose sweeps without operator authorization.

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

Triage `cw-dep-bcgait3-speedbc1-cont1` against its gate (per-tick
reward x ep_len decomposition + pinned-speed panels on the 0.5M/1M
snapshots, det+sto, DR-0, vs the parent's 2M panel). Otherwise no
fast-gait launches without an operator-chosen lever.

## Operator Gates

- Promote `postlower4` and/or change the runner/eval contract to remaining-rise semantics.
- Fast-gait fork: operator chose "just keep training" (fb 20260822T000318Z) — `cw-dep-bcgait3-speedbc1-cont1` running. Refuted levers so far: faster cadence, k_walk_cmd_track, speed-obs + overspeed/heading charges. If cont1's reward rise proves misaligned (predicted), next lever is again an operator choice.
- Bench-promote the download hierarchy when the robot returns.
- Recover mode (reopened 08-20, sim-ready): decide flip handling (ship unsupported vs order a flip-hardening arm) and, when the robot is back, the recover-mode hardware safety contract (185 deg tilt envelope inside recover only) + on-robot transformer compute check. See `rl_docs/RECOVER_DEPLOY.md` blockers.
- Reopen geometry/CAD only by explicit operator direction.

## Closed For Now

- Single-policy distills are not the download answer.
- Generic speed-band or coefficient sweeps are not justified by current evidence.
- Bench measurements are paused while the robot is unavailable.

Keep this file under 120 lines. Replace stale bullets instead of appending a
chronological log.
