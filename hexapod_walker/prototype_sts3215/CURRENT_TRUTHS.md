# CURRENT TRUTHS - accepted facts and rulings

Last compacted: 2026-08-20 UTC. This file contains accepted current facts,
not campaign narrative. If history, logs, or old status prose disagree with
this file, this file wins.

## Mission

The goal is reliable real-robot joystick control: stand up, sit down, turn,
and walk where pointed. Reliability and safety outrank speed. Slip is a
calibration signal, not automatic failure.

## Current Top Ruling

SIM SPRINT is binding while the robot is off the bench for repair. The fleet
focuses on a download-ready MuJoCo rise+walk answer. Hardware attempts and
bench-only items stay parked until the operator says the robot is back.

## Current Download Answer

The measured product baseline is the hierarchy in `rl_docs/DOWNLOAD_ANSWER.md`:

- stance: `ppo_goal_cw_stand_footlow2_hard1`
- walk: `ppo_goal_cw_dep_bcgait1_hard1`
- session controller: re-anchor per mode, entry slew, STOP->stance hold, rot60 wrapper

Held-out session gate: det 0.967, sto 0.853 at n=600. This remains the
answer until a replacement beats the relevant gate and promotion contract.

## Known Gaps

- Post-lower rise is the weak session boundary. `postlower4` looks better only under remaining-rise semantics; changing the contract and promoting it are operator decisions.
- Takeoff roll transient is a hardware/sim-boundary issue; entry slew is the best measured sim mitigation so far.
- Learned stand-up on hardware is unproven. Scripted stand/sit glides remain fallback hardware tools.
- Fast gait is not deployable yet, but a live path exists. The raised motor profile destabilizes WARM-STARTED/transplanted policies (all 4 A/B canaries failed identically); a faster TripodGait cadence is refuted at the teacher level (08-20 preflight grid: every rung strictly worse at every profile dose). A fresh BC-INIT clone of the native-cadence teacher DOES survive RL fine-tune under the full profile: canary `cw-dep-bcgait2-fastbc1` PASSED discovery (tall, clean 6-leg gait, zero falls, direction correct, ~2x deployed speed) but overspeeds the commanded band 2x (nothing priced the excess under vel:=ref). Hardening rung `cw-dep-bcgait2-fastbc1-track1` (added `reward.k_walk_cmd_track=1.0`) FAILED its pre-registered gate — the tracking price made the overspeed WORSE (det 1.88x->2.10x, sto 1.20x->1.76x; own-DR sto 1.12x->1.92x). That lever is refuted; the sub-line is stalled on the operator (q_20260820T2330Z) for a new lever or a park. No download change unless a successor beats `bcgait1_hard1`.

## Real Robot Facts

- Scripted tripod gait walks, crabs, and turns, with visible loaded-foot slip and roughly half commanded travel.
- Working gaits rock about 10-20 degrees; large roll/tilt events and falls matter more than scalar reward.
- Learned stand-up previously tripped tilt_roll reliably on hardware; do not treat sim success alone as bench promotion.
- Servo/control facts in force: 25 Hz loop, 1.5 deg/tick stateful slew, loaded servo settle about 250-325 ms, air settle about 9 ms.
- No physical motion without an explicit operator ask.

## Policy And Eval Facts

- Policies output 18 raw joint targets through the SafetyLayer.
- `cw-dep-vref1-r1` established the vx/vy-measured-as-reference contract used by the hardware walking base.
- `eval_session` is the main gate for stance/walk session candidates.
- Video/physical behavior outranks reward alone. A reward-passing cheat is a metric bug, not a skill.
- Matched-parent controls are mandatory for injected physics/sensor axes.

## Research Rulings

- Operator-authenticated orders obey first and ask after, unless blocked by safety, typos, failing tests/preflight, or mechanical impossibility.
- One-variable-per-run is repealed. Coupled bundles are allowed when the operator orders them or the mechanism requires them; pre-registration and honest verdicts still bind.
- Agent-doable work drains before backoff. If code/triage/precondition work is available, execute it instead of rereading docs.
- No broad doc sweeps during execution cycles. Read the startup packet, inspect only files needed for the current decision, then launch, triage, code, or exit with the concrete gate.
- Peripheral launches to fill GPUs are violations. Idle pods are acceptable when the blocker is operator, hardware, or specification-gated.

## Track Facts

- `hw` is the mainline.
- `multitask` pause was lifted, but it remains secondary during SIM SPRINT unless directly serving download readiness or explicitly ordered.
- `nobc` gait-from-scratch stays closed. The operator's 08-21 anti-slip/no-speed-target reopening (order 20260821T133626Z) ran exactly one canary, `cw-nobc-slipwalk1-r1`, and it froze (0.001 m travel, 0.34 m scuffing, 4 legs unused). Its reward stack was preflight-proven correct (SLIPWALK bank: travel out-earns stall/park/skate by 300-2000), so from-scratch gait's blocker is EXPLORATION from a blank init, not reward specification. Sub-line stopped by operator instruction; reopening needs a start that already moves.
- `quad`, `turn`, `arch`, and `dynrep` are secondary during SIM SPRINT unless directly serving download readiness or explicitly ordered.
- Recover/tangle was reopened by operator order 08-20. The recover champion (`ppo_goal_cw_recover_predictive1b_pop3_s13` + frozen encoder `cw-dynrep-tf-state2-recovered1.pt`) is packaged and sim-gate-verified through the deployment runner (`rl_move/sim/recover_runner.py`; DR-0 21/23, own-DR 22/23 on the 23-rung ladder — equal to the training-path gate). Recovery is an ADDITIONAL operator-requested mode; it does not change the rise+walk download answer. Flip is out of envelope; hardware items are bench-parked (`rl_docs/RECOVER_DEPLOY.md`).
