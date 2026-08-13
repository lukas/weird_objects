# TAKEOFF.md — walk-takeoff roll transient: instrumentation + staged gait-entry design

Owner track: hw. Operator ruling (08-13 ~12:4x UTC): STOP reward/DR
sweeps on takeoff; (a) INSTRUMENT the first ~1.5 s after gait start,
(b) DESIGN a staged gait-entry transition; training arms only after
an instrumented design exists. This file is that instrumented design.
Prior evidence: `rl_docs/BENCH_REPORT_2026-08-11.md` (18-walk A/B),
`rl_docs/SIM.md` known-gaps §4 (open-loop replay diagnosis),
`rl_move/hardware_traces/walk_replay_summary.json`.

## 1. What the bench tapes measure (26 walk tapes, 08-11 night; analysis 08-13)

Per-tick source: `rl_move/hardware_traces/bench_blast_*/rl_walk_*.csv`
(25 Hz; roll/gyro/q/cmd/act/currents + `vx_ref/vy_ref`).

- **The runner already stages the COMMAND**: every tape shows 1 s of
  zero velocity command, then a 1 s linear ramp starting at t=1.04 s
  (`rl_policy.py` WALK_HOLD_S/WALK_RAMP_S — the trained profile).
- **The roll transient does NOT wait for the command.** 14/26 tapes
  cross 5° of relative roll at or before t≈1.0 s — i.e. during the
  ZERO-COMMAND settle, before the velocity ramp begins; median
  5°-crossing across all tapes ≈0.9 s. The transient is triggered by
  policy ENGAGE, not by the first step.
- **Mechanism: a whole-body posture snap at max slew.** From tick 0
  the policy's actions are pinned at/near ±1 and the post-safety
  commands move at the full 1.5°/tick limit on 96–99% of all
  18 joint-ticks through the first second — at zero command. All six
  legs participate equally (cumulative command travel in the first
  1.5 s: 153–166° per leg, sd ≤11°). The policy immediately
  re-organizes the stance into its own preferred posture/gait cycle
  at maximum joint speed (37.5 °/s per joint); the body rolls on the
  support set while that happens.
- **Roll-rate scale:** peak |gyro_x| in the first 1.5 s: median
  32 dps, range 11–89 dps. Roll direction at the 5° crossing is
  biased (+17/−9) but not deterministic — consistent with the bench
  "coin flip" fall pattern.
- **Initial pose is NOT the gap:** |cmd−q| at tick 0 is only
  ~1.4–1.5° mean (the runner starts the policy from the captured
  plant stance it just held) — the snap is the policy's own posture
  preference + immediate gait cycling, not a start-pose mismatch.
- Open-loop replay (08-12, SIM.md §4): feeding these recorded action
  streams to the sim plant reproduces the excursions (sim peaks
  8.7–29.5° vs hw 6–25°) — the sim can express the transient; the
  closed-loop policy in sim simply never visits it (policy-in-loop
  baseline peaks 3–5.5°, and the no-push arm of the 08-13 prototype
  probe below reproduces that: peak 1.6°).

### 1b. Per-foot contact story in the takeoff window (matched sim replay, 08-13)

Tool: `rl_move/scripts/takeoff_audit.py` (`--all --replay`) — per-tape
command-side fingerprint (raw first-tick target jump, slew-saturation
counts, 5°-crossing ramp phase, tracking, currents, time-to-stable)
plus the open-loop matched replay's per-foot loading; artifacts:
`rl_move/hardware_traces/takeoff_audit/takeoff_audit_20260813.{json,md}`
(27 tapes = the 08-11 corpus + the 08-13 walk). Sim-replay evidence,
not hardware force sensing (which still does not exist — §4):

- **The engagement stance is already a three-foot knife edge.** After
  settling the replay at each tape's recorded start pose, the median
  per-foot load is L0/L2/L4 ≈ 7.5 N each and L1/L3/L5 ≈ 0 — the plant
  stance concentrates support on one tripod, the same support-geometry
  class as the stand-fall mechanism (SIM.md gap 4). The zero-command
  posture snap therefore starts with minimal support redundancy.
- **Feet start breaking contact during the zero-command settle:**
  first liftoff at median 0.64 s (L0 first in 12/26 replays), before
  the velocity ramp; median 9 contact transitions inside the first
  1.5 s; the loaded-foot count bottoms out at a median of 2.
- **~100 mm cumulative loaded-foot slip** (median) inside the window.
- Open-loop replay roll peak median 21.7° vs hardware 22.5° across the
  corpus — reconfirming per tape that the recorded command stream
  alone (no feedback, no disturbance) reproduces the excursion.
- The 08-13 tape (`bench_blast_20260813_091608`) matches the corpus:
  first-tick raw target jump 101°, crossing at 1.76 s (ramp phase),
  rode a 24° peak and ended holding a ~21° lean through the tail.

**Design consequence (measured, not conjectured):** any staged entry
that only shrinks the FIRST STEP or slows the velocity ramp attacks
the wrong stage — the excursion happens at zero command. The stage
that must be throttled is the POLICY HANDOFF itself: cap how fast the
policy may re-organize the stance right after engage, then return
authority.

## 2. The design: entry slew ramp at policy handoff

Mechanism (built 08-13, default-off, bit-exact off):
`safety.entry_slew_ramp_s` + `safety.entry_slew_start_deg`
(`rl_move/safety.py`; unit tests `rl_move/tests/test_safety_entry.py`;
semantics bank 91-pass green). After `set_nominal()` (episode start in
sim; policy engage in `rl_policy.py` — same call, same layer, both
stacks share the code path) the per-tick slew limit starts at
`entry_slew_start_deg` and ramps linearly to `max_delta_q_deg` over
`entry_slew_ramp_s` seconds. SNAP_ATTRS already deep-copies `safety`,
so the MJX pool-restore path carries the counter.

Staged entry as deployed (proposal):
1. Stage 0 (existing): scripted plant-stance hold; start-pose gate.
2. Stage 1 (new): engage policy with zero command under the entry
   slew ramp (start 0.25°/tick ≈ 6× slower, ramp back to 1.5°/tick
   over ~1.5 s — covers the measured 0.48–1.5 s excursion window).
3. Stage 2 (existing): 1 s velocity ramp, unchanged. Optionally
   gyro-gated (don't start the ramp until |gyro_x| < threshold for
   0.5 s) — NOT prototyped yet; only add if stage 1 alone is
   insufficient on the bench.

Known risk to test (why the prototype exists): the policy trained
with full 1.5°/tick authority; throttled ticks are out of
distribution and could themselves destabilize or just delay the same
snap to t=ramp-end.

## 3. Sim prototype protocol + results

`rl_move/sim/probe_gait_entry.py`: walk episodes from plant start,
runner-shaped command (pin_command: 1 s zero + 1 s ramp), calibrated
hardware-transient proxy `dr.walk_push` 2.6 N·m/1.5 s prob 1 (the
axis that reproduces the bench coin-flip regime policy-in-the-loop,
SIM.md) + a clean no-push arm; 12 seeds det; metrics: falls, peak
roll/rate in the first 2.5 s, tail roll, forward progress at 10 s.
Arms: entry off / ramp1.5s-start0.25 / ramp1.0s-start0.5, on the
deployed walk champion (`tip1`) and the tall-walk candidate
(`bcgait1_hard1`).

RESULTS (08-13, 144 rollouts on train-0, det, paired seeds — same
push draw per seed across arms; rows:
`logs/probe_gait_entry/all_rows.json`):

| ckpt | entry | push | falls | peak2.5s med | rate med | x@10s med (no-push) |
|---|---|---|---|---|---|---|
| tip1 | off | 2.6 N·m | **9/12** | 30.4° | 106 dps | +0.491 m |
| tip1 | ramp1.5/0.25 | 2.6 N·m | **4/12** | 7.2° | 48 dps | +0.418 m |
| tip1 | ramp1.0/0.5 | 2.6 N·m | 5/12 | 9.0° | 70 dps | +0.456 m |
| bcgait1_hard1 | off | 2.6 N·m | 6/12 | 17.6° | 86 dps | +0.545 m |
| bcgait1_hard1 | ramp1.5/0.25 | 2.6 N·m | 6/12 | 14.4° | 46 dps | +0.535 m |
| bcgait1_hard1 | ramp1.0/0.5 | 2.6 N·m | 6/12 | 15.2° | 72 dps | +0.547 m |

- **Deployed walker (tip1): the entry ramp SAVES 5 of 9 paired falls
  and causes 0 new ones** (per-seed McNemar 5–0); surviving episodes'
  peaks collapse (25–34° → 2.8–17.4°) and walking fully resumes after
  the entry (+0.42 m at 10 s vs the baseline's knocked-flat
  survivors). Falls all happen at ticks 22–34 (0.9–1.4 s), inside the
  entry/push window, as the tapes predict.
- **Tall-walk candidate (bcgait1_hard1): fall COUNT unchanged (6/12
  → 6/12, same seeds)** — under this fixed 2.6 N·m dose its falls are
  push-sign/geometry-determined, though median roll rate halves
  (86→46 dps) and fall peaks soften (32–36° → 27–29°). The entry ramp
  is not a walk-push armor for the tall gait; its value there is only
  the (hardware-side) snap removal.
- **The throttle is safe out-of-distribution**: 0/12 falls on every
  no-push arm for both checkpoints and all schedules; cost is only
  the entry itself (~7–15% less progress inside a 10 s window; steady
  gait unchanged, tail roll in-band).
- Honesty note: sim closed-loop never exhibits the hardware snap
  (no-push baseline peak ~1.4° med), so sim CANNOT directly show the
  ramp fixing it. The hardware argument is mechanical: the measured
  snap is slew-saturated (96–99% of joint-ticks at 1.5°/tick), so a
  6× slower entry cap bounds the self-induced angular impulse by
  construction; the probe's contribution is that the policy tolerates
  the throttle (no new instability, walking resumes) and that under a
  calibrated worst-case disturbance it strictly helps the deployed
  policy.

RECOMMENDATION (to the operator, deploy is operator-only): flip
`safety.entry_slew_ramp_s=1.5`, `entry_slew_start_deg=0.25` in the
runner cfg for walk engage on the next bench session and re-run the
takeoff reps of the 08-11 A/B protocol (alternating lead policy);
judge by fell/tail per the standing metric ruling. If bench falls
persist WITH the snap throttled, the residual is genuinely
external-disturbance-like and the next design stage is the gyro-gated
velocity ramp (stage 2 option above), not more entry throttle.

## 4. What this does NOT do

- No training arm is launched from this doc (ruling: design first).
  If the bench adopts the entry sequence and wants the policy
  fine-tuned UNDER it, that is a new hw-track arm citing this file as
  --evidence; the MJX parity of the new cfg keys must get a dedicated
  parity test before any GPU training run sets them.
- The bench-tape half of instrumentation that needs NEW hardware
  data (per-foot loading at takeoff — no foot force sensing exists;
  camera-synced contact-break order) stays operator bench work; the
  sim-side per-foot contact story is computed per tape by
  `rl_move/scripts/takeoff_audit.py --replay` (§1b).
