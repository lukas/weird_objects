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

RESULTS: (pending — filled by the probe run of this cycle)

## 4. What this does NOT do

- No training arm is launched from this doc (ruling: design first).
  If the bench adopts the entry sequence and wants the policy
  fine-tuned UNDER it, that is a new hw-track arm citing this file as
  --evidence; the MJX parity of the new cfg keys must get a dedicated
  parity test before any GPU training run sets them.
- The bench-tape half of instrumentation that needs NEW hardware
  data (per-foot loading at takeoff — no foot force sensing exists;
  camera-synced contact-break order) stays operator bench work; the
  sim-side per-foot contact story comes from the replay JSONs.
