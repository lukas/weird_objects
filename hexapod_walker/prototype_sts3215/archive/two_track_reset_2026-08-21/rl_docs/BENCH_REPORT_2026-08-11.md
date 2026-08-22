# Bench report — 2026-08-11 evening (unattended camera sessions)

Nine `bench_blast` sessions ran on the physical robot this evening,
almost all fully unattended (iMac camera recording, spoken countdowns,
fall-recovery loop, thermal gate). This file is the consolidated read:
what happened, what the tools are, and what it implies for RL training.

Raw data: `rl_move/hardware_traces/bench_blast_20260811_18*` and `_19*`
(per-session `summary.json`, episode CSVs, robot logs, `camera.mp4`,
frame sheets). Regenerate every table below with ONE command:

```sh
python -m rl_move.scripts.bench_report --since 20260811_18
```

## The tools (all landed tonight, all reusable)

| tool | what it does |
|---|---|
| `rl_move/scripts/bench_blast.py --go --auto --camera 0` | Runs a whole bench session unattended: spoken countdowns, iMac camera recording with exact unix-time sync, per-step preflights, fall detection → safe_zero(force) → stand recovery, thermal gate before every motion, terminal-result capture (never kickoff responses), CSV + robot-log pull. |
| `rl_move/scripts/video_review.py --session <dir>` | Syncs the camera video to the session log (exact, via `camera.t0_unix`), cuts a labeled frame sheet per event (walks, rises, turns), full-res zooms for tape segments. |
| `rl_move/scripts/bench_report.py` | NEW: one markdown table for any set of sessions — per-walk roll-trace metrics (transient timing, peak, tail, fell/recovered/clean), per-rise trip signatures, per-policy tallies. Reads only local files. |
| robot log pull | Each session dir gets `robot_logs/` (errors, events, measurements) filtered to the session window; the episode CSVs carry per-tick roll/gyro/currents/actions/obs/rot60_k. |

## Finding 1 — the learned rise fails DETERMINISTICALLY on hardware

10/10 attempts tonight (2 session openers + 8 recovery attempts across
5 sessions) tripped `tilt_roll` with an astonishingly tight signature:

| metric | value across all 10 |
|---|---|
| trip tick | 225–228 (~9.0–9.1 s, mid belly-curl, before the height ramp) |
| max relative roll | 10.1–10.6° (trip threshold 10°) |
| max current | 0.23–0.27 A (no load event — it's kinematic rocking) |

Two of the attempts started from a VERIFIED clean zero (max pose delta
0.5°, stand preflight green) — start pose is exonerated. Sim probes of
the same checkpoint keep |roll| ≤ 1.7° through the whole rise. The
hardware body rocks over the tucked legs during the curl; the sim's
never does. This is a pure sim-to-real gap in the curl dynamics, not a
threshold problem and not noise.

Meanwhile the scripted `POST /api/zero {"pose":"stand"}` glide stood
the robot up every time it was asked (except phantom-temp aborts,
finding 4). It is the working stand-up path until a rocking-hardened
policy exists.

## Finding 2 — the takeoff transient is UNIVERSAL; falls are a coin flip; the A/B has no winner

All 18 walks tonight (both policies, both directions, clean and
post-fall starts) show the same shape in the episode CSVs:

- |roll| crosses 5° within **0.6–1.5 s** of gait start;
- it peaks **13–27°** somewhere in the first ~1.5–5 s;
- then either settles back to level ("recovered", tail ≤ ~2°) or
  escalates to a fall — with no obvious predictor in policy, direction,
  or peak size (a 23° peak recovered; a 15° peak fell).

Full-night tallies (`bench_report`): **vref1 6/10 fell, 3 recovered,
1 clean. tip1 4/7 fell, 3 recovered, 0 clean.** The lone clean walk
(peak 7.8°) was vref1-r3 in the attended 18:24 session.

Two corrections to earlier same-night claims:

1. "tip1 is the deploy champion" (early-evening read off 4 walks) does
   NOT survive the full-night sample. Neither policy separates.
2. The A/B alternation scheme has a confound: round 1 always gives
   vref1-fwd / tip1-back, so 1-round sessions never walk tip1 forward.
   Every tip1 evening walk was backward. Fix before trusting any
   fwd/back split.

The discriminating problem is not vref1-vs-tip1 — it is surviving the
takeoff transient at all.

## Finding 3 — turn signs: half-answered

`omega=+0.3` (6 s scripted turn): large net rotation on camera,
read as **CCW from above — matching the z-up convention**
(single reading, 19:33 session; frames in
`bench_blast_20260811_193306/video/turn_p03_*.png`).

`omega=-0.3`: STILL unmeasured. The first attempt was silently refused
by the robot (`pending measurement — save or discard it first` — the
+0.3 turn's un-annotated measure record blocked it; the robot just
stood there for its window). That bug is fixed (`bench_blast` now
discards the pending record before each video-mode turn) and the rerun
executed, but the camera was being carried away during exactly that
window. First item for the next session.

## Finding 4 — tonight's "thermal wall" was (mostly) PHANTOM BUS READS

Three motion aborts blamed temperature: L2 hip 72 °C (19:18), L4 hip
68 °C (19:36), L4 hip **150 °C** (19:51). The 150 °C one broke the
story: the same servo read a steady 33 °C seconds later — thermally
impossible — and the debounced watchdog (`servo_watch`, two consecutive
hot reads required, exactly because of documented 08-09 phantoms) never
tripped once all night. The single-read temp checks in the glide/rise
prep code (`safe_zero.py`, `pinned_tip.py`) were the ones aborting
sessions, and corrupted bytes on the shared bus fake hot reads.

So: possibly NO real thermal event happened tonight. (The 72 °C reading
had a plausible heat story — falls + recoveries stack load — so treat
it as unconfirmed rather than fake; servos all measured ≤ 38 °C minutes
later.)

Fixes landed:

- `safe_zero.py` / `pinned_tip.py`: temp trips now need two consecutive
  hot reads ~0.3–0.6 s apart (same debounce the stall check always had).
  A real overheat still trips in under a second.
- `servo_watch.py` + `bench_api.py`: the always-on watchdog now has a
  **thermal panic** — on a (debounced) overtemp it kills ALL motion
  (aborts the demo/RL worker, gait stop, torque-all off) instead of
  cutting one servo and letting the job drive the other 17 legs. Busy
  cadence tightened 10 s → 5 s.
- `bench_blast.py`: thermal gate before every motion step (hold at
  ≥55 °C until back under 45 °C, abort if it won't cool).

## Finding 5 — session automation debugged itself into a real harness

Each unattended failure mode got a permanent fix the same night:
terminal results instead of kickoff lies, fall recovery
(safe_zero `force=true` — a real fall ALWAYS trips the tilt gate),
scripted-stand fallback behind the deterministic learned-rise trip,
demo-aware waits (`/api/zero` and `safe_zero` run as demos invisible to
`wait_idle`; one abort read a mid-glide pose and masked the real
error), auto-safe_zero when the opening pose isn't belly zero (a
stalled safe_zero had left the L4 knee 78° off and hold-hunting — the
operator's "twitching leg"), pending-measurement discard before turns,
CSV size-stable pulls, alternating walk directions to stay in frame.

## What this implies for RL training

1. **Train the takeoff transient, not the A/B — and it must be
   DYNAMIC.** The sim's plant-start episodes do not produce the
   hardware's 13–27° first-seconds roll excursion, so neither policy
   ever learned to manage it — recovery is luck. The static-lean dose
   arm already CLOSED while this report was being written:
   `cw-dep-tip1-takeoff25-r1` (proper 20–25° tipped-start injection,
   matched baseline) showed child == parent with ZERO falls in both —
   the sim already recovers static tipped starts at the hardware
   regime. So the gap is not the lean, it is the roll RATE: a
   gait-start roll-velocity perturbation axis (code, unbuilt) or
   contact/pinning work. Whatever the arm, gate on the fell/tail
   criterion, not peak — hardware shows peaks near 25° are survivable.
2. **We now have the data to model the transient instead of just
   dosing it.** The episode CSVs carry per-tick roll, gyro, per-servo
   currents, commands AND actions for all 18 walks. A trace-replay
   calibration (feed the recorded action sequences to the sim plant,
   compare roll evolution) would tell us whether the gap is contact
   (feet unsticking from settled plant), actuator lag under load, or
   mass distribution — and turn "add DR" into "fix the model."
   This is the same lever the rise needs (see 3).
3. **The rise gap is a specific, reproducible target.** Trip at tick
   ~227 of the curl, 10.1–10.6° roll, currents flat — the hardware
   curl rocks laterally where sim glides. The queued
   `cw-stand-riserock1` drained as a stub and is VOID (the rocking-DR
   code was never written); the rise-rock axis is still unbuilt code
   that would treat the symptom, while the loaded-knee actuator model
   treats the cause. Because
   the failure is deterministic and cheap to reproduce (10 for 10!),
   it is the best sim-calibration probe we have: match THIS trace
   first, then retrain.
4. **Keep the 10° rise trip.** The walk envelope tolerates and often
   recovers 15–25° transients, which tempts raising the rise trip —
   but a rise failure means falling from a half-risen, legs-tucked
   pose with no stance to recover into. Harden the policy, not the
   threshold (standing ruling from 08-11 stays).
5. **Fix the A/B before drawing policy conclusions.** Alternate which
   policy leads each round (or force per-policy direction coverage) so
   tip1 gets forward walks. Until then, treat vref1-vs-tip1 as
   undecided and spend bench time on transient reps instead.
6. **Fell/tail is the metric.** `bench_report` now computes it from
   the CSVs; the sim eval side should report the identical statistic
   (`tail |roll| over the last second`) so hardware and sim numbers
   are directly comparable.
