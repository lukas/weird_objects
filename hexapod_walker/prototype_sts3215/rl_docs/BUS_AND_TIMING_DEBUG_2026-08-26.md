# Bus And Timing Debug Notes - 2026-08-26

## Problem

At 100 Hz the host occasionally saw MCU stream failures such as missing binary
headers, checksum/desync errors, or non-binary bytes where the snapshot reply
should have started. A still robot could pass short runs, but intermittent long
ticks and serial parser misses made 100 Hz unreliable.

## What changed

### MCU bridge

- Added bridge debug counters and `DBG` commands so the host can ask the MCU
  what happened without guessing from Linux-side symptoms.
- Added binary parser diagnostics for unexpected bytes before the `0xA5 0x5A`
  frame header.
- Changed the `S` snapshot command from "do every sensor read synchronously
  inside the host tick" to "serve the latest cached snapshot, then refresh
  Feetech position/speed and IMU data between host ticks."
- Added cache age reporting so the host can see whether it is using fresh data
  or falling behind.

### Linux bus tooling

- Added optional binary trace/debug capture in `mcu_feetech_bus.py`.
- Added `bus_bench.py --bridge-debug` output for bridge counters and raw
  pre-header bytes.
- Preserved the last known live servo scan when an empty scan result is seen,
  so one bad text reply does not poison the host's live-servo cache.

### Calibration bus error-rate reporting

- The calibration checkup now records `bus_error_rate_still` after safe-zero.
  It is a short read-only 100 Hz probe: use `read_snapshot` on stream-capable
  firmware, otherwise fall back to `read_all_positions`.
- The moving calibration phases now run through a tracked bus wrapper. The
  final `bus_error_rate_moving` phase summarizes the transactions that happened
  during ground contact, geometry, IMU body-frame, stability, mass-shift, and
  traction checks.
- The saved calibration report includes `bus_error_rate.still` and
  `bus_error_rate.moving`, with attempts, failures, error rate, first errors,
  timing percentiles, and live-joint counts. The current non-blocking pass/fail
  threshold is 1% failures.

### RL policy cadence

- Every exported NumPy policy must declare `meta.training_hz`.
- The live runner now uses the model's declared training frequency for its
  control period instead of assuming one fixed value.
- Drive mode refuses to mix stand and walk policies trained at different rates.
- Current shipped policies are marked as `training_hz: 25.0`.

### Live timing guard

- The live RL runner now records per-tick timing for service, observation,
  policy inference, safety, write, read, and total lag.
- Episode CSV logs include:
  `service_ms`, `obs_ms`, `policy_ms`, `safety_ms`, `write_ms`, `read_ms`,
  and `lag_ms`.
- A run trips safely if it has one severe deadline miss or several consecutive
  meaningful misses. The runner reports the timing summary, stops the episode,
  and limps instead of silently falling behind.

## Main observation

The strongest evidence pointed at the MCU bridge doing too much synchronous bus
work inside a 100 Hz host request, not at Linux scheduling jitter or random
servo hardware failure. The old snapshot path could spend around 800 ms inside
MCU-side feedback collection in bad cases. Serving cached data moved that work
outside the critical host tick.

After the cache-served snapshot change, a no-motion 100 Hz stream test completed
3000/3000 transactions with no bus errors, desyncs, or checksum failures. The
observed tick timing was roughly 5.8 ms mean, 6.1 ms p99, and 6.7 ms max, with
position and IMU ages staying around 6 ms or less.

## How the 100 Hz command tick works

100 Hz means the host has a 10 ms wall-clock budget per control tick. The
important change is that the host tick no longer tries to perform "send 18
targets, read 18 positions, read IMU" as separate blocking bus operations.

The stream-capable MCU firmware keeps a background cache of servo position,
servo speed, and IMU data. The host then uses the binary `S` command in two
modes:

- `read_snapshot`: `S n=0`. No motion command; return the latest cached
  position/speed/IMU snapshot. This is used for no-motion bus tests and
  still calibration probes.
- `step_all`: `S n=18`. Send all 18 absolute joint targets with shared
  speed/accel limits, have the MCU issue one SyncWrite, and return the latest
  cached snapshot in the same host round trip.

That gives the live controller one bus transaction for the whole tick. The
state has a small cache age, usually a few milliseconds, but the host avoids
stacking several serial and servo waits inside the same 10 ms slot.

The live RL paths prefer `step_all` when the bus exposes it. If a snapshot is
missing or incomplete, the controller records diagnostic state and treats the
sample as stale/missing rather than assuming the policy itself failed. On
legacy firmware with no stream support, the code falls back to `write_all` plus
the older read path, which is safer at lower rates.

## Command options if 100 Hz still shows errors

- Keep using `step_all` for high-rate live control. It is the intended 50-100 Hz
  path because it collapses command plus feedback into one host transaction.
- Use `read_snapshot` for read-only monitoring, no-motion benches, and the
  still bus error-rate calibration phase. It is the cleanest way to separate
  serial/read reliability from motion effects.
- Use `write_all` when you need to command a pose but do not need feedback in
  the same tick. This is fine for simple scripted moves or lower-rate control.
- Use `write_all` plus `read_all_positions`/`read_imu` as the legacy fallback.
  It works, but it is bus-heavier and should generally be treated as a
  25-50 Hz path unless measured otherwise.
- Avoid 18 separate per-joint writes for live control. Single-joint commands
  are useful for setup and manual checks, but they are the wrong shape for a
  tight whole-body control loop.
- If errors appear only while moving, compare `bus_error_rate_still` against
  `bus_error_rate_moving` before changing policy code. A still pass plus a
  moving failure points more toward power, grounding, vibration, connector
  margin, or signal integrity than toward RL timing.
- If the bus is marginal, lower the live control rate or decimate feedback
  reads. Any policy deployed at a different cadence must declare and match its
  trained `meta.training_hz`.

## Operational notes

- This update does not retrain the policies.
- Policies trained at 25 Hz should continue to run at 25 Hz unless retrained.
- A future 100 Hz policy should be exported with `meta.training_hz: 100.0` and
  trained with observations/actions sampled at that same rate.
- Bus transport passing at 100 Hz does not by itself make a 25 Hz policy into a
  100 Hz policy. The action cadence, observation history, and per-tick slew
  contract still need to match training.
- If bus symptoms return, first capture `--bridge-debug` output and compare
  bridge counters, cache ages, and Linux tick timing before changing wiring.
