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

## Operational notes

- This update does not retrain the policies.
- Policies trained at 25 Hz should continue to run at 25 Hz unless retrained.
- A future 100 Hz policy should be exported with `meta.training_hz: 100.0` and
  trained with observations/actions sampled at that same rate.
- If bus symptoms return, first capture `--bridge-debug` output and compare
  bridge counters, cache ages, and Linux tick timing before changing wiring.
