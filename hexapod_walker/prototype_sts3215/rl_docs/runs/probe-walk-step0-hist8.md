# probe-walk-step0-hist8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T02:20:09+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: none (from scratch)

**hypothesis**: MECHANICAL PROBE (audit sec6): step0 reward package x obs.history_frames=8 never ran together from scratch. If-true: obs width = 8x base (trainer log), all three reward parts in audited bands, step events fire by 150k, no traceback, healthy fps. If-false: fix before the 4M cw-walk-step0-hist8 arm.

**gate**: mechanical only: healthy to 150k, history obs width correct, reward_step_event/reward_drag/reward_park_duty present, no traceback

