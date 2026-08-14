# sysid/ — measured hardware-to-MuJoCo calibration

Implements the sim-to-real system-identification plan
(`~/Documents/HEXAPOD_SIM_TO_REAL_SYSID_PLAN.md`): stop tuning the
simulator by intuition, measure it. Deterministic command protocols run
identically on hardware and in MuJoCo; the difference is the reality
gap, and it is scored automatically.

Builds on what already exists — `rl_move/sim/servo_model.py`
(ServoProfile + fitted parameter sets), `rl_move/sim/replay_trace.py`
(free-base episode replay), `rl_move/sim/fit_loaded_actuator.py` (the
08-10 loaded knee fit). This package adds the missing pieces:
versioned protocols, an on-robot protocol runner with per-tick
send/receive timestamps, suspended (fixed-base) replay, distribution-
style latency stats, a general per-axis fitter with held-out
validation, and the Reality Gap Report.

## Layout

```
sysid/
  protocols.py   protocol builders (step ladders, sine sweeps,
                 per-servo spread, champion traj)  -> protocols/*.json
  run_hw.py      HTTP client: run a protocol on the robot, pull the
                 trace                             -> datasets/<run>/
  trace.py       sysid CSV schema load/write
  replay.py      suspended MuJoCo replay (trace or protocol)
  metrics.py     step / sine / latency-distribution / jitter metrics
  fit.py         per-axis actuator fit + holdout    -> sim_model_*.json
  report.py      Reality Gap Report                 -> reports/<stamp>/
  plots.py       cmd / hardware / sim overlay figures
```

Robot side (deployed with `linux_control/`): `sysid_protocol.py`
(schema + materializer, shared verbatim with this package) and
`sysid_runner.py` (25 Hz streaming executor, safety trips, CSV with
`t_send`/`t_recv` per tick), exposed as `POST /api/sysid/run`.
**The endpoint exists once `linux_control/` is redeployed to the
robot** — nothing here touches the robot until the operator runs
`run_hw.py --go`.

## Safety (non-negotiable, same rules as everywhere)

- Standard protocols assume: **robot on a stand, feet OFF the ground,
  operator watching.** That's ALL the operator does — the runner
  positions the legs itself: a slow (12 °/s), eased, trip-protected
  glide to the protocol's `home_deg` (bench zero for the standard
  batteries) or a champion trajectory's first row, verified to 3°
  before any segment runs.
- The one thing software cannot do is *verify the zero frame visually*:
  if `set_zero` is stale/wrong, the glide's tracking/current trips are
  the backstop (they limp and say so), but confirm zero after any
  hand-posing or reassembly.
- `run_hw.py` is a dry-run unless `--go`. Whole-body `traj` protocols
  additionally need `--force`; a traj that is discontinuous with the
  pose mid-protocol trips instead of yanking joints.
- The runner uses soft torque, per-joint current/temp trips, and a
  tracking-error trip (unexpected force = limp + descriptive error,
  suggesting a `set_zero` re-check). It always limps at the end.
- `python -m sysid.run_hw --abort` = emergency stop (`/api/rl/stop`).

## Workflow (maps to the plan's phases)

```sh
cd hexapod_walker/prototype_sts3215      # repo .venv via direnv

# Phase 0 — build the versioned protocol files (already in git;
# rebuild only to change repeats/amps — the hash changes with content)
python -m sysid.protocols build

# Preview any protocol in sim before touching hardware
python -m sysid.replay --protocol sysid/protocols/steps_air_v1.json \
    --servo-params loaded --plot

# Phase 1+2 — bench session (operator; robot suspended):
python -m sysid.run_hw --protocol sysid/protocols/steps_air_v1.json --go
python -m sysid.run_hw --protocol sysid/protocols/sines_air_v1.json --go
# Phase 6 — every servo, reduced battery:
python -m sysid.run_hw --protocol sysid/protocols/servo_spread_v1.json --go

# First overlay + gap numbers (also covers Phase 2: latency/jitter
# DISTRIBUTIONS from the per-tick t_send/t_recv and step onsets)
python -m sysid.report --csv sysid/datasets/<run>/*.csv \
    --servo-params air loaded

# Phase 3 — fit actuator/timing params (holdout: ±10° steps, 0.5 Hz)
python -m sysid.fit --csv sysid/datasets/<run>/*.csv \
    --out rl_move/sim/sim_model_sysid.json
# use in training/eval: --cfg-set bus.servo_params=<path to json>

# Phase 8 — suspended champion replay (Test A):
python -m sysid.protocols champion --csv <rl_*.csv or sim eval csv>
python -m sysid.run_hw --protocol sysid/protocols/champion_*.json \
    --go --force
python -m sysid.report --csv sysid/datasets/champion_*/*.csv \
    --servo-params sim_model_sysid.json
```

Decision tree after Test A (plan Phase 8): suspended mismatch →
actuator/timing (iterate here); suspended match → move to loaded/
contact work, for which `fit_loaded_actuator.py` and
`replay_trace.py` (free-base, on ground) are the existing tools.

## Conventions

- A protocol file's SHA-256 content hash identifies the experiment;
  the runner embeds the full protocol in every trace summary, so every
  dataset is self-describing and reproducible.
- Raw hardware traces in `datasets/` are append-only; never edit them.
- Fit quality is judged on the HELD-OUT set and full-trace replay RMSE
  (`fit.py` prints fitted-vs-base for both) — training-fit alone
  counts for nothing.
- Phases 4/5/7 (loaded actuator, current↔torque, contact) come after
  suspended agreement; fit contact only with actuator params frozen.
