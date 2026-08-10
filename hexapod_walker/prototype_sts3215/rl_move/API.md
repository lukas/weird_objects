# Hexapod control API (prefer over SSH)

Base URL: `http://hexapod.local:8080` (or `HEXAPOD_URL`).

## 2026-08-06 incident — read this

Wrong logical zeros + unsupervised stand/plant blends tipped the robot,
browned out the board, held stilts at ~7 A, and cooked **L5 knee (ID 19)**.

**Safe sequence only:**

1. Limp / hand-set a **known visual** pose (usually legs straight out).
2. `POST /api/set_zero` — that pose becomes logical 0°.
3. Tiny air moves; predict then read. Stop if a servo ID is missing/hot.
4. Capture plant only when the operator likes the stance (`capture_plant`).
5. **No** autonomous stand-up.

## RL routes

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/rl/state` | Pose + plant + IMU + status |
| GET | `/api/rl/policy` | Deployed policy metadata (stance + walk) |
| GET | `/api/rl/preflight?mode=` | Read-only readiness (`stand`/`lower`/`walk`) |
| POST | `/api/rl/stand` | RL policy stand-up from belly (preflight-gated) |
| POST | `/api/rl/lower` | RL policy lower to belly (needs captured plant) |
| POST | `/api/rl/walk` | RL walk, EXPERIMENTAL: `{"vx":0.03,"vy":0,"duration_s":6}`, clamped 0.06 m/s / 20 s; needs captured plant |
| POST | `/api/rl/capture_plant` | Save **current** 18 joints (no motion) |
| POST | `/api/rl/set_stance` | Small crouch step; refuses Δq > 25° unless `force` |
| POST | `/api/rl/find_plant` | **Disabled** unless `{"force":true}` |
| POST | `/api/rl/probe_dynamics` | Air-only ±amp per joint → `logs/motor_model.json` |
| POST | `/api/rl/stop` | Abort worker |
| POST | `/api/set_zero` | Present pose → logical 0° (required after hand-set) |
| POST | `/api/zero` | Sit/stand glide; refuses large Δq unless `force` |
| POST | `/cmd` | `ARM` / `X` limp / `HOLD` / `# j deg` / `C` / `P` |

Drive `C` (centre) and `P` (stand) refuse if any live joint would move
more than **25°** from present unless the command includes `FORCE`.

## Deployed policies (2026-08-10, hardware attempt #2)

- stance = `ppo_goal_cw_stance_dr10` (`rl_policy_weights.json`, obs 68)
  — drives BOTH stand and lower. Tilt trip 10° (its trained envelope).
- walk = `ppo_goal_cw_dep_vref1_r1` (`rl_walk_weights.json`, obs 72,
  zip md5 `f9a466cfa7ae7213e48799a24032ac54`, verdict PASS 08-10) — the
  deployment-contract champion: trained with `goal.walk_obs_body_vel=2`,
  so the runner's vx/vy_meas := ref feed IS the training contract, and
  with a 25° relative-tilt envelope; the runner widens its SafetyLayer
  trip to 25° in walk mode to match (stand/lower stay 10°). Command in
  the trained band **0.05–0.06 m/s** (below 0.05 is out-of-distribution);
  duration clamped to 20 s; starts only from the captured plant stance.
  Still gated `hardware_ready: false` pending contact/current pricing
  calibration (tape-measure distance session).

Swap weights: `python -m rl_move.sim.export_policy_np --policy <zip>
--out linux_control/<weights>.json` → scp to
`/home/arduino/hexapod_sts/linux_control/` → `pkill -f web_drive.py`
(systemd `Restart=always` brings it back with the new files).

## RL episode logging (2026-08-09, on-robot, automatic)

Every stand / lower / walk run writes a full local trace under
`/home/arduino/hexapod_sts/linux_control/logs/` (`_EpisodeLog` in
`linux_control/rl_policy.py`) — nothing to enable:

- **`rl_<mode>_<stamp>.csv`** — one row per 25 Hz control tick,
  82 columns: `t_s`, body `roll_deg`/`pitch_deg` (attitude filter),
  `gyro_{x,y,z}_dps`, goal refs (`height_ref_mm`, `vx_ref_mps`,
  `vy_ref_mps`), running `max_cur_a`, then per joint 0–17:
  `q*_deg` (measured), `cmd*_deg` (commanded, post-safety),
  `act*` (raw policy action in [-1,1]), `cur*_a` (per-servo current;
  blank on ticks without full feedback). Flushed every ~1 s so a
  safety trip / kill still leaves the trace up to that moment.
- **`rl_<mode>_<stamp>_summary.json`** — params (policy meta, q_nom,
  tilt ref, preflight readings, walk vx/vy) + final result (ticks,
  max current, overruns, error/trip reason if any).
- **`events.jsonl`** gets `kind:"rl_episode"` markers at start and
  end (end carries the result + csv name), so episodes line up with
  button presses, overtemp events, MCU traffic on one timeline.
- The move-route JSON response includes `"log": "<csv name>"`.

Fetch for analysis (laptop):

```bash
scp arduino@hexapod.local:hexapod_sts/linux_control/logs/rl_*.csv /tmp/
# or stream events live: linux_control/receive_robot_logs.py
```

Analysis starters: commanded-vs-measured per joint (`cmd*` − `q*` =
tracking error / stall detection), `cur*` spikes vs. joints, roll/pitch
during walk, `overruns` in the summary for loop-rate health.

## Laptop

```bash
python3 -m rl_move.remote state
python3 -m rl_move.remote capture_plant
# limp:
curl -X POST --data 'X' http://hexapod.local:8080/cmd
```

SSH only for deploy/restart when the operator asks — never for routine motion.
