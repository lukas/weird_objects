# Hexapod control API (prefer over SSH)

Base URL: `http://192.168.4.44:8080` (or `HEXAPOD_URL`).

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

## Laptop

```bash
python3 -m rl_move.remote state
python3 -m rl_move.remote capture_plant
# limp:
curl -X POST --data 'X' http://192.168.4.44:8080/cmd
```

SSH only for deploy/restart when the operator asks — never for routine motion.
