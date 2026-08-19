# RL balance run log

Append-only diary of hardware sessions. Newest entries at the bottom.

---

## 2026-08-06

### Context
- Phase-1 package `rl_move/` created from merged `RL_PLAN.md`.
- Step B probe OK earlier: pos ~5 ms, IMU ~5 ms, combined ~13 ms @ 50 Hz.

### Tip / brownout #1 (~20:07 UTC board time)
- Ran early `balance_test` which blended toward stand plant + SyncWrite.
- Robot tipped; board hard-rebooted (`last` → `crash`).
- Cause: servo pack brownout (18 motors fighting) → Uno Q 5 V collapse — not a software shutdown.

### Tip / brownout #2 (~20:31)
- After recovery, probe showed nearly level (~−1.5° roll / +2° pitch) but knee samples ~−92° (crumpled / odd pose).
- `hold_current` + torque enable + SyncWrite → second hard reboot.
- Hardened: preflight tilt, tighter kill (10°), limp on terminate, **`enable_motion: false` default**.

### Dry-run (motion off)
- `balance_test` 1×250 steps, truncated OK, board stayed up.
- Overruns high (~94) from double pacing in script+env — follow-up.

### Next
- Robot reported upright by operator — enable motion, hold-current only, short episode.

### Motion hold-current #1 (operator upright, ~20:34 board)
- Log file: `rl_move/RUNLOG.md` started.
- Preflight probe: roll −1.5°, pitch +2.0°, bus/IMU OK; L0 sample
  yaw≈−2°, hip≈−65°, knee≈+36° (not CAD plant — holding *current*).
- `enable_motion=true`, `hold_current_pose=true`, 3 s / 150 steps @ 50 Hz target.
- **PASS** — truncated, no terminate, end tilt (−1.5°, +2.0°), board stayed up.
- Note: ~147/150 “overruns” — each `step()` (SyncWrite + sense) often &gt;20 ms;
  loop is effectively ~30–40 Hz under load. Follow-up: slim tick or lower hz.
- `hexapod-web` restarted after test.

### balance_sine ±1° (hold-current, 25 Hz) — PASS
- Config: motion on, hold current, hz=25, amp=1°, period=6 s, 1.5 cycles/axis.
- Roll: (−1.6°, +2.0°) → (−1.51°, +1.94°), overruns 16/225.
- Pitch: (−1.4°, +2.1°) → (−1.44°, +1.98°), overruns 17/225.
- Board stayed up; no tip.
- **Caveat:** IMU barely moved (~0.1°) despite ±1° body cmds — IK may be
  ineffective from current non-plant pose, or sign/frame bug. Next: verify
  joint targets change under sine, or stand to real plant slowly then retest.
- Control hz 50→25 in `config.yaml` (SyncWrite+sense often &gt;20 ms).

### Pivot: feet not on ground (operator)
- Balance IK assumes planted feet; current pose had feet in air → sine could
  not move chassis IMU. Next step: slow `stand_to_plant` before more balance.

### stand_to_plant #1 (PSU ~3 A limit) — board drop
- 12 s blend from hip≈−65°/knee≈+36° toward plant. Progress ~33% then SSH
  reset; Uno Q unreachable. Likely PSU current-limit foldback.

### stand_to_plant #2 (PSU raised to ~10 A, 25 s blend) — tilt abort, board stayed up
- Start pose odd after reboot: L0 hip≈+15.5° / knee≈**−91.7°** (max |Δq|≈176°).
- Knee stuck ~−22° for ~8–40% of blend, then resumed; tilt grew and trip at
  **64%** (roll≈−12° / pitch≈−10°) → limp. Board remained reachable.
- Power-domain OK enough at 10 A for this attempt; geometry/start pose was
  the failure mode. Next: hand-set near plant or shorter Δq before blend.

### stand_to_plant #3 (after belly reset, 30 s) — tilt abort again
- Start L0 −38°/−36°, level IMU. Progressed smoothly to hip≈0° / knee≈+40°
  then roll spike → limp at **60%** (roll≈−12°). Board stayed up; web restarted.
- Pattern: large simultaneous stand from sprawled knees tips sideways before
  plant. Need hand-assist / hold chassis through last third, or stage knees
  first with body supported.

### Policy pivot (GPT + operator) — stop auto +20/+80
- +80° is knee axis max; experimentally tucks feet under and tips.
- Next: hand-set stable stance → `capture_plant` (full `joints_deg`) →
  balance starts with hold-current only; refuse if far from plant.
- `stand_to_plant` disabled without `--force` + captured plant.
- 25 Hz accepted; no 50 Hz chase for now.

### HTTP API + geometry plant (prefer over SSH)
- Added `GET/POST /api/rl/*` on hexapod-web + `rl_move/remote.py` client.
- **Incident:** unsupervised stand/plant with wrong zeros → tip/brownout /
  ~7 A stilts / one knee servo dead/hot (since replaced 08-09; bus 18/18
  healthy — RESOLVED, not an open issue). set-zero-here later fixed the
  frame (straight-out → logical 0°). Air ±15° nudges then matched encoders
  on live joints.
- Hardening: cursor rule + AGENTS.md; drive `C`/`P`/`#` Δq>25° refuse;
  `find_plant`/geometry require `force`; `enable_motion` default false.
---

## 2026-08-19

### Bus plumbing upgrade (code only — NOT yet flashed/verified on robot)

- External review (GPT, via operator): the 20-40 Hz ceiling was the
  synchronous request/response plumbing, not the 1 Mbps servo bus. Also
  found the USB `feetech_bus.py` still decoded present-speed with the
  SCS 0.732 rpm/unit convention (~50x inflation, fixed in the MCU path
  2026-08-07) — now fixed in both USB copies (`speed_counts_to_deg_s`).
- Firmware `feetech_bridge` STREAM mode: MCU free-runs acquisition
  (pos+speed syncRead + IMU every pass ~150-250 Hz; full current/load/
  volt/temp state at ~10 Hz) into RAM caches; `P`/`F`/IMUR/PWR/DX serve
  cached. New `S` binary op = SyncWrite + state snapshot in ONE host
  round trip (`n=0` = read-only snapshot). Host driver enables STREAM on
  open (`HEXAPOD_NO_STREAM=1` opts out); old firmware falls back to the
  legacy paths everywhere.
- `RobotStateEstimator.update()` uses the one-round-trip snapshot;
  `McuFeetechBus.step_all()` available for future gym-style loops.
  DriveController gait loop 20 → 50 Hz; per-joint pose reads bulked.
- rl_policy stays 25 Hz — the trained contract (config `control.hz`
  feeds the sim env dt). The "no 50 Hz chase" ruling above is now a
  training-side decision only; the hardware blocker is gone.
- Next bench session: flash `firmware/feetech_bridge`, then
  `python3 linux_control/bus_bench.py` (read-only) — expect
  read_snapshot well over 100 Hz and rl episode overruns 0.
