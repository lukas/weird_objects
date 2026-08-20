# RECOVER MODE — deployment package + runner contract + blocker list

Created 2026-08-20 (operator order, MCP operator lane
20260820T222458Z: "reopen the recover/tangle gate and make the
recovery model usable/production-ready"; sim/deploy readiness only).
This is the recover-mode analog of DOWNLOAD_ANSWER.md — an ADDITIONAL
operator-requested mode; it does NOT alter the rise+walk answer.

## The package (what to download for recover)

| Piece | Artifact | Integrity |
|---|---|---|
| Recovery policy | `rl_move/sim/policies/ppo_goal_cw_recover_predictive1b_pop3_s13.zip` | md5 `cba811e6a1e63b4b043d65f755356fb4` |
| Frozen dynamics encoder (REQUIRED at load) | `rl_move/dynamics/models/cw-dynrep-tf-state2-recovered1.pt` | md5 `9df48f687967c25085ee50171e4110ff` |
| Loader + runner (obs contract, mode gating) | `rl_move/sim/recover_runner.py` (`load_recover_policy`, `RecoverRunner`) | contract test `rl_move/tests/test_recover_runner.py` |
| Deployment sim gate | `rl_move/sim/eval_recover_runner.py` | reports `logs/recover_runner_gate/` |

Candidate selection: **s13** over s11/s12 — only member whose matched
gate eval recovered flip on own-DR det (s11: flip stuck near-inverted
on own-DR; s12: flip missed both passes); all three clear the other
22 rungs. Ledger verdicts cw-recover-predictive1b-pop3-s1[123].

Load ONLY through `load_recover_policy()`: the SB3 zip pickles the
ABSOLUTE train-pod encoder path in `policy_kwargs`, so plain
`PPO.load` fails off-pod; the loader rewrites it to the local encoder
copy and verifies both md5s.

## Observation/runner contract (test-locked, bit-exact vs training env)

- 16 frames x 90 dims newest-first (1440 total); frame =
  [59 proprio | 11 goal zeros | 2 vel zeros | 18 plant-relative q].
- q_nom = entry encoder read (hold-current); plant-relative q =
  (q - q_plant)/q_scale; prev_action = last CLIPPED action, zeros
  until the first policy tick.
- tilt reference is LEVEL (calibrated IMU bias only, default (0,0))
  — never re-anchor at the fallen attitude (opposite of the stance
  runner's re-anchor-at-entry).
- Entry = manual command only; REFUSED while gyro not quiet
  ("recover after it lands"). Then a 15-tick (0.6 s) ENTRY HOLD
  commanding the captured pose fills the history with REAL frames
  (the trained reset-history probe); the first policy action fires on
  exactly that stack.
- Actions: 18 absolute joint targets ([-1,1] -> AXIS_LIMITS via
  `action_to_q_rad`) through the SafetyLayer. Recover mode NEEDS the
  widened tilt envelope (`safety.max_roll_deg=185`,
  `max_pitch_deg=185`) — a fall is a recoverable state. Current
  limits unchanged. Active budget 16 s (trained horizon), then
  TIMEOUT + freeze on last safe command.
- Exit: deploy-side detector (level <=6 deg + gyro quiet + joints
  quiet + pose near plant, held 0.5 s) -> hand off to STANCE HOLD
  (the session STOP route). Detector stats below.

## Deployment-runner sim gate (2026-08-20, train-9, n=23 rungs/pass)

Ladder driven end-to-end through `RecoverRunner` (runner-built obs
from raw sensor reads, deploy-honest q_nom + level tilt ref):

- **DR-0 det: 21/23** — identical to the champion's matched
  training-path gate (21/23), same two misses: `zero` is the
  DOCUMENTED quiet-hold-timing false negative (clean stand: detector
  fires @2.0 s, end tilt 0.14 deg), `flip` is the genuine weak rung.
- **own-DR 0.1 det: 22/23** — everything recovers except flip
  (ends inverted, 179 deg). Typical recovery time 3-5 s, end tilt
  <=1.5 deg across all successful rungs.
- **flip isolation: 0/6 at own-DR** (seeds 0-2, deploy vs
  `--parity-qnom` IDENTICAL) — the deployment contract deltas are NOT
  the cause; flip is intrinsically unreliable in this checkpoint
  (s13's single own-DR flip success in the 08-18 gate was a favorable
  draw; s11/s12 also fail flip).
- Detector agreement 17/23 (DR-0) / 19/23 (own-DR): silent on the 4
  instant rungs that succeed during the entry hold (benign);
  correctly fires on the zero false-negative; ONE false fire on DR-0
  flip (fired @7.4 s on a transiently level partial recovery that
  then leaned to 14.7 deg — see blocker 3).

## BLOCKER LIST to production (ordered)

1. **[operator] Flip rung unsupported.** 0/6 own-DR, misses DR-0,
   weakest across all three seeds. Ship recover with flip explicitly
   out of envelope (operator rights a fully-inverted robot by hand),
   or order a flip-hardening arm (respec from s13) before shipping it.
2. **[operator] Hardware safety contract for recover mode.** The
   trained 185/185 deg tilt envelope must be mirrored on the robot
   (tilt kill-switch relaxed ONLY inside recover mode, current
   limits + operator kill switch unchanged) — a bench/firmware
   decision, parked until the robot is back.
3. **[RESOLVED 08-20 same cycle] Handoff detector hardening.** The
   0.5 s detector false-fired once on a transiently-level DR-0 flip
   partial recovery; hold raised to 1.0 s and re-verified: no false
   fire on flip, all true recoveries + the zero correct-fire intact
   (fires 1.8-3.2 s into the stand). Keep as a bench monitoring item.
4. **[operator/bench] On-robot compute + IMU calibration.** The
   frozen transformer runs per-tick (16x86 encode); needs torch (or
   an ONNX export) on the deploy computer at 25 Hz — unmeasured on
   the board. Level-IMU bias calibration is required for the tilt
   reference (a stale bias shifts every tilt obs). Both bench items.
5. **[operator] Current pricing.** Trained with default current
   regularizers; the standing hardware rule (`reward.k_current=0`
   until pricing calibrated) was a training-arm rule, but recover's
   servo currents during untangling are untested on hardware —
   treat first bench reps as current-limited experiments.

Known non-blockers: `zero` rung DR-0 "miss" is an eval-scoring
artifact (video + detector + end-tilt all confirm clean stands);
STO 0/23 on the 08-18 gates is the same documented strict-hold
artifact, not a behavior defect.
