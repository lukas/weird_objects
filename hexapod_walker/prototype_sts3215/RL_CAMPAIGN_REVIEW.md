# Hexapod RL — overnight campaign review (2026-08-08)

Self-contained status brief for external review. Covers the 20-run experiment
campaign run Fri Aug 7 ~6:30 PM → Sat Aug 8 ~6:45 AM PT on 4 CoreWeave GPU pods.
Companion docs: `RL_PLAN_NEXT.md` (current plan), `rl_move/API.md`.

## 1. System context

- **Robot:** 18-DOF hexapod (6 legs × hip-yaw / hip-pitch / knee), Feetech
  STS3215 serial-bus servos, Arduino Uno Q controller, IMU on body. Control
  loop 25 Hz.
- **Sim:** MuJoCo 3.11 twin (`rl_move/sim/sim_env.py`) with calibrated servo
  model (latency, rate limit, dead-band, torque→current estimate), hinged foot
  pads, IMU complementary filter, domain randomization (geometry, masses, IMU
  mount, friction, actuator params, start-pose jitter) scaled by a single
  `dr_scale` knob (0 = nominal, 1 = full).
- **Policy:** PPO (SB3), MLP. **Action space = 18 raw joint-position targets**
  (the earlier 6D body-IK action space was abandoned — it was noise-fragile:
  its flat-rise choreography only survived at action std ≤ 0.05, so PPO
  rationally unlearned it under normal exploration noise).
- **Goal-conditioned, one policy, modes:** hold / lean / track (attitude
  targets), unload (leg), raise (small body lift), rise (belly-flat → stand;
  starts sampled from flat / bridge / crouch), lower (stand → belly), walk
  (commanded body velocity).
- **Reward:** kernel-based task tracking is the only large positive term; weak
  penalties on gyro, action delta, current. Recent additions (this campaign):
  per-servo hot-current penalty (`k_current_hot`, soft threshold
  `current_hot_a`), stance contact bonus (`k_stance_contact`), dense stance
  clearance penalty (`k_stance_clearance`, pays per mm a hovering foot is above
  its episode-start height; **exempted for raise mode** after it collapsed
  raise), walk swing touchdown bonus (`k_walk_swing`), configurable walk
  command range (`walk_speed_min/max_m_s`) and progress gain (`k_walk_prog`).
- **Evaluation:** exact-path harness (`rl_move/sim/eval_checkpoint.py`):
  6 episodes/mode, deterministic AND stochastic passes, evaluated at the run's
  own DR level; logs per-servo currents (peak, p95, hottest servo, time above
  soft limit, cross-leg imbalance = max-leg mean current / mean), gait metrics
  (per-foot duty cycle, swing count/length, slip, forward distance), videos
  with telemetry overlay. Success gates: rise/lower/raise = height error
  threshold at episode end; walk = mean commanded-velocity tracking error
  (gate ≤ 0.03 m/s); hold/lean/track = degrees of attitude error.
- **Training pattern:** warm-start lineage with W&B run stitching; 48 parallel
  envs per pod; periodic in-training deterministic evals every 100–200k steps.

## 2. Where each track stands

### Track A — stand ↔ belly (SOLVED, full DR)

DR ladder from the `cw-lower-smooth2` base (DR 0.2), warm-starting each rung:

| Rung | Run | Result (6 eps/mode, det + stochastic) |
|---|---|---|
| DR 0.5 | `cw-stand-dr05` | PASS — rise 6/6 (flat/bridge/crouch), lower 6/6 |
| DR 0.8 | `cw-stand-dr08` | PASS |
| DR 1.0 | `cw-stand-dr10` | PASS — **hardware candidate** |

Every rung passed on the first attempt. Unlike the old body-IK line, the
raw-joint rise is robust to the policy's own sampling noise.

### Track B — even stance / current distribution (mostly solved)

Problem: all policies had learned a 3-leg tripod hold — three feet never
touched (duty cycle ~0.01), concentrating current (imbalance 1.77–2.2,
hottest servo >1.5 A for 5–7 s of a 10 s hold).

| Run | Change | Result |
|---|---|---|
| `cw-stance-even` | `k_current_hot=0.2` + `k_stance_contact=0.5` | Hot time halved (3.7 → 1.5–2.4 s) but tripod unchanged — the contact bonus is gradient-dead (a hovering foot earns nothing until it actually touches) |
| `cw-stance-clear` | + `k_stance_clearance=10.0` (dense, per-mm) | **Tripod broken** — all six feet load, min-leg duty 0.01 → ~0.31, imbalance → 1.36. But raise collapsed to 0/6 (penalty punished the lift) |
| `cw-stance-raisefix` | Exempt raise mode from clearance penalty | PASS full gate — raise 5/6 recovered, six-foot stance kept, rise/lower 6/6, imbalance ~1.37 (best-ever single episode 1.16) |
| `cw-stance-dr08` | DR ladder 0.8 | PASS — hold 6/6 six-footed, raise 5/6, rise 5–6/6, lower 6/6 |
| `cw-stance-dr10` | DR 1.0 — **running now** | Latest periodic eval on pace: rise 2/2 all starts, lower 2/2, hold 0.50° |

### Track C — walking (progressing; the hard one)

Baseline `cw-walk2` (10M steps): a "skate" — feet stay planted, body shuffles,
velocity tracking error 0.064 m/s, 0/6 tracking success. Campaign chronology
(deterministic harness vel err, m/s):

| Run | Change | vel err | Verdict |
|---|---|---|---|
| `cw-walk2-gait` | swing touchdown bonus `k_walk_swing=1.0` | 0.064 | Gait improved (stride 12→23 mm, slip −20%, 2× forward distance) but tracking unchanged, 0/6 |
| `cw-walk-fresh-gait` | fresh init, same reward (ablation) | 0.075 | Fresh policy converged to the same skate → the warm-start prior is NOT the trap; the objective/curriculum was |
| `cw-walk-slow` | narrow commands to 0.02–0.06 m/s (were 0.03–0.12) | 0.036 | **First real tracking gain** — commands became reachable |
| `cw-walk-prog3` | 3× progress reward (`k_walk_prog`) at original range | 0.065 | Refuted — most motion of any run, zero tracking gain, 0/6 |
| `cw-walk-slow2` | consolidate at 0.02–0.06 | 0.032 det / **0.028 sto, 5/6** | **Gate hit** (≤0.03). Champion for slow range |
| `cw-walk-curr08` | widen straight to 0.02–0.08 (from slow, unconsolidated) | 0.043 | Failed — consolidate-then-widen confirmed |
| `cw-walk-w08` | widen consolidated slow2 → 0.02–0.08 | 0.041 | Regressed (1/6), rise eroded to 2/6 — full-size widen too big |
| `cw-walk-w08-s1` | seed-1 twin | — | **INVALID** — bit-identical weights to w08 (seed bug, see §4) |
| `cw-walk-dr04` | DR 0.2→0.4 at slow range | 0.035 det / 0.031 sto, 3/6 | Near-miss (gate 4/6 at ≤0.03) |
| `cw-walk-w07` / `-s1` | half-size widen 0.02–0.07, 2 real seeds — **running** | 0.037 / 0.030 in-flight | Seeds genuinely diverge post-fix |
| `cw-walk-dr04b` | consolidate DR 0.4 — **running** | 0.037 in-flight | Flat-rise flapping (0/2 last periodic eval) |

Current best walking: a deliberate ~3–4 cm/s gait that tracks slow commands.
Nothing tracks ≥0.08 m/s commands yet.

## 3. Key lessons this campaign

1. **Dense gradients beat sparse bonuses.** The clearance-per-mm penalty broke
   the tripod in one run where the contact bonus did nothing. Same lesson as
   the earlier curl ratchet: if exploration can't feel reward before the
   behavior is complete, PPO won't find it.
2. **Curriculum beats reward re-pricing for walking.** Making commands
   reachable (speed curriculum) produced the only tracking gains; tripling the
   progress reward produced motion without tracking.
3. **Consolidate before widening, and widen in small steps.** slow → slow2
   crossed the gate; both 0.02→0.08 widens regressed. Currently testing 0.07.
4. **Multi-task interference is real.** The 70% walk mix erodes rise (flat-rise
   flaps 0/2–2/2 in walk-line evals). May need mix rebalancing or merging
   skills by distillation later.
5. **The eval harness catches what W&B scalars can't.** Duty cycles exposed
   the tripod; per-mode splits exposed the raise collapse; weight checksums
   exposed the seed bug.

## 4. Process bug found & fixed: `--seed` was a no-op on warm starts

`cw-walk-w08` and its "seed-1 twin" finished 5M steps with **bit-identical
weights** (max param diff 0.0). Cause: SB3 `PPO.load()` restores the
ancestor's seed and `learn()` re-seeds torch + envs from `model.seed`,
silently ignoring the `--seed` CLI arg. Every prior warm-started "seed
comparison" in this project was two copies of one run. Fixed by overwriting
`model.seed = args.seed` after load; the round-5 twins now genuinely diverge
(0.037 vs 0.030 m/s in-flight).

## 5. Known weaknesses / open risks

- **Raise never fully solves** — 4–5/6 across every lineage. The 10–30 mm
  body-lift reward appears marginal; worth a focused look.
- **Peak currents brush the breaker.** All policies show 2.5–2.7 A peaks
  (hardware breaker: 2.5 A sustained). Sustained hot time improved, but
  **torque→current calibration must be re-validated under MuJoCo 3.11** (the
  2.3.7→3.11 upgrade shifted quiet-hold peaks 2.46→2.60 A) before trusting any
  current numbers on hardware.
- **Walk uses privileged body velocity** in observations (sim-only). Deploy
  path needs frame-stacking, a recurrent policy, or teacher–student
  distillation — not yet started.
- **Rise erosion in walk runs** (interference, §3.4).

## 6. In flight right now (round 5, ~3 h in, 4 pods)

| Pod | Run | Question | Latest periodic eval |
|---|---|---|---|
| friction | `cw-stance-dr10` | Even-stance line at full DR 1.0 | rise 2/2 all starts, lower 2/2, hold 0.50° |
| s3 | `cw-walk-w07` | Does a half-size widen (0.02–0.07) hold the gate? | walk err 0.037 |
| s4 | `cw-walk-w07-s1` | True seed twin — curriculum reliability | walk err 0.030 |
| long5m | `cw-walk-dr04b` | Consolidate walk at DR 0.4 | walk err 0.037; flat-rise 0/2 |

## 7. Decisions we'd like a second opinion on

1. How to widen the walk speed range without regression — smaller rungs
   (0.065, 0.07, …), command-conditioned curriculum inside one run, or accept
   the slow gait and pivot to robustness/deploy?
2. How to stop walk-training from eroding rise — goal-mix rebalancing, EWC-style
   regularization toward the stand champion, or train separately and distill
   into one student?
3. Whether the raise weakness matters (it's a subset of rise) or should be
   dropped as a mode.
4. When to take `cw-stand-dr10` / `cw-stance-dr10` to hardware, and what the
   minimum extra validation should be given the MuJoCo 3.11 current-calibration
   drift.
