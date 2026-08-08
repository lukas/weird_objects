# STS3215 Hexapod — RL Plan
## Freeze recovery, curriculum walking, hardware-realistic observations

**Status:** Current plan (post overnight-campaign external review)
**Date:** 2026-08-08
**Robot:** `prototype_sts3215` — 18× STS3215, Uno Q, MPU-6050
**Package:** `rl_move/`

This is the **single** RL plan document. It merges the 2026-08-08 external
(GPT) review with the state of the project after the overnight CoreWeave
campaign. Predecessor docs are preserved in [`archive/`](archive/):

- `archive/RL_PLAN.md` — Phase-1 plan + full 2026-08-06/07 status log
  (bring-up, sys-ID, sim calibration, reward design history).
- `archive/RL_PLAN_NEXT.md` — the raw-joint forward plan that drove the
  campaign (visual-eval harness spec §2, current-distribution metrics §4,
  gait metrics §5, hardware candidate gate §8).
- `archive/RL_CAMPAIGN_REVIEW.md` / `archive/RL_CAMPAIGN_REVIEW_2026-08-08.md`
  — the 20-run overnight campaign briefs (run-by-run hypotheses/verdicts).
- `archive/HEXAPOD_RL_EXTERNAL_REVIEW_2026-08-08.md` — the external review
  this plan implements.

Hardware safety rules are unchanged and non-negotiable: see §10 below and
`.cursor/rules/hexapod-sts-hardware-safety.mdc`.

---

## 0. Where we are (evidence, not aspiration)

The overnight campaign (20 W&B runs, 4 CoreWeave pods, Aug 7–8) settled the
framing question: **raw-joint PPO can learn useful hexapod behavior.** The
project now holds two distinct assets to be treated separately:

- **Recovery/control (Track A+B — solved, near hardware-ready):**
  stand ↔ belly (rise from flat/bridge/crouch + lower) passes 6/6
  deterministic + stochastic at **full DR 1.0** (`cw-stand-dr10`). The
  even-stance line (`cw-stance-raisefix` → `dr08` → `dr10`) additionally
  broke the learned 3-leg tripod hold: all six feet load, cross-leg current
  imbalance 2.2 → ~1.37. Every DR-ladder rung passed first try.
- **Locomotion (Track C — emerging, curriculum signal found):** best walk
  is a deliberate ~3–4 cm/s gait that hit the tracking gate (0.028 m/s
  stochastic vel err, gate ≤0.030) at 0.02–0.06 m/s commands
  (`cw-walk-slow2`). Both attempts to widen straight to 0.08 m/s regressed;
  reachable commands were the only lever that ever improved tracking
  (3× progress reward was cleanly refuted). Nothing tracks ≥0.08 m/s yet.

Key campaign lessons (keep operating on these):

1. **Dense gradients beat sparse bonuses** — the per-mm stance-clearance
   penalty broke the tripod where the contact bonus did nothing.
2. **Curriculum beats reward re-pricing** for walking.
3. **Consolidate before widening; widen in small steps.**
4. **Multi-task interference is real** — walk-heavy mixes erode rise.
5. **The exact-path eval harness catches what W&B scalars can't**
   (tripod, raise collapse, seed bug). Video review is mandatory and
   systematic: W&B `video/rollout` entries are multi-episode reels
   cycling every active goal mode (`--video-episodes`, ~40 s), and each
   `eval_checkpoint.py` run emits a per-mode `contact_sheet.png`. Every
   gate decision walks the per-mode checklist in
   `rl_move/sim/VIDEO_REVIEW.md`.

Known weaknesses / open risks:

- **Seed bug (found + fixed):** `--seed` was a no-op on warm starts (SB3
  `PPO.load` restores the ancestor's seed); every prior warm-start "seed
  comparison" was two copies of one run. Fixed by overwriting `model.seed`
  after load. Historical multi-seed claims need repair (§8).
- **Sim current is uncalibrated** since the MuJoCo 2.3.7 → 3.11 upgrade
  (quiet-hold peaks drifted 2.46 → 2.60 A with no physical change). Policies
  show 2.5–2.7 A peaks right at the 2.5 A hardware breaker — do not trust
  those numbers until recalibrated (§2).
- **Walk observes privileged body velocity** (sim state) — hardware cannot
  provide it (§6).
- **Raise** (10–30 mm body lift) is stuck at 4–5/6 everywhere (§5).

---

## 1. Freeze the stand/recovery champion (`STAND_V1`)

When `cw-stance-dr10` completes its gate, freeze it as a **versioned,
immutable hardware candidate** (`STAND_V1`). Do not continue training that
checkpoint with walking — walking has repeatedly eroded rise, and the
known-good recovery policy becomes the reference other work is measured
against.

Before hardware, run a **much larger frozen-policy evaluation** instead of
more PPO:

- 500–1,000 episodes per important start condition (flat / bridge / crouch
  / stand-for-lower), deterministic AND stochastic, full DR 1.0.
- **Explicit parameter sweeps and corners**, not just random DR: friction,
  actuator strength, actuator latency, servo rate limits, geometry, IMU
  bias/noise/mount error, start-pose perturbations — and **combinations of
  adverse parameters** (random DR undersamples dangerous corners like
  low friction + weak actuator + high latency).
- Report **distributions and worst cases**, not means. Auto-save
  telemetry/video for the worst episodes.
- Keep reporting per-servo current, hot time, imbalance, joint speeds.

Corner testing is a hardware gate, in addition to the existing candidate
gate carried from the previous plan (clean exact-path visual eval at DR 0
and DR 0.2, ≥20-episode success with zero safety terminations, no
systematic hot leg, conservative per-joint deployment envelopes derived
from successful sim trajectories, low SafetyLayer intervention frequency,
insensitivity to hardware-level obs noise).

## 2. Recalibrate torque → current (blocker)

The MuJoCo 3.11 physics drift moved quiet-hold predicted peaks ~0.1 A with
no physical change, and simulated policies sit at 2.5–2.7 A peaks near the
2.5 A breaker. **Revalidate the torque→current mapping before trusting any
simulated current number as a hardware safety gate**, and before any
aggressive hardware rise testing. This is a prerequisite for §1's gate and
§10's hardware plan.

## 3. Walking: adaptive speed curriculum (not reward escalation)

The strongest walking result: raising the progress reward did nothing;
making commands achievable dramatically improved tracking. Curriculum is
the axis. Rules:

- **No more large range jumps** (no 0.02–0.06 → 0.02–0.08 style widens).
- Implement an **adaptive speed curriculum** — sample commands roughly:
  - 70% from the mastered range (e.g. 0.02–0.06 m/s),
  - 20% near the current frontier (e.g. 0.06–0.07),
  - 10% slightly beyond it (e.g. 0.07–0.08),

  and shift probability upward only when the frontier bucket becomes
  reliable.
- Make **speed-conditioned evaluation a first-class metric**: per
  speed-bucket curves/tables of commanded vs achieved speed, tracking
  error, success rate, stride length, swing count, slip, current, and
  stability — not a single mean velocity error. The quantity that matters
  is the location and movement of the learning frontier.
- Gait-quality metrics from the eval harness (duty cycle, swing, slip,
  forward distance) remain mandatory — reject policies that make speed by
  dragging, slipping, or body oscillation.

## 4. Multi-task interference: measure before regularizing

Walk-heavy training erodes rise. Do **not** jump to EWC. In order:

1. **Simpler task-mix changes first** — rebalance so rise/lower/stand get
   substantial continued training while walking progresses.
2. **Measure whether the objectives actually conflict**: estimate policy
   gradients on separate walk and rise batches and compute
   `cos(g_walk, g_rise)`. Frequent strong opposition = genuine destructive
   interference; mild = insufficient rehearsal.
3. Let that result pick the architecture strategy:
   - modest interference → keep the shared goal-conditioned policy, fix
     the sampling;
   - severe interference → specialist policies, or a shared trunk with
     mode-specific heads, with optional distillation later.

There is no requirement that the robot run one monolithic network. A
high-level state machine selecting a frozen recovery controller vs a
locomotion controller may be safer and easier to validate.

## 5. Deprioritize `raise`

Do not spend compute pushing raise from 5/6 to 6/6 unless investigation
shows the failure indicates a broader defect — belly → stand is already a
more demanding height-changing behavior. Keep raise in evaluation as a
diagnostic probe; it must not drive training decisions.

## 6. Hardware-realistic walking observations — start now

This is one of the largest deployment blockers: walking observes true
simulated body velocity, which hardware cannot supply. Do not pour compute
into a policy whose observation interface can't exist on the robot.

Begin a hardware-realistic observation line immediately:

1. **Frame stacking / history first**, using only hardware-available
   quantities: joint positions, joint velocities (if reliably estimable),
   IMU orientation, IMU gyro, previous actions, commanded velocity.
   ~200–400 ms of history initially.
2. If frame stacking is insufficient, test a **recurrent policy
   (GRU/LSTM)**. For this problem, prioritize memory over raw MLP size —
   velocity must be inferred from temporal observations.
3. The existing privileged policy becomes a **teacher**:
   - teacher: joint state + IMU + true body velocity → actions;
   - student: joint/IMU/action history → actions;
   - behavior cloning / distillation combined with the normal RL objective
     (scaffolding exists in `distill_joint_policy.py`).

**The key next walking milestone is a policy whose observation vector can
actually be constructed on the robot** — not another few thousandths off
the velocity error.

## 7. Model capacity / architecture ablation

Reasonable to test now, but as an ablation, not the primary strategy. Hold
environment, curriculum, reward, and budget fixed; compare roughly the
current MLP, ~4× parameters, and ~10–16× parameters (e.g. 256×256 vs
512×512 vs 1024×1024 or a deeper 512 net), with **multiple genuine seeds**.

Measure: final walking performance, sample efficiency, seed variance,
**rise retention**, gait quality, policy entropy, value loss, current/jerk.
A particularly interesting outcome would be capacity reducing walk↔rise
interference — that would mean capacity is limiting multi-task learning.
Architecture may matter more than parameter count: also consider the shared
trunk + mode-specific heads if §4's gradient measurements support it.

## 8. Repair seed confidence

Do not rerun the historical experiment tree. Do **replicate the
foundational claims with genuinely independent seeds** (3 each):

1. full-DR stand/rise/lower champion,
2. six-foot even-stance champion,
3. slow-walk champion.

From now on every run records enough to detect accidental seed reuse:
requested CLI seed, `model.seed`, Torch seed, env seeds, parent checkpoint
checksum, initial + final policy checksums, git SHA. Independent-seed
experiments must **fail loudly** if RNG/model-init state indicates
duplication.

## 9. Compute allocation (next round, 4 pods)

Once the currently running round finishes:

| Pod | Experiment |
|---|---|
| 1 | **Stand validation** — massive frozen `STAND_V1` DR/corner/worst-case eval (§1) |
| 2 | **Adaptive walking curriculum** — main experiment (§3) |
| 3 | **Adaptive walking curriculum, independent seed** — reliability check |
| 4 | **Hardware-realistic walking obs** — frame-stack/student line, no privileged velocity (§6) |

Model-size/architecture ablations (§7) enter the queue as pods free up.
Keep using pods for architecture-level questions, not micro reward tweaks;
multi-seed only after a config wins.

## 10. Hardware strategy

**No autonomous deployment of experimental policies to the robot — ever.**
All existing safety rules stand: set-zero-here before absolute poses,
operator present and explicitly authorizing each session, HTTP not SSH,
25 Hz, limp on anomaly, stop after tip/brownout/hot motor/missing ID.

Once `STAND_V1` passes the expanded validation (§1) **and** current
calibration is trustworthy (§2), begin conservatively. The first hardware
test is **not** belly → stand. With the robot placed in a normal,
supportable standing configuration, verify:

- joint/action mapping and IMU sign/frame conventions,
- deterministic inference,
- SafetyLayer behavior,
- current behavior,
- stability under tiny commanded corrections.

Expand the envelope only after these agree with simulation. Keep the
recovery/get-up policy and experimental walking policies **separately
promotable**, so walking research can never silently modify the known-good
hardware recovery controller.

## 11. Research process (keep doing what worked)

The campaign's process was the valuable output as much as the scores: it
falsified the warm-start-trap hypothesis, falsified progress-reward
re-pricing, discovered the tripod pathology, found the dense reward that
fixed it, discovered the seed bug, and established the speed curriculum.
Continue operating this way:

- For every significant experiment keep a log entry with: observation /
  problem → hypothesis → experiment designed to distinguish it → result →
  interpretation → next decision
  (`rl_move/orchestrator/EXPERIMENT_LOG.md`).
- Prefer experiments that distinguish between competing explanations; do
  not run experiments just because a hyperparameter hasn't been tried.
- Standing practices (hard-won): a checkpoint is not a result until it
  passes the exact-path visual eval (`rl_move/sim/eval_checkpoint.py`);
  gate on the stochastic harness eval, never W&B return curves; split
  rise/lower stats by start kind; keep-best champions per skill, never
  keep-last; champions are read-only files; warm-start + one variable per
  run where possible.

## 12. Definition of done for this phase

**A walking policy using only hardware-available observations that walks
robustly in simulation, alongside a separately frozen and validated
recovery policy (`STAND_V1`) capable of reliably taking the robot from
belly to a safe six-foot stance.** Protect the first asset (recovery) while
aggressively investigating the second (locomotion).

---

## Appendix A — Sim ↔ real facts (carried from the Phase-1 plan)

Key facts for anyone doing MuJoCo / transfer work. Verified 2026-08-06.

### A.1 Stand plant mismatch (the #1 sim↔real gap)

| Context | Hip | Knee | Approx foot drop |
|---|---|---|---|
| **Live robot stand / plant** (default or `plant_pose.json`) | **+20°** | **+80°** | ~157–159 mm (feet under body) |
| **CAD / MuJoCo / `sts/` RL posture** | **−25°** | **+60°** | ~35 mm (feet out, crouch) |

Align plant height before training anything for transfer, or policies
trained on the crouch will fight the real stand. Positive hip = femur
toward the floor on hardware.

### A.2 Motors / bus

- STS3215, ~12 V / 30 kg·cm; half-duplex TTL @ 1 Mbps; 4096 counts/rev,
  centre 2048 (≈11.378 counts/deg).
- `WritePosEx` position + speed + acc. **speed=0 means MAX** — drivers
  coerce to hold speed 250.
- Feedback available per servo: pos, speed, load %, bus V, temp,
  current (~6.5 mA/LSB). No foot-force sensors, no external encoders.
- Soft torque limit SRAM addr 48 (0–1000).

### A.3 Rates

| Path | Rate |
|---|---|
| Robot teleop walk (`DriveController`) | 20 Hz SyncWrite, **open-loop** (no per-tick read-back) |
| RL loop target | 25 Hz today, 50 Hz once tick ≤ 20 ms |
| MuJoCo timestep | 500 Hz (0.002 s) |
| Bulk position read ×18 | ~197 Hz ceiling; full feedback ×18 ~77 Hz |

### A.4 MuJoCo model fidelity (`mujoco_prototype.py`)

- Link lengths match CAD (coxa 12.5 / femur 90 / tibia 128 mm; chassis
  hex 200 mm flat-to-flat).
- Collision = chassis box + foot spheres; meshes visual-only. Sim foot
  *touch* sensors are privileged (not on the real bus).
- Lumped mass ~2.1 kg in sim vs ~1.3 kg in CAD text — **mass is
  uncertain; weigh or identify before trusting dynamics.** This is what
  the motor-dynamics / sys-ID probe feeds.
- Actuator torque ceiling ≈ 2.70 N·m (near nameplate stall).

### A.5 Existing walk RL stack (`../sts/`) — do not hijack

Residual PPO on top of `TripodGait` (57-dim base obs + STS motor
channels; per-joint residual ≈ ±0.06 rad). Checked-in artifacts are
~4k-step smoke runs, not trained policies. It still uses the −25°/+60°
crouch and privileged contacts; treat as a separate track from
`rl_move/`.

### A.6 Doc conflicts to ignore

- Stand “−25°/+60°” in older READMEs / WIRING → superseded by
  `DEFAULT_STAND_* = +20/+80` on the robot.
- “No MCU bridge / Linux owns USB” wiring blurbs → superseded by
  `feetech_bridge` + `mcu_feetech_bus.open_feetech_bus()`.
- `sts/posture.py` comment claiming its stand matches
  `feetech_bus.standing_pose` → false today (matches MuJoCo crouch).
