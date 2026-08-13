# dynrep — Dynamics-representation pretraining

W&B: tag `track:dynrep`, run prefix `cw-dyn-`. Design doc + binding
gates: **rl_docs/DYNREP.md** (read before triaging anything here).
Code + runbook: `rl_move/dynamics/README.md`.

**Goal:** learn a task-independent latent of the hexapod's body
dynamics by self-supervised action-conditioned multi-horizon
prediction on diverse sim rollouts (failures included), then test
whether PPO reusing that representation (frozen / continually
anchored) learns new motor skills with fewer env steps than PPO from
scratch. Primary metric: sample efficiency on a NEW task.

## Now (08-12 late: PPO wiring + v2 + pilot)

- **Frame layout v2** (breaking, re-collected + retrained same night):
  q is stored RELATIVE to the episode's settled `q_nom`, because the
  policy obs contract is (q−q_nom) with q_nom captured per episode at
  reset — an encoder pretrained on absolute q (v1) can never be fed
  from the deployed obs. `ep_qnom` kept in shard metadata.
- **Residual short-horizon heads** (delta-state parametrization,
  model.py): the obs-input variant was losing to the full-history
  linear baseline at k=1 by ~2-3% (1-step dynamics is locally
  linear). Predicting the state DELTA from the newest frame makes
  persistence the head's zero output; k=1 joint-pos RMSE dropped
  1.7 deg -> 0.32 deg. **G1 PASS at every horizon for BOTH v2 models**
  (`dyn_v2`, `dyn_v2_obs` 40k steps + cosine LR decay; reports in
  `rl_move/dynamics/logs/`). G2 (obs input set) met.
- **A/B/C wiring landed** (`sb3_encoder.py`, `train_ppo_transfer.py`,
  `run_pilot.sh`): scratch vs frozen-z vs slow-LR encoder + offline
  dynamics anchor. Dual-task eval CSV at every eval point = retention
  curves for free.
- **sb3 gotcha (cost a debugging round, keep forever):**
  `ActorCriticPolicy(ortho_init=True)` (the default) orthogonally
  re-initializes every Linear INSIDE a custom features extractor
  after construction — it silently wiped the pretrained encoder
  (GRUs survive, so it looked half-trained). Fix:
  `DynFeaturesExtractor.reload_pretrained()` after fresh PPO
  construction; the anchor callback now prints the untouched anchor
  loss at start (must match pretraining val, ~2.0) as a tripwire.
- **Pilot cohort DONE (single seed, directional only)** — hold from
  scratch 150k, then lower warm-started 150k, matched everything;
  eval CSVs `logs/ppo_pilot_*_eval.csv`:
  - Phase 1 (hold) is a degenerate testbed at this budget: ALL three
    conditions found the tip-early attractor (no alive bonus ->
    ending the episode beats holding badly; ep_len collapses to ~35).
    Known reward-family attractor, not a condition discriminator —
    pod-scale phase should use the campaign's hold stack or a
    survival-gated variant.
  - Phase 2 (lower transfer) is where the signal is. Steps to lower
    return ≈200: **B ~50k, C ~110k, A ~117k** — the frozen
    pretrained z acquired the new task ~2.3x faster than scratch.
    Final lower return: **C 279 > A 260 > B 228**. Hold retention at
    end (return / early-term): **B 68.5 / 0.00, C 36.0 / 0.00,
    A 23.6 / 0.25** — both pretrained conditions kept the old task
    terminate-free, scratch didn't.
  - C's anchor loss stayed 2.04-2.19 throughout (pretraining val
    ~2.0): PPO fine-tuning never pulled the encoder off the
    predictive objective.
  - Reading: consistent with the track hypothesis (B fastest
    acquisition + best retention; C best final performance). One
    seed, 4 eval episodes/point — needs seed replication before any
    verdict label.

## Current numbers (v2, 08-12 night)

- Dataset `datasets/v2` (local, gitignored): 1,200 episodes / ~258k
  steps (~2.9 sim-hours), 3 collect seeds; actor mix random /
  walk-champion / stance-champion / tripod / noslip; ~30% of episodes
  end in falls/trips (kept on purpose); DR ∈ {0, 0.3, 0.6, 1.0}.
  (`datasets/v1` = retired absolute-q layout; delete freely.)
- `dyn_v2` (full input): k=1/2/5 state MSE 0.110/0.153/0.178 vs
  persistence 0.268/0.399/0.717 and matched linear 0.128/0.172/0.196;
  latent MSE 0.154/0.165 at k=10/25 vs unchanged-z 1.06/1.27.
- `dyn_v2_obs` (the PPO transfer candidate): 0.140/0.168/0.188 vs
  matched linear 0.154/0.191/0.213 — PASS everywhere. k=1 joint-pos
  RMSE 0.32 deg, tilt 0.14 deg, contact acc 0.93.
- G3 first probe (v1-era, re-run pending on v2): linear probes from z
  recover roll/pitch at R² 0.97/0.98, feet-on count R² 0.57.

## Next

- Seed replication of the pilot (3+ seeds; `run_pilot.sh <steps>
  <seed>`) before claiming anything. Then pod-scale budgets for the
  brief's real task ladder (stand → forward walk → yaw → recovery) —
  local Mac budgets cannot reach walking, and phase 1 needs a
  non-degenerate hold stack.
- Online-window dynamics anchor for C (currently offline-replay).
- G3 proper on v2: per-foot contact probes + 2D embedding of
  standardized trajectories (upright/fallen, tipping direction, gait
  phase).
- Latent-size ablation (64/128/256) only after the first A/B/C
  comparison lands.
