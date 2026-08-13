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

## Now

- Track opened 08-12 (operator brief); V1 pipeline landed in
  `rl_move/dynamics/` and the first full pass ran the same day.
- Dataset `datasets/v1` (local, gitignored): 1,200 episodes / 258k
  steps (~2.9 sim-hours), 3 collect seeds; actor split random 354 /
  walk-champion 282 / stance-champion 239 / tripod 197 / noslip 128;
  357 episodes end in falls/trips (kept on purpose); DR ∈
  {0, 0.3, 0.6, 1.0}.
- **G1 PASS for both v1 models** (20k steps, ~0.8M params, H=16,
  eval on 124 held-out episodes, baselines information-matched to the
  model's input set — reports in `rl_move/dynamics/logs/`):
  - `dyn_v1` (full 86-dim input): state MSE 0.116/0.151/0.171 at
    k=1/2/5 vs persistence 0.268/0.398/0.715 and linear
    0.127/0.170/0.194; latent MSE 0.092/0.108 at k=10/25 vs
    unchanged-z 0.80/1.03. Joint-pos RMSE 1.7-2.0 deg, contact acc
    0.92-0.95.
  - `dyn_v1_obs` (**G2**, 59 policy-visible dims): also PASS —
    0.149/0.175/0.191 vs matched linear 0.153/0.190/0.210. NOTE the
    k=1 margin over linear is thin (~3%); the representation's value
    shows at k≥5 and in the latent horizons, as expected (1-step
    dynamics is locally near-linear).
- G3 first probe: linear probes from z recover roll/pitch at
  R² 0.97/0.98 and feet-on-ground count at R² 0.57 — the latent
  visibly organizes around attitude + support configuration.

## Next

- A/B/C PPO wiring design (the G1 precondition is met): sb3
  features-extractor wrapping the frozen (B) / slowly fine-tuned +
  dynamics-anchored (C) `dyn_v1_obs` encoder over the env's
  `obs.history_frames=16` stack; A = matched from-scratch baseline.
  First tasks per DYNREP.md: stand → forward locomotion transfer.
- Grow the dataset before the PPO phase (more seeds, more DR-heavy
  and getup episodes) and retrain `dyn_v1_obs` longer — the thin k=1
  linear margin should widen with data before we lean on the encoder.
- G3 proper: per-foot contact probes + 2D embedding of standardized
  trajectories (upright/fallen, tipping direction, gait phase).
- Latent-size ablation (64/128/256) only after the first A/B/C
  comparison lands.
