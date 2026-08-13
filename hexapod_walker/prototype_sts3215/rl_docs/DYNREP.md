# DYNREP — dynamics-representation pretraining track (design doc)

Opened 2026-08-12 (operator brief: "Hexapod Dynamics Representation:
Concrete First Build"). Code: `rl_move/dynamics/` (README there is the
runbook). W&B tag `track:dynrep`, group `dynrep`, run prefix `cw-dyn-`.
Status: `rl_docs/tracks/dynrep/STATUS.md`.

## Hypothesis

A task-independent representation trained to predict how the robot
evolves under actions (self-supervised, action-conditioned) will
transfer better across standing, walking, steering, and recovery than
representations learned only from PPO rewards. The research question:

> Can a model first learn how this body behaves, and then reuse that
> knowledge to learn new motor skills faster?

The primary metric is **sample efficiency on a new task**.

## V1 build (deliberately small — no transformer)

- **Frame** (86 dims, `dynamics/frames.py` layout v2): joint pos/vel,
  tilt (episode-relative roll/pitch), gyro, IMU specific force, servo
  currents, per-foot touch forces, previous action. Extracted from the
  sim env AFTER its sensor-corruption path — never privileged ground
  truth. All continuous channels standardized so no channel dominates
  by scale. Joint positions are stored RELATIVE to the episode's
  settled start pose `q_nom` — the policy obs contract ((q−q_nom)
  with q_nom captured per episode at reset), so a pretrained encoder
  can be fed from the deployed observation; absolute q is not
  recoverable from the obs (v1 stored absolute q and was therefore
  unwireable — kept only as a lesson).
- **Model** (~1M params): frame MLP 256→256 (SiLU) → GRU 256 → latent
  z=128; future-action GRU conditions every prediction head
  (action-conditioned by construction, never `state_t → state_{t+k}`
  alone).
- **Horizons** t+1, t+2, t+5 → raw physical state (q, qd, IMU) MSE +
  contact BCE; t+10, t+25 → future latent, target =
  `stop_gradient(encoder(future_history))`. History window H=16
  (25 Hz ⇒ 0.64 s).
- **Data** (`dynamics/collect.py`): diversity of physical experience
  over task labels. Actor mix random-OU / scripted tripod / scripted
  noslip / stance champion / walk champion, each with noise variants;
  goal mixes spanning rise/lower/hold/lean/walk (belly, plant, crouch,
  mid-stride and park starts); DR scale ∈ {0, 0.3, 0.6, 1.0} (brings
  tipped starts, walk kick/push, rise rock). Failures are kept — the
  model should understand falling and slipping as well as success. No
  single champion may dominate the dataset.

## Gates (binding, in order)

- **G1 — beat trivial baselines** (`dynamics/eval_model.py`): on
  held-out windows, at EVERY horizon, the model must beat (a)
  persistence (predict current state unchanged) and (b) a linear ridge
  predictor from [history, future actions]. **If the neural model
  cannot beat these, do not move on to PPO.** For long horizons the
  bar is latent-prediction error below the "latent unchanged"
  reference, with the state-space baselines reported alongside.
- **G2 — deploy-contract variant**: the PPO-facing encoder must be
  trained with `--input-set obs` (59 policy-visible dims: q, qd, tilt,
  gyro, prev action). It must still pass G1 (the `full` input set is
  the ceiling reference, not the transfer candidate).
- **G3 — latent sanity**: dumped latents (`--dump-latents`) organize
  visibly around upright/fallen, contact configuration, and tilt
  before we claim "reusable body knowledge".

## First experimental comparison (after G1+G2)

Same env family, matched seeds and budgets; only the representation
changes:

- **A — PPO baseline**: raw observations → PPO, from scratch (the
  existing `train_ppo_sim.py` recipe, obs history 16 to match H).
- **B — frozen pretrained z**: obs/action history → pretrained encoder
  → frozen z → PPO actor/critic (only the policy head learns).
- **C — continually anchored**: as B but the encoder fine-tunes slowly
  (encoder LR ≪ policy LR, `ScaledLRPPO`) while the predictive
  objective keeps running. The most interesting condition — the
  dynamics loss is a task-independent anchor. V1 approximation
  (implemented): instead of the joint sum `L_PPO + λ·L_dynamics`, the
  anchor alternates — after every PPO rollout a few dynamics-loss
  gradient steps run on the OFFLINE pretraining dataset through the
  shared encoder. An online-window anchor (dynamics loss on the
  policy's own fresh rollouts) is the follow-up once the pilot says
  the condition is worth it.

Downstream tasks, in order: stand, forward locomotion, small yaw
steering, recovery/return toward zero pose. Measure: env steps to
threshold, final performance, variance across seeds, retention of
earlier tasks at every checkpoint. Transfer test: train stand →
checkpoint → train forward locomotion on identical budgets (later:
locomotion → self-righting); plot new-task performance vs additional
env steps while re-evaluating old tasks.

## Ablations (after the first comparison, one variable each)

z ∈ {64, 128, 256} before any deeper network. Do not assume extra
layers create better semantics.

## Do not do yet (v1 discipline)

Large transformer; 10+ layer MLP; huge latent; contrastive stacks;
VAEs; full Dreamer-style world model; planning through the model; one
policy for every task at once.

## Logging

Pretraining: per-horizon train/val prediction loss, joint-state error,
IMU error, contact accuracy, latent error (CSV next to checkpoint;
optional W&B). PPO phase (later): reward, task success, falls,
tracking error, energy/current, old-task retention, steps-to-threshold
— the existing `SCORE/*` + eval harness conventions apply. Periodically
save latents from standardized trajectories for the organization
analysis (G3).
