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

## V1 build (historical GRU baseline)

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
  contact BCE; current and all future horizons → privileged simulator
  truths as auxiliary targets (true body-frame velocity, yaw rate,
  relative heading sin/cos, command refs, along/cross-command motion,
  chassis height); t+10, t+25 → future latent, target =
  `stop_gradient(encoder(future_history))`. Privileged truths are
  labels only, never encoder inputs. History window H=16 (25 Hz ⇒
  0.64 s).
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
- **G1.1 — revised gate (recorded PROSPECTIVELY 2026-08-13, operator
  next-steps directive).** The original G1 was too brittle: it let a
  ~2.7% loss to matched ridge at the nearly-linear one-step horizon
  veto an encoder that beat every baseline at k=2/5/10/25 — but the
  scientific question is whether the learned temporal state improves
  control, not whether a GRU can out-regress ridge on locally linear
  1-step servo motion. Revised criteria (implemented as
  `eval_model.py --k1-ridge-tol`, default 0.05, reported as
  `gate_g1_1_pass` alongside the legacy verdict):
  - shortest horizon (k=1): must beat persistence and be within 5%
    of the matched ridge MSE;
  - every other short horizon (k=2, k=5): must beat persistence AND
    ridge outright;
  - latent horizons (k=10, k=25): unchanged — beat unchanged-z.
  No seed fishing: the gate is evaluated once per pre-registered
  encoder run. Downstream control testing still requires matched
  A/B/C baselines regardless of the gate. **Historical verdicts are
  NOT changed retroactively**: the two `dyn_v2pod_obs` G1 FAILs
  (08-13, seeds 0/1) stand as recorded; G1.1 applies to encoders
  gated from this date forward.
- **G2 — deploy-contract variant**: the PPO-facing encoder must be
  trained with `--input-set obs` (59 policy-visible dims: q, qd, tilt,
  gyro, prev action). It must still pass G1 (the `full` input set is
  the ceiling reference, not the transfer candidate).
- **G3 — latent sanity**: dumped latents (`--dump-latents`) organize
  visibly around upright/fallen, contact configuration, tilt, true
  velocity/yaw-rate and command-frame motion before we claim
  "reusable body knowledge".

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

Beyond the sample-efficiency curves, three operator-priority tests
(08-13; full wording in `rl_docs/tracks/dynrep/STATUS.md` → Next):
(1) gait QUALITY of C vs scratch — loaded-foot slip, roll, contact
sequencing, slew saturation, currents, falls, videos — not merely
return; (2) the stand→walk handoff from the deployed standing state
at zero velocity (peak roll + simultaneous slew saturation in the
first second — a recurrent representation knows "I have just been
standing with six feet loaded"); (3) robustness to actuator/model
mismatch — A vs C under held-out randomized latency, servo speed,
compliance, and contact — the sim-to-real reason a world model
exists.

## Ablations (after the first comparison, one variable each)

z ∈ {64, 128, 256} before any deeper network. Do not assume extra
layers create better semantics.

**Scaling experiment (operator next-steps 08-13, supersedes the "keep
it small" cap for the PREDICTION study only):** S ~0.8M (current) /
M ~5.9M / L ~17M × history 16/48 × dataset 1200/4800 eps, all
`--input-set obs`, matched steps/optimizer (`pod_scale_sweep.sh`,
`analyze_scale.py`). The deliverable is the prediction scaling curve +
G3 probe quality; a larger encoder still earns PPO wiring only through
the normal gates and matched A/B/C. The likely bottleneck is data/task
diversity, not parameters — the matrix includes the data axis to test
exactly that.

## Do not do yet (v1 discipline)

**Superseded for phase-1 prediction on 2026-08-15 by the operator's causal
Transformer direction.** The current model is intentionally not reduced:
4 layers, width 512, 8 heads, FF 1024, z=256 (~13.6M parameters). The first
Transformer run overfit because 20.48M optimizer draws repeatedly sampled a
fixed corpus with only tens of thousands of highly overlapping valid centers;
the train/validation actor and mode coverage was not grossly broken. Current
production runs therefore generate GPU MJX/Warp trajectories until planned
window reuse is <=2x and the trainer mechanically refuses undersized data.

Still deferred: contrastive stacks, VAEs, a full Dreamer-style world model,
planning through the model, and one policy for every task at once.

## Logging

Pretraining: per-horizon train/val prediction loss, joint-state error,
IMU error, contact accuracy, latent error (CSV next to checkpoint;
optional W&B). PPO phase (later): reward, task success, falls,
tracking error, energy/current, old-task retention, steps-to-threshold
— the existing `SCORE/*` + eval harness conventions apply. Periodically
save latents from standardized trajectories for the organization
analysis (G3).
