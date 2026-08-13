# dynamics — self-supervised dynamics-representation pretraining (track: dynrep)

Can a model first learn how this body behaves, and then reuse that
knowledge to learn new motor skills faster? V1: a small
action-conditioned multi-horizon predictor (frame MLP + GRU + 128-dim
latent, ~1M params) trained on diverse saved simulator rollouts —
standing, walking, falls, random flailing, DR perturbations — then
(only after it beats trivial baselines) an A/B/C PPO transfer
comparison. Full design + gates: `rl_docs/DYNREP.md`.

| File | What it is |
|------|------------|
| `frames.py` | The canonical 86-dim per-tick feature frame (layout v1), extraction from a live sim env |
| `collect.py` | Rollout collector: 5 actor types x goal mixes x DR scales -> npz shards |
| `data.py` | Shard loading, episode-hash train/val split, normalization stats, window sampler |
| `model.py` | Frame MLP(256,256, SiLU) -> GRU(256) -> z(128) -> short-horizon raw-state heads + long-horizon latent heads |
| `train.py` | Pretraining loop; per-horizon train/val logging (CSV + optional W&B), best-val checkpoint |
| `eval_model.py` | Gate G1: held-out prediction vs persistence + linear-ridge baselines (information-matched to the model's input set); latent dump for cluster analysis |
| `sb3_encoder.py` | `DynFeaturesExtractor`: stacked env obs -> un-scale -> pretrained encoder -> z (+goal tail); `ScaledLRPPO` + `set_group_lrs` for condition C's slow encoder LR |
| `train_ppo_transfer.py` | The A/B/C comparison: scratch vs frozen-z vs anchored encoder, dual-task eval CSV (retention for free), offline dynamics-anchor callback |
| `run_pilot.sh` | Local pilot cohort: hold from scratch, then lower warm-started (A/B/C, matched); run names are seed-suffixed (`pilot_hold_A_s0`) |
| `analyze_pilot.py` | Cross-seed aggregation of the pilot eval CSVs: steps-to-threshold, final returns, hold retention, optional per-seed curve plot |
| `datasets/` `models/` `logs/` | Generated (gitignored) |

## Quick start (from `prototype_sts3215/`, repo `.venv`)

```sh
make -C rl_move/dynamics smoke      # tiny end-to-end sanity run (~3 min)

# pretraining pass (layout v2 — obs-contract relative q):
../../.venv/bin/python -m rl_move.dynamics.collect \
    --out rl_move/dynamics/datasets/v2 --episodes 400 --seed 0
../../.venv/bin/python -m rl_move.dynamics.train \
    --data rl_move/dynamics/datasets/v2 --name dyn_v2_obs \
    --input-set obs --steps 40000 --lr-final-frac 0.05
../../.venv/bin/python -m rl_move.dynamics.eval_model \
    --ckpt rl_move/dynamics/models/dyn_v2_obs.pt \
    --data rl_move/dynamics/datasets/v2 --dump-latents

# A/B/C transfer pilot (after G1 passes on the obs encoder):
sh rl_move/dynamics/run_pilot.sh 150000 0      # [steps] [seed]
../../.venv/bin/python -m rl_move.dynamics.analyze_pilot \
    --seeds 0 1 2 --plot
```

Training applies a one-time reward penalty on early termination
(`--term-penalty`, default 30) — without it the pilot's low-budget hold
task collapses into "tip over at tick ~35" for every condition
(measured 08-12). Evals always run the raw env, so eval CSVs stay
comparable across runs with different penalties.

Collection runs the real MuJoCo env stack (plain python, NOT mjpython)
and needs the two local champion checkpoints in
`rl_move/sim/policies/` (already pulled; if one is missing its episode
share falls back to random actions with a warning). Datasets are
append-safe: re-running `collect` with a new `--seed` adds shards.

## Rules of the road

- **Do not connect PPO until gate G1 passes** (`eval_model` prints
  PASS/FAIL): the model must beat persistence AND the linear predictor
  at every horizon on held-out windows. This is the brief's hard gate.
- The PPO encoder can only use the `obs` input set (59 policy-visible
  dims). Train the transfer candidate with `--input-set obs`; the
  default `full` set (currents, contacts, accel) is for the
  representation-quality ceiling and analysis.
- Keep v1 small on purpose: no transformer, no VAE, no Dreamer-style
  imagination, no planning through the model (see DYNREP.md
  "do not do yet").
- Don't let one champion dominate collection — the default actor mix
  exists to model the hexapod's dynamics, not one policy's trajectory
  distribution. Failures (tips, early terminations) are kept on
  purpose.
