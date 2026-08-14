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
| `eval_model.py` | Gates G1 (legacy) + G1.1 (revised 2026-08-13, `--k1-ridge-tol`): held-out prediction vs persistence + linear-ridge baselines (information-matched to the model's input set); latent dump for probes/cluster analysis |
| `probe_latents.py` | G3 linear probes from dumped z: roll/pitch/gyro R², per-foot contact balanced accuracy, shuffled-target chance floor |
| `merge_shards.py` | Merge parallel per-seed collection subdirs (collect.py shard numbering races under concurrent writers); `--require-actor` guards against recipe drift |
| `sb3_encoder.py` | `DynFeaturesExtractor`: stacked env obs -> un-scale -> pretrained encoder -> z (+goal tail); `ScaledLRPPO` + `set_group_lrs` for condition C's slow encoder LR |
| `train_ppo_transfer.py` | The A/B/C comparison: scratch vs frozen-z vs anchored encoder; tasks hold/lower/walk/rise; eval CSV carries per-task gait/transition QUALITY metrics (slip, peak roll/rate, slew saturation, contact switching, height/dh, vx tracking) + optional `--eval-heldout` dynamics-mismatch suites (broad DR, latency, servo speed, deadband, torque) |
| `run_pilot.sh` | Local pilot cohort: hold from scratch, then lower warm-started (A/B/C, matched); run names are seed-suffixed (`pilot_hold_A_s0`) |
| `analyze_pilot.py` | Cross-seed aggregation of the pilot eval CSVs: steps-to-threshold, final returns, hold retention, optional per-seed curve plot |
| `pod_v3_pipeline.sh` | v3 drift-fix pipeline (stood down in favor of pod_pilot_rep2.sh — kept as the G1.1-gated variant) |
| `pod_pilot_rep2.sh` | Drift-fix replication on the INTENDED v2 recipe (v2pod2), gated on ORIGINAL G1 (no post-hoc weakening) |
| `pod_holdwalk.sh` | hold->walk transfer cohort (+ rise-retention canary + heldout suites in phase 2) |
| `pod_risewalk.sh` | rise->walk benchmark cohort — the operator's actual robot objective; rise retention is the first-class hypothesis |
| `pod_scale_sweep.sh` | Representation scaling matrix: S/M/L (~0.8/5.9/17M) x history 16/48 x 1200/4800-ep data, each cell gate-evaluated; aggregate with `analyze_scale.py` |
| `pod_chain_abc.sh` | Watcher that chains pod_holdwalk.sh behind the scale sweep: waits for POD_SCALE_SWEEP_DONE, picks the first gate-passing cell in a fixed pre-declared preference order, launches the cohort with fresh seeds (>= 5) |
| `datasets/` `models/` `logs/` | Generated (gitignored; also excluded from `snapshot.sh --sync` code tarballs) |

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

# pod-scale hold->walk transfer pair (operator directive 08-13;
# preconditions in the script header — G1-passed pod encoder +
# v2pod dataset + rep triage done):
SEEDS="1 2 3 4 5" nohup sh rl_move/dynamics/pod_holdwalk.sh \
    > rl_move/dynamics/logs/pod_holdwalk.log &
../../.venv/bin/python -m rl_move.dynamics.analyze_pilot \
    --seeds 1 2 3 4 5 --phase2 walk --phase2-threshold <thr> --plot
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
