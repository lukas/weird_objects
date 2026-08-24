# dynamics — self-supervised dynamics-representation pretraining (track: dynrep)

Can a model first learn how this body behaves, and then reuse that
knowledge to learn new motor skills faster? The current phase-1 model is
a 13.6M-parameter causal Transformer trained on diverse simulator
rollouts: standing, walking, falls, random flailing, DR perturbations,
current/future servo state, contacts, velocity, and heading. Only after
it beats trivial baselines does it enter an A/B/C PPO transfer comparison.
Full design + gates: `rl_docs/DYNREP.md`.

| File | What it is |
|------|------------|
| `frames.py` | The canonical 86-dim deployable per-tick feature frame (layout v2) plus privileged target sidecar; extraction from a live sim env |
| `collect.py` | Rollout collector: 5 actor types x goal mixes x DR scales -> npz shards |
| `collect_mjx.py` | H200 MJX/Warp collector: thousands of simulator worlds, original five-actor/DR recipe, terminal-frame preservation, W&B throughput/freshness tracking |
| `fresh_pipeline.py` | Orchestrator entry point: collect until the optimizer reuse budget is met, then launch the unchanged full-size Transformer |
| `data.py` | Shard loading, whole-episode 80/10/10 train/validation/test split, train-only normalization stats, window sampler, and split-coverage checks |
| `model.py` | Causal Transformer (current default: 4 layers, width 512, 8 heads, FF 1024, z=256) -> current/future physical, privileged-truth, contact, current, and latent heads |
| `train.py` | CUDA pretraining loop; GPU-resident sampling, physical train/validation metrics and generalization-gap alarms in W&B, best-validation checkpoint, one-time test evaluation, and a hard planned-window-reuse gate |
| `eval_model.py` | Gates G1 (legacy) + G1.1 (revised 2026-08-13, `--k1-ridge-tol`) on test by default: held-out prediction vs persistence + linear-ridge baselines (information-matched to the model's input set); privileged-target diagnostics; latent dump for probes/cluster analysis |
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
| `pod_memwatch.sh` | Container-OOM guard (train-10 OOMKilled 08-14, whole overlay fs lost): logs memory.current + top-RSS process every 60s; above 85GiB kills the largest python so the pod survives |
| `check_cohort.py` | Mechanical status for script-owned transfer cohorts: reads the manifest, live `train_ppo_transfer` processes, and done markers. Use this before believing STATUS prose. |
| `datasets/` `models/` `logs/` | Generated (gitignored; also excluded from `snapshot.sh --sync` code tarballs) |

## Quick start (from `prototype_sts3215/`, repo `.venv`)

```sh
make -C rl_move/dynamics smoke      # tiny end-to-end sanity run (~3 min)

# pretraining pass (layout v2 — obs-contract relative q):
uv run python -m rl_move.dynamics.collect \
    --out rl_move/dynamics/datasets/v2 --episodes 400 --seed 0
uv run python -m rl_move.dynamics.train \
    --data rl_move/dynamics/datasets/v2 --name dyn_v2_obs \
    --input-set obs --steps 40000 --lr-final-frac 0.05
uv run python -m rl_move.dynamics.eval_model \
    --ckpt rl_move/dynamics/models/dyn_v2_obs.pt \
    --data rl_move/dynamics/datasets/v2 --dump-latents

# Production H200 path: generate enough fresh GPU-sim data for the exact
# optimizer budget, then train the full Transformer. Both stages use W&B.
uv run python -m rl_move.dynamics.fresh_pipeline \
    --name cw-dynrep-tf-state2-fresh --steps 40000 \
    --data rl_move/dynamics/datasets/v5_mjx_fresh \
    --batch 512 --history 16 --horizons 1,2,5,10,25 \
    --max-window-reuse 2 --collect-n-envs 2048 \
    --arch transformer --device cuda --input-set obs

# A/B/C transfer pilot (after G1 passes on the obs encoder):
sh rl_move/dynamics/run_pilot.sh 150000 0      # [steps] [seed]
uv run python -m rl_move.dynamics.analyze_pilot \
    --seeds 0 1 2 --plot

# pod-scale hold->walk transfer pair (operator directive 08-13;
# preconditions in the script header — G1-passed pod encoder +
# v2pod dataset + rep triage done):
SEEDS="1 2 3 4 5" nohup sh rl_move/dynamics/pod_holdwalk.sh \
    > rl_move/dynamics/logs/pod_holdwalk.log &
uv run python -m rl_move.dynamics.analyze_pilot \
    --seeds 1 2 3 4 5 --phase2 walk --phase2-threshold <thr> --plot

# verify a script-owned cohort mechanically, never from prose:
uv run python -m rl_move.dynamics.check_cohort --cohort holdwalk
uv run python -m rl_move.dynamics.check_cohort --cohort risewalk
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
The `priv` sidecar stores supervised simulator-only labels such as true
body velocity, yaw rate, relative heading, command velocity and
along/cross-command motion. These labels are targets and diagnostics
only; they are not included in any encoder input set.

## Rules of the road

- **Do not connect PPO until gate G1 passes** (`eval_model` prints
  PASS/FAIL): the model must beat persistence AND the linear predictor
  at every horizon on held-out test windows. This is the brief's hard gate.
- The split is a stable whole-episode 80/10/10 hash. Normalization and
  optimization use train only; checkpoint selection uses validation only;
  test is evaluated only after selecting the checkpoint. Training refuses a
  corpus whose validation or test split omits any collected actor, DR level,
  or mode.
- Transfer cohorts currently run through pod scripts, not
  `launch_run.py`; until they are launcher-wired, every launch must
  write `rl_move/dynamics/logs/<cohort>_manifest.jsonl`, and a cycle
  may call it launched only after `check_cohort.py` sees either live
  `train_ppo_transfer` processes or a final `done` event.
- The PPO encoder can only use the `obs` input set (59 policy-visible
  dims). Train the transfer candidate with `--input-set obs`; the
  default `full` set (currents, contacts, accel) is for the
  representation-quality ceiling and analysis.
- Do not shrink the Transformer to compensate for data starvation. A
  production run may draw at most 2x as many optimizer windows as distinct
  train-window centers unless an explicit smoke/debug override is present.
- No VAE, Dreamer-style imagination, or planning through the model yet.
- Don't let one champion dominate collection — the default actor mix
  exists to model the hexapod's dynamics, not one policy's trajectory
  distribution. Failures (tips, early terminations) are kept on
  purpose.
