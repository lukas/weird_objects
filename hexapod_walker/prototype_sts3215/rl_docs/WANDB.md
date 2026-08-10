# W&B — how this project uses Weights & Biases

One project holds the whole campaign: **`l2k2/hexapod-balance`**
(https://wandb.ai/l2k2/hexapod-balance). Every training run logs there
automatically (group `ppo-goal-lineage`); `--no-wandb` disables.
Defaults live in `rl_move/sim/train_ppo_sim.py`
(`WANDB_ENTITY_DEFAULT` / `WANDB_PROJECT_DEFAULT`, env-overridable).

## Credentials

- The key lives in **`rl_move/sim/wandb.env`** (gitignored). It exists
  on the operator Mac AND on every train pod (pushed to
  `/workspace/prototype_sts3215/rl_move/sim/wandb.env` by
  `bootstrap_train_pod.sh`).
- `ops.sh` sources it for you. For ad-hoc API use, source it first:

  ```sh
  set -a; source rl_move/sim/wandb.env; set +a
  python -c "import wandb; ..."   # repo .venv has wandb installed
  ```

- A bare `kubectl exec` has NO creds — `launch_run.py drain` (dedupe
  check) and anything calling `wandb.Api()` fails without sourcing.

## Read runs with ops.sh — don't hand-write API code

| Question | Command |
|---|---|
| One run: state, steps, reward trend, URL | `ops.sh wandb <run>` |
| Full triage read (ledger + W&B + evals + videos) | `ops.sh review <run>` |
| Cache summary/config/history to the run's experiment dir | `ops.sh wandbdump <run>` (then query the cache, not the API) |
| Any results being dropped? | `ops.sh triage [hours]` — recent W&B runs × verdict × note × processed |
| What's ACTUALLY training | `ops.sh census` — W&B lags fresh launches ~8 min (JAX compile); an "empty" project is normal right after a launch |

For genuinely ad-hoc queries: `wandb.Api()` filtered by
`display_name`, take the newest match (names can have retry suffixes).

## What's on a run page

- **Name = ledger run name** (e.g. `cw-walk-foo-c1`); checkpoint is
  `policies/ppo_goal_cw_walk_foo_c1.zip` (dashes→underscores). Names
  are **append-only** — the launcher refuses reuse, so retries get
  `-r1`/`-rr1` suffixes. A 0-step run is an infra launch failure
  (collision/shm), not science.
- **Config** carries the full spec + DR ranges + `parent_run`; notes
  carry the spec text. SB3 scalars (losses, `ep_rew_mean`, fps) arrive
  via `sync_tensorboard`; reward components, eval panels (`eval/*`),
  rollout videos (`video/rollout`), and canary metrics log directly.
- **`--- OUTCOME ---` note = the verdict.** `launch_run.py update
  --set verdict=…` auto-mirrors the ledger verdict to the run notes
  and attaches an `analysis-<run>` artifact (ledger entry, run doc,
  every eval file existing at verdict time — so run evals BEFORE
  verdicting). `ops.sh wandbnote <run> "…"` writes a richer paragraph.
- **Lineage = the artifact DAG.** Each run publishes
  `ckpt-<out-name>` (type policy-checkpoint, md5 + parent in
  metadata) and declares its `--init-from` parent via `use_artifact`.
  Continuations also `fork_from` the parent run at its end step.
  Pre-08-09 runs predate this and appear rootless.

## Gotchas

- A short "resume blip" on a finished run's timeline is the
  artifact-attach mechanism (`wandb.init(id=…, resume="allow")`), not
  a training restart.
- Local `wandb/` run dirs are **gitignored** — they were tracked once
  and poisoned every pod sync (08-09). Never re-add them.
- W&B run state is not ground truth for "is it training" (`ops.sh
  census` is) nor for verdicts (the ledger / `rl_docs/runs/` is).
