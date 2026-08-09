# COMMANDS — how to run everything (60 seconds, saves 20 minutes)

Distilled from mining every prior cycle transcript (2026-08-09): the
same commands were re-derived, and the same mistakes re-made, dozens
of times. `rl_move/orchestrator/ops.sh` implements the common
operations — use it. Sibling docs: `rl_docs/README.md` (index),
`rl_docs/GOAL.md` (plain-English mission), `rl_docs/EXPERIMENT_LOGS.md`
(per-run summary.md convention).

**STANDING RULE — promote what you figure out:** if a command failed,
was slow, or took several tries before you got it right, add it as an
`ops.sh` subcommand (or a snippet below) IN THE SAME CYCLE, and note
it in `rl_docs/README.md` if it changes what a file covers. Never
leave the next agent to rediscover it.

## Which question → which command (don't mix these up)

| Question | The ONE command |
|---|---|
| **Triage a finished run (START HERE)** | `ops.sh review <run>` — ledger+gate, W&B trend, eval table, video paths in one shot |
| Eval numbers table from a report.json | `ops.sh report <run\|path>` (per-episode + medians + term counts) |
| What is actually training right now? | `ops.sh census` (/proc truth; W&B lags launches ~8 min) |
| How many slots are free / where? | `python3 rl_move/orchestrator/capacity.py` |
| Ledger + procs + watcher, one screen | `ops.sh status` |
| One run's metrics/state | `ops.sh wandb <run>` (ledger: `ops.sh entry <run>`) |
| What's queued to launch? | `launch_run.py backlog list` |
| A past run's story | `rl_docs/runs/<run>.md` |
| Are results being lost/ignored? | `ops.sh triage [hours]` |
| Finished but not yet analyzed? | ledger `triage` field (watcher-stamped: `awaiting…` → `in-cycle…` → `done` on verdict); shown on the status page "Analysis pipeline" |
| Write the cycle's RL_LOG line | `ops.sh logline "c<N>: …"` — the ONLY way; never `cat >>` RL_LOG |
| Frames from a video | harness already wrote `*.png` sheets; else `ops.sh frames <mp4> [n]` |
| Operator wants an overview in a browser | status page at http://127.0.0.1:8090 — full setup/restart runbook in "Operator status page" section below |

**DO NOT hand-write python for any row above.** Transcript mining
(08-09) found >500 ad-hoc snippets re-parsing experiments.json,
report.json, and the W&B API for exactly these questions.

## Where things are (the #1 recurring failure: wrong paths)

- Controller repo root: `/workspace/weird_objects`. Cycles start HERE.
- PROTO dir: `/workspace/weird_objects/hexapod_walker/prototype_sts3215`
  — `cd` here first; every doc/tool path below is relative to it.
  (9/9 failures of `cat rl_move/orchestrator/guardrails.yaml` were
  agents trying orchestrator paths from the repo root.)
- `/workspace/prototype_sts3215` on the CONTROLLER is a STALE copy
  (its ledger is a symlink to the real one). Never work there.
- On TRAINING PODS the code IS at `/workspace/prototype_sts3215`
  (no git there; the `.code_sha` marker is the only version record).
- Docs: `RL_PLAN.md` + `RL_LOG.md` are CONDENSED (~120 lines each,
  read them whole — no more sed windows). Full history:
  `archive/RL_LOG_FULL_2026-08-09.md`, `archive/RL_PLAN_FULL_*.md`.
  Log append rule: ONE line per cycle via `ops.sh logline` — never
  `cat >>`; evidence goes to the ledger verdict + W&B, not the log.

## ops.sh (rl_move/orchestrator/ops.sh) — use instead of hand-rolling

- `ops.sh status` — active runs + live procs per pod + watcher tail.
- `ops.sh procs <pod>` — training/eval processes. **Pods have NO
  `ps`** — this scans /proc; agents rediscovered that trick 10+ times.
- `ops.sh census` — every train pod's live trainer from /proc. THE
  ground truth for "what is running": W&B lags fresh launches by up
  to ~8 min (JAX compile) and looks empty when things are fine.
- `ops.sh trainlog <run> [n]` — tail the run's train log on its pod
  (pod + log path come from the ledger; don't guess).
- `ops.sh entry <run>` — the run's ledger entries.
- `ops.sh wandb <run>` — state, steps, reward-quarters trend, std, URL.
- `ops.sh review <run>` — the standard triage read in ONE command
  (ledger, W&B, eval table, videos). If its output + one video answer
  pass/fail, record the verdict and stop.
- `ops.sh report <run|report.json>` — per-episode table + medians +
  gait_valid/termination counts from a harness report.
- `ops.sh logline "text"` — the only sanctioned RL_LOG write (one
  timestamped line under the git lock).
- `ops.sh frames <mp4> [n]` — contact sheet from any video (but the
  harness already writes `walk_*.png` sheets next to eval videos —
  check those first).
- `ops.sh pullckpt <run>` — fetch checkpoint from its pod + md5.
- `ops.sh pushckpt <pod> <ckpt>` — copy a checkpoint TO a pod + md5
  both sides. **`snapshot.sh --sync` EXCLUDES policies/** — a
  warm-start parent must be pushed explicitly or the run dies at
  init with FileNotFoundError (killed cw-walk-longdist, 08-09).
  If `kubectl cp`/exec-stdin streams keep dropping (websocket
  close/broken pipe — hit train-2/3, 08-09 c35): HTTP-serve from
  the controller (`python3 -m http.server 8765` in policies/) and
  on the pod `python3 -c "import urllib.request; urllib.request.
  urlretrieve('http://10.0.0.46:8765/<zip>', '<dest>')"` — pods
  have no curl/wget. Always md5 after.
- `ops.sh evalcmd <run>` — prints the exact-path harness eval command
  with the run's own `--cfg-set`s pulled from the ledger.
- `ops.sh waitlog <file> <regex> [timeout]` — poll for completion.
  **`sleep 60; tail …` is BLOCKED by the harness** — use this.
- `ops.sh expdir <run>` — create `logs/experiments/<run>/` with the
  summary.md template. Only for DIG-IN runs (08-09 lightweight
  process): clear pass/fail needs just the ledger verdict (which
  auto-renders `rl_docs/runs/<run>.md`) + `wandbnote`.
- `ops.sh wandbdump <run>` — cache the run's W&B summary/config/
  history into its experiment dir (query the cache, not the API).
- `ops.sh triage [hours]` — "is anything lost/ignored?" table: every
  recent W&B run × ledger verdict × OUTCOME note × watcher processed
  flag. Run it whenever the operator asks if results are being
  dropped (they asked twice on 08-09; this answers in 5 s).
- `ops.sh drain` — place backlog onto free pods, detached + creds
  sourced. Raw `launch_run.py drain` needs W&B creds (dedupe check)
  and takes minutes PER LAUNCH by design (two-phase verify waits out
  the pod's JAX/Warp compile) — never run it attached to a terminal
  you might close. The watcher auto-drains too, but NOT while PAUSEd
  (e.g. during restart_watcher.sh).
- `ops.sh killrun <run>` — kill a run's procs on its pod. Pods have
  no pkill, and a naive /proc scan KILLS ITSELF (your scan's cmdline
  contains the run name — a kill command suicided this way 08-09).
  Then record it: `launch_run.py update … status=KILLED verdict=…`.

## Hard-won gotchas (each cost a cycle at least once)

0. **Any untracked NON-doc file under the prototype tree (e.g. a fresh
   `rl_move/sim/park_banks/*.npz` from `harvest_park_states`) marks
   every `snapshot.sh --sync` `-dirty` and the launcher then REFUSES
   ALL drain launches** — 4 GPUs idled behind one uncommitted 26KB
   npz while its specs burned 3 attempts each into `backlog_failed.json`
   (08-09 c51). Commit (`snapshot.sh <name>`) right after generating
   any training-input artifact, BEFORE queueing specs that need it.
   Requeue after fixing: move items backlog_failed→backlog under
   `backlog.json.lock` with attempts reset, then `ops.sh drain`.

0b. **A controller eval can DEADLOCK on a corrupt ffmpeg pipe**
   (log shows `corrupt input packet` / `Invalid buffer size`, then
   the python's utime freezes with no children — hit the groundtilt5
   dr0ret pass, 08-09 c60, under heavy concurrent-eval load). Detect:
   output dir stops growing AND `cat /proc/<pid>/stat` utime is
   static across 5 s. Fix: kill the pid and rerun the pass with
   `--no-video` (metrics JSON is what gates need; frame strips from
   the hung run remain usable).

1. **`eval_checkpoint` runs ONLY as a module** from the PROTO dir:
   `python3 -m rl_move.sim.eval_checkpoint …`. Running the .py path
   dies on relative imports. Flags (stop re-running --help):
   `checkpoint --task {goal,joint_goal,joint_walk} --modes … --per-mode N
   --dr-scale F --seed N --episode-seconds S --stochastic
   [--end-posture-gate] [--no-video|--video-every N] --out DIR
   --cfg-set k=v (repeatable)`.
   **Driving candidates additionally need the JOYSTICK GATE**
   (`python3 -m rl_move.sim.eval_drive <ckpt> --dr-scale 0.2 --out
   FILE.json [--cfg-set …]`): scripted direction panel + randomized
   instant-flip stress; zero in-envelope falls = exit 0. The generic
   harness only samples the training distribution — it can't prove
   direction coverage or flip robustness (backforth lesson, 08-09).
2. **Launch ONLY via `launch_run.py launch`.** Raw `kubectl exec …
   nohup train_ppo…` hits the 2-minute exec timeout (looks dead,
   actually launched → ledger drift; this caused a real incident).
   Ledger edits ONLY via `launch_run.py update`.
3. **Eval with the run's OWN cfg** (`ops.sh evalcmd`): evaluating with
   default cfg silently drops the run's reward package and voids the
   verdict (`lowent-dr03` burned 4M steps on this class of mistake).
4. **kubectl needs `KUBECONFIG=~/.kube/coreweave.yaml`** (ops.sh sets
   it). `kubectl exec` dying mid-command does NOT mean the remote
   command died — check before re-running anything with side effects.
5. **snapshot → sync → launch, in that order, same cycle.** The
   launcher refuses a pod whose `.code_sha` ≠ local HEAD (also
   `-dirty`). **Never `--sync` with a dirty tree**: it stamps the
   pod marker `<sha>-dirty`, which can NEVER equal HEAD, so every
   later launch on that pod is refused until you commit
   (`snapshot.sh <name>`) and re-sync (cost cycle 51 four refused
   drain launches, 08-09). Snapshotting AFTER launching leaves the pod a commit
   behind and blocks the watcher's auto-continue (cycle 33 incident).
   After `snapshot.sh <run>`, run `snapshot.sh --sync <pod>` for the
   pod you're about to launch on.
6. **Checkpoint naming:** run `cw-walk-foo-c1` ⇒
   `rl_move/sim/policies/ppo_goal_cw_walk_foo_c1.zip` (dashes→
   underscores). Always record + compare md5 when pulling.
7. **W&B:** project `l2k2/hexapod-balance`; creds already in the
   cycle env. Prefer `ops.sh wandb <run>`; for ad-hoc queries use
   `wandb.Api()` filtered by `display_name`, newest match.
8. **git:** `snapshot.sh` serializes commit/tag/push under a lock —
   never raw `git push` for cycle edits; a brief wait on its lock is
   normal. Re-read RL_LOG/RL_PLAN right before editing (concurrent
   cycles append too).
9. Guardrails: `rl_move/orchestrator/guardrails.yaml` (from PROTO).
   Watcher log: `/workspace/orchestrator.log`. Cycle logs:
   `/workspace/cycle_logs/`. Per-run train logs live ON THE POD at
   `/tmp/train_<run>.log`.
10. **Restart the watcher ONLY via `restart_watcher.sh`** (in the
    orchestrator dir; deployed at `/workspace/restart_watcher.sh`).
    It PAUSEs, waits for in-flight cycles, sanity-parses the new
    code, then swaps the tmux session. Killing the watcher/tmux
    directly murders in-flight cycles, which only write their
    output at exit — 3 cycles' tokens were torched this way on
    08-09 and their runs got re-triaged from scratch (and the
    operator's assistant repeated the exact mistake later the same
    day — READ THIS LIST before touching infrastructure).
11. **Verdicts auto-mirror to W&B notes AND package the analysis
    artifact.** `launch_run.py update --set verdict=…` pushes the
    verdict under the `--- OUTCOME ---` marker on the run's W&B page
    and attaches `analysis-<run>` (type run-analysis) to the run:
    ledger entry, `rl_docs/runs/<run>.md`, and every
    `logs/ckpt_eval/<run>_*` + `logs/experiments/<run>/` file
    (report.json, contact sheets, videos). So run your harness evals
    BEFORE setting the verdict — files that exist at verdict time are
    what gets archived. `ops.sh wandbnote` can still replace the note
    with a richer paragraph. History: all 87 pre-existing verdicted
    runs were backfilled 08-09 ~15:20Z — DON'T re-backfill. Early-
    campaign artifacts legitimately hold 0 eval files (outputs were
    already deleted); probe-*/smoke-* runs have no artifact at all
    (no W&B run to attach to). Mechanism note: the public API can't
    create artifacts on a finished run, so the code briefly
    `wandb.init(id=…, resume="allow")`s it — the extra short resume
    blip on a run's timeline is this, not a training restart.
12. **Checkpoint lineage lives in W&B artifacts** (08-09): every
    training run publishes `ckpt-<out-name>` (type
    policy-checkpoint, md5 + parent in metadata) and declares its
    `--init-from` parent via `use_artifact`. The W&B artifact DAG is
    now the run/checkpoint family tree — pre-08-09 parents predate
    this and appear rootless.
13. **`killrun` leaks /dev/shm segments; the NEXT launch on that pod
    dies at first env reset with worker `EOFError`** (c54: two 0-step
    corpses on train-4 after c53's stopgo35 kill — /dev/shm 64M was
    98% full of `hexmjx-*` segments). Diagnose: `kubectl exec <pod>
    -- df -h /dev/shm`. Fix: confirm no live trainer (mind the
    self-match gotcha), then `rm -f /dev/shm/hexmjx-*` and relaunch
    under a NEW name (`-r1`): W&B names are append-only, the launcher
    refuses reuse. Eval completion marker is `artifacts` (the harness
    never prints WROTE) — `waitlog ... 'artifacts|Traceback'`.

14. **Batch-eval shell footgun (c60):** `CFG="..." && nohup A $CFG & nohup B $CFG &`
    puts the assignment INSIDE the first background job's subshell — B
    (and later jobs) run with an EMPTY $CFG, i.e. default cfg = silently
    voided verdicts (gotcha 3), detectable as cmd_dist/30s outside the
    walk band in report.json. Assign the variable on its OWN line (or
    `export` it), THEN background the evals; always spot-check one
    `/proc/<pid>/cmdline` for the cfg-sets after launching a batch.
    Related (c60/c61): controller evals get load/OOM-killed SILENTLY
    under heavy contention (empty log, no Traceback, no report.json,
    partial video files) — verify the pid in /proc before trusting
    `waitlog`, and relaunch (setsid helps).

## Operator status page (web) — setup & restart runbook

One auto-refreshing HTML page for the human operator: watcher
ON/PAUSED/OFF, in-flight cycles + what they're triaging, analysis
pipeline (ledger `triage` field), per-pod fleet census, backlog,
ledger runs, Claude token usage + est. spend, log tails. Code:
`rl_move/orchestrator/status_server.py` (stdlib only, port 8090 —
5183/5173 are BuildViz, 8080 is the robot).

Two pieces, both must be up:

1. **Server, on the controller pod** (`hexapod-sweep-friction`), in
   tmux session `statusweb`:

   ```sh
   kubectl --kubeconfig=$HOME/.kube/coreweave.yaml exec hexapod-sweep-friction -- \
     bash -c "tmux kill-session -t statusweb 2>/dev/null; \
       tmux new-session -d -s statusweb 'source /root/orchestrator.env; \
       cd /workspace/weird_objects/hexapod_walker/prototype_sts3215 && \
       python3 rl_move/orchestrator/status_server.py 2>&1 | tee /tmp/status_server.log'"
   ```

2. **Port-forward, on the operator's laptop** (dies on sleep/network
   blips — restart it freely, it's stateless):

   ```sh
   kubectl --kubeconfig=$HOME/.kube/coreweave.yaml \
     port-forward hexapod-sweep-friction 8090:8090
   ```

Then open **http://127.0.0.1:8090** (raw data at `/json`).

Health checks: `curl -s http://127.0.0.1:8090/ | head -c 100` on the
laptop; on the pod, `tmux has-session -t statusweb` and
`/tmp/status_server.log`. If the page loads but fleet/token sections
are empty, the slow collector hasn't finished its first pass — wait
~2 min. After editing `status_server.py`: commit, push, `git pull` on
the controller, then re-run step 1 (kill+new tmux session). The
server is read-only and safe to restart at any time — it never
touches training, the watcher, or the ledger.

## Time budget guidance

The operator's standing complaint is cycle latency. Read the two
condensed docs + guardrails (fast), use `ops.sh`, launch evals in
PARALLEL (`nohup … &` all of them, then `waitlog` each), and don't
reproduce evidence into RL_LOG — link it. If a run is clearly
improving and flagged auto-continue, the watcher already relaunched
it; your job is the verdict, not the relaunch.
