# rl_docs — index (read this first, then only what you need)

Small, single-purpose files so no one (human or LLM) has to dig
through a 5,000-line log to answer a question. Each file says what
it is for; keep them SHORT when you edit them.

| File | What it answers | When to read |
|------|-----------------|--------------|
| `GOAL.md` | What are we doing and why, in plain English; where we are; what's blocked | Every cycle, first |
| `WISHLIST.md` | Operator's backlog of things to learn — pull from it whenever pods would idle | Every cycle, when deciding launches |
| `COMMANDS.md` | How to run everything: `ops.sh` helpers, paths, hard-won gotchas | Every cycle, before running commands |
| `EXPERIMENT_LOGS.md` | Per-run `logs/experiments/<run>/summary.md` convention + cached W&B data | When finishing or investigating a run |
| `runs/` | One GENERATED summary per run (status, hypothesis, gate, verdict) — rendered from `experiments.json` by `launch_run.py`; never hand-edit | Browsing past runs; `launch_run.py runsmd` refreshes |
| `../RL_PLAN.md` | The current plan, gates, and queue (~120 lines) | Every cycle |
| `../RL_LOG.md` | Condensed campaign history, 1–3 lines per cycle | Every cycle |
| `../rl_move/orchestrator/guardrails.yaml` | Hard limits you must obey | Every cycle |
| `../archive/` | Full history, reviews, audits (long; search, don't read) | Only when the condensed docs point there |

Standing rule: if you had to FIGURE OUT a command (it failed, was
slow, or took several tries) and then got it right, promote it —
add an `ops.sh` subcommand or a snippet to `COMMANDS.md` in the
same cycle, and keep this index accurate. The next agent should
never have to rediscover it.
