# rl_docs — index (read this first, then only what you need)

Small, single-purpose files so no one (human or LLM) has to dig
through a 5,000-line log to answer a question. Each file says what
it is for; keep them SHORT when you edit them.

| File | What it answers | When to read |
|------|-----------------|--------------|
| `AGENT.md` | How the autonomous agent works, what we learned works/fails, future work | Taking over the campaign (human or LLM) — read FIRST |
| `GOAL.md` | What are we doing and why, in plain English; where we are; what's blocked | Every cycle, first |
| `SKILLS.md` | What the robot can DO today: passed skills + their checkpoints (W&B artifact per row) | On any PASS (update it!), or when the operator asks what works |
| `WISHLIST.md` | Operator's backlog of things to learn — pull from it whenever pods would idle | Every cycle, when deciding launches |
| `COMMANDS.md` | How to run everything: `ops.sh` helpers, paths, hard-won gotchas; § "Operator status page" = web dashboard runbook | Every cycle, before running commands; when the operator's status page is down |
| `EXPERIMENT_LOGS.md` | Per-run `logs/experiments/<run>/summary.md` convention + cached W&B data | When finishing or investigating a run |
| `runs/` | One GENERATED summary per run (status, hypothesis, gate, verdict) — rendered from `experiments.json` by `launch_run.py`; never hand-edit | Browsing past runs; `launch_run.py runsmd` refreshes |
| `../RL_PLAN.md` | The current plan, gates, and queue (~120 lines) | Every cycle |
| `../RL_LOG.md` | Condensed campaign history; append ONE line per cycle via `ops.sh logline` only | Every cycle |
| `../rl_move/orchestrator/guardrails.yaml` | Hard limits you must obey | Every cycle |
| `../archive/` | Full history, reviews, audits (long; search, don't read) | Only when the condensed docs point there |

Standing rule: if you had to FIGURE OUT a command (it failed, was
slow, or took several tries) and then got it right, promote it —
add an `ops.sh` subcommand or a snippet to `COMMANDS.md` in the
same cycle, and keep this index accurate. The next agent should
never have to rediscover it.
