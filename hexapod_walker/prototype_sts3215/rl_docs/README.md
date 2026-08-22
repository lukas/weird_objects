# rl_docs — index (read this first, then only what you need)

Small, single-purpose files so no one (human or LLM) has to dig
through a 5,000-line log to answer a question. Each file says what
it is for; keep them SHORT when you edit them.

The campaign runs exactly two tracks (operator reset 2026-08-21):
`joystick` and `amp`. Superseded docs live in
`../archive/two_track_reset_2026-08-21/`.

| File | What it answers | When to read |
|------|-----------------|--------------|
| `../RL_GOALS.md` | The two goals in plain English | Every cycle, first |
| `../CURRENT_TRUTHS.md` | Accepted facts and rulings; wins on conflict | Every cycle |
| `../RL_PLAN.md` | The two-track operating plan and queue | Every cycle |
| `../STATUS.md` | Operator-facing digest: how it's going, what's waiting | Catching up; after any story-changing verdict (update it!) |
| `tracks/joystick/STATUS.md` | Goal/Now/Next for the RL-from-teacher joystick track | Working that track |
| `tracks/amp/STATUS.md` | Goal/Milestones/Now/Next for the from-scratch AMP track | Working that track |
| `AMP_LOCOMOTION.md` | The AMP program charter (binding, incl. repo adaptations — no Isaac Lab) | Before any amp-track design decision |
| `DOWNLOAD_ANSWER.md` | The current deployable answer + gate evidence | When a verdict might change what we'd put on the robot |
| `SKILLS.md` | What the robot can DO today: passed skills + checkpoints | On any PASS (update it!) |
| `COMMANDS.md` | How to run everything: `ops.sh` helpers, paths, gotchas; § "Operator status page" = dashboard runbook | Every cycle, before running commands |
| `HARDWARE.md` | Real-robot evidence, sim2real findings | When a decision hinges on real-world data |
| `SIM.md` | What the physics sim models, actuator numbers, DR coverage | Before touching sim params/DR or judging sim-vs-real gaps |
| `REWARD.md` | Every reward term: cfg key, default, what it pays/charges, income-gate design rules | Before adding/changing any reward term |
| `EVALS.md` | Every eval metric: `SCORE/*` names, `eval/*` details, harnesses, caveats | Reading a W&B page or wiring a new metric |
| `GAIT.md` | Anti-paddle gait quality: lift-and-place metrics, known exploits | Judging walking video; designing gait rewards |
| `FAST_PROFILE.md` | Raised servo profile facts + command-tracking prep | Fast-gait arms on the joystick track |
| `COMPLIANCE.md` | Structural-compliance measurement + sim hook | Sim-fidelity questions |
| `EXPERIMENT_LOGS.md` | Per-run `logs/experiments/<run>/` convention (dig-ins) | When digging into a run |
| `WANDB.md` | How W&B is wired: project/creds, ops.sh readers, gotchas | First time touching W&B or on auth failure |
| `runs/` | One GENERATED summary per run — rendered from `experiments.json`; never hand-edit | Browsing past runs |
| `../RESEARCH_RULES.md` + `../RUN_INTERPRETATION_RULES.md` | How to design, launch, continue, and judge runs (incl. the 08-21 reward/eval ruling) | Before launch/triage |
| `../rl_move/orchestrator/guardrails.yaml` | Hard limits you must obey | Every cycle |
| `../archive/` | Full history, reviews, retired docs (search, don't read) | Only when the active docs point there |

Standing rule: if you had to FIGURE OUT a command (it failed, was
slow, or took several tries) and then got it right, promote it —
add an `ops.sh` subcommand or a snippet to `COMMANDS.md` in the
same cycle, and keep this index accurate. The next agent should
never have to rediscover it.
