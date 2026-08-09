# Per-experiment logs — `logs/experiments/<run>/`

**Scope (08-09 lightweight process): this directory + summary.md is
for DIG-IN runs only.** A clear pass/fail needs just the ledger
verdict (`launch_run.py update`, which auto-renders
`rl_docs/runs/<run>.md`) and an `ops.sh wandbnote` paragraph. The
watcher's `wandbdump` cache still lands here for every run.

Dig-in experiments get ONE directory that tells the whole story:
`logs/experiments/<run>/` (under `prototype_sts3215/`; gitignored —
this lives on the controller, not in GitHub).

## Layout

```
logs/experiments/cw-walk-anchortol5/
  summary.md        <- dig-in runs only; written by the verdict cycle
  wandb_summary.json  <- cached W&B summary+config (ops.sh wandbdump)
  wandb_history.csv   <- cached scalar history    (ops.sh wandbdump)
  (optional: eval output dirs, frames you judged, scratch analysis)
```

`ops.sh expdir <run>` creates the directory with a summary.md
template. `ops.sh wandbdump <run>` caches the W&B data so later
questions don't need the API again.

## summary.md format — plain language FIRST

```markdown
# <run> — one-line outcome

## What we tried and why (plain English, 2-4 sentences)
Written for a human who knows nothing about the codebase. Big-goal
context first: what problem in the robot's behavior this run
attacks, what we changed, what we hoped would happen.

## What happened
Result in 2-4 plain sentences: did the hoped-for thing occur, what
the video showed, what the verdict was.

## Details (optional, keep short)
Gate numbers, key metrics, links: W&B url, ledger entry, eval dirs,
parent run, checkpoint md5.
```

The same plain-language-first rule applies to W&B run notes: the
`--hypothesis` you pass at launch MUST OPEN with 1–2 sentences a
non-expert can read ("The robot's feet slide while it walks; this
run makes sliding unprofitable by X; we hope to see Y"). Technical
pre-registration (gates, if-true/if-false) comes after.
