# multitask — Multitask learning

W&B: tag `track:multitask`. Excess-capacity research; run prefix
`cw-mt-`. Design + verdict labels: **rl_docs/MULTITASK.md** (read it
before triaging anything here — this track has its own binding rules
for what counts as forgetting vs acquisition failure).

**Goal:** test whether a fresh command-conditioned generalist
(stand + forward + small yaw/lateral trained SIMULTANEOUSLY, one
coherent reward) beats the sequential specialist-fine-tuning pattern —
both at zero-shot command interpolation and at acquiring a genuinely
new command later (the phase-2 transfer test).

## Now

- Track opened 08-12 (operator brief). Wave 1 A/B/C cohort queued to
  the backlog: `cw-mt-a1` (specialist control, vx fixed 0.05),
  `cw-mt-b1` (vx 0-0.06 + ±0.15 yaw + 40% stand segments),
  `cw-mt-c1` (b1 + lateral via heading ≤0.34 rad). Matched recipe
  (cloned from the cw-dep-fresh1 PASS), matched seed/budget/DR; the
  only variable is the command distribution. 2M discovery each.
- Fresh init IS the hypothesis (warm-start default waived, recorded
  in each spec).

## Next

- Triage wave 1 with MULTITASK.md's eval protocol: per-command
  retained suite + zero-shot interpolation probes + video (paddle vs
  phase-structured gait is THE observation of interest).
- [CODE, when wave 1 lands] fixed retained-command suite runner
  (`eval_cmd_suite`) if `eval_drive.py`/`eval_yaw.py` can't already
  probe exact (vx, vy, wz) triples per checkpoint — per-command
  tracking error/falls/slip/current, det+sto, machine-readable.
- Phase 2 transfer test after wave 1: warm-start A/B/C on the SAME
  new command (larger yaw or backward), fixed 1M/2M/5M budgets,
  measure acquisition speed AND retained-suite erosion.
- Wave 2 levers (one at a time): 256×256 net arch on the best
  generalist; obs history; k_drag_stance; body-height command; seed
  twins of the winner.
