# Operator questions — obey-then-ask log

Doctrine (operator 08-15): when a decision cycle executes an
operator-authenticated order that conflicts with a written rule, a
prior verdict, or the cycle's own judgment, it EXECUTES FIRST and
appends a question here instead of declining (full rules:
ORCHESTRATOR_PROMPT.md "Operator orders: obey first, ask after").

The operator answers via `/mcp submit_feedback` WITH the dashboard
token (the entry arrives operator-stamped) or any repo/dashboard note.
The next cycle reconciles: it updates RESEARCH_RULES.md /
CURRENT_TRUTHS.md / its mindset to encode the operator's reasoning,
marks the question CLOSED with a pointer to the change, and never
re-litigates it.

Entry format (append; newest last; update status in place):

    ## q_<UTCstamp> — OPEN | ANSWERED | CLOSED
    - cycle: <cycle log name>
    - operator order: <kick/feedback id + one-line summary>
    - conflicted with: <rule / doc / verdict, quoted>
    - why the cycle would have declined: <1-2 lines>
    - what was executed: <run/action + verification>
    - ANSWER (operator): <fb id + summary, filled when answered>
    - rulebook change: <commit/file, or "none — operator confirmed
      one-off exception">

---

## q_20260815T1920Z — OPEN
- cycle: operator kick 08-15 ~18:34 UTC (cw-dynrep-tf-state2-fresh focus)
- operator order: focus note — "cw-dynrep-tf-state2-fresh is launching
  now through the new first-class dynrep-fresh orchestrator trainer on
  train-3. Do not duplicate it," then verify data stage >=10.24M
  windows / reuse <=2x and the 13.62M-param CUDA Transformer stage.
- conflicted with: mechanical reality, not a rule. train-3 was occupied
  by cw-arch-tf-joymodes-scratch1-acq1 (launcher: "GPU pods host
  exactly one run"), so the ordered launch was REFUSED at 18:34; a
  concurrent cycle re-placed it on train-1, where stage-1 collection
  OOM-killed the whole pod at 18:52 (96Gi, 1.18M/10.24M windows).
- why the cycle would have declined: nothing declined — but execution
  materially deviates from the order's letter: the run now lives as
  `cw-dynrep-tf-state2-fresh2` (append-only rename; the launcher
  refuses reusing a name whose -data W&B run exists) on train-10 (clean
  pod + pod_memwatch + freshly recorded CUDA-torch capability), not
  train-3. "Do not duplicate" was honored: the dead original is
  verdicted DEAD, exactly one live instance exists.
- what was executed: DEAD verdict + W&B outcome note for the original;
  retry-once launch VERIFIED RUNNING on train-10 (W&B flaf42k7 data
  stage, gate metrics logging, passed the parent's death point);
  train-1 recreated + bootstrapped + CUDA-torch reinstalled.
- ANSWER (operator): _pending_
- rulebook change: _pending_ — also flagging: the launcher counts only
  trainers when calling a GPU pod free, so it will happily stack a
  96Gi-scale collector next to a heavyweight harness eval (that
  stacking is the likely train-1 killer). Should pod_eval/eval
  processes count toward launcher placement?
