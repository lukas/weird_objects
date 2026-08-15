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

(no questions yet)
