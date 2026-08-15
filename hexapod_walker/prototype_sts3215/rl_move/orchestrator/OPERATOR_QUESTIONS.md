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

## q_20260815T2010Z — OPEN
- cycle: opkick-recover-any2 (08-15 ~19:5x–20:1x UTC operator KICK)
- operator order: fb_20260815T194955_9441a0 + kick focus note — sync
  aa1023c, stop cw-recover-any1, launch cw-recover-any2 from
  cw-stand-footlow2-hard1 with the seven listed cfg overrides.
- conflicted with: this cycle's own prompt concurrency assignment
  ("Another decision cycle ... already handles: ... cw-recover-any1.
  Do NOT evaluate, verdict, or launch on behalf of those runs").
- why the cycle would have declined: acting on any1's line risked
  duplicating the concurrent fb_193318 cycle's stop-and-fix work
  (its uncommitted ledger edits + conflicted sim files were live in
  the shared clone when I started).
- what was executed: the order, fully — verified the concurrent
  cycle had already KILLED any1 (verdict recorded, process dead,
  its duplicate implementation dropped for aa1023c; conflicts
  resolved to main by the time I acted), then launched the run
  myself (respec --from any1, parent cw-stand-footlow2-hard1, all
  7 cfg keys, train-1, aa1023c code bit-exact). First attempt
  (W&B lf5afhd6, name cw-recover-any2) was a FALSE START — train-1's
  fresh bootstrap lacked sb3-contrib so the bg eval/video/canary
  child died at first import; killed at ~5M, pod env fixed,
  bootstrap_train_pod.sh patched, relaunched as cw-recover-any2b
  (W&B u9sp8dki; W&B names append-only). All directive
  verifications pass on any2b incl. split onefoot/park eval
  (SCORE/recover_onefoot_success=1, park=1 at 1.0M). No duplicate
  launch occurred (launcher dedupe REFUSED correctly). Suggest
  future operator kicks that retarget a run named in a concurrency
  list say so explicitly.

## q_20260815T2050Z — CLOSED (08-15 ~22:0x UTC, by fb_20260815T214555_008f42)
- cycle: opkick-recover-any3-scratch1 (08-15 ~20:3x-20:5x UTC)
- operator order: fb_20260815T201417_5f7f0e (stop cw-recover-any2 /
  lf5afhd6, launch cw-recover-any3-scratch1 FROM SCRATCH) then
  fb_20260815T201712_39279d (SUPERSEDES the scratch launch: wait for
  the bucket-0 curriculum implementation + a new exact main SHA +
  final launch directive; if any3 launched concurrently, stop/preserve
  it as superseded).
- conflicted with: (a) "warm-start by default" + the running-runs
  hands-off etiquette — killing cw-recover-any2b, a CONCURRENT cycle's
  env-fixed relaunch of the stopped arm, is not literally named by
  either directive; a peer cycle (20:28 dynrep logline) read the same
  two directives as "any2b RUNNING is fine". (b) The stop order's own
  evidence ("zero success, no split SCORE metrics") turned out to
  describe an INSTRUMENTATION failure: any2 was eval-blind (missing
  sb3-contrib killed its eval sidecar), and the fixed twin any2b was
  succeeding.
- why the cycle would have declined: any2b at kill time (2.75M) showed
  the line's FIRST genuine recovery successes — split det eval
  onefoot success=1 (solved in 1.58 s) AND park success=1 (2.36 s),
  SCORE/recover_success=1, tipped_recovery_success=1, BC recover
  anchor filling, curriculum correctly bucket-1-only. The "warm-start
  flatlined at zero" premise behind pausing the line came from the
  blind evaluator. On the merits I would have let any2b run while the
  bucket-0 curriculum was being built.
- what was executed: obeyed the strict reading — the warm-started arm
  is STOPPED and the recovery line WAITS. cw-recover-any2 verdict
  corrected (eval-blind FALSE START, preserved as the warm-start
  diagnostic); cw-recover-any2b KILLED at 2.75M via ops.sh killrun
  (verified dead, /dev/shm cleaned), verdict + W&B note record its
  positive evidence, checkpoint ppo_goal_cw_recover_any2b.zip
  preserved on train-1 + W&B u9sp8dki (RESUMABLE on one word);
  cw-recover-any3-scratch1 NEVER LAUNCHED (first attempt REFUSED
  pod-busy, then superseded; ledger stub marked SUPERSEDED "do not
  drain/retry"; snapshot tag exp/cw-recover-any3-scratch1 = 6f909719
  exists, nothing trained on it). RECOVER preflight bank 13/13 PASS
  on current main was run before the supersede arrived and stands.
- DECISION NEEDED: (1) resume any2b (warm + fixed evaluator, already
  succeeding on bucket-1) alongside or instead of the bucket-0
  from-scratch plan? (2) confirm the bucket-0/scratch design should
  weigh any2b's det onefoot/park successes — the zero-success premise
  was instrumentation, not learning.
- ANSWER (operator): answered implicitly by directive
  fb_20260815T214555_008f42 (08-15 21:45 UTC): (1) the bucket-0
  from-scratch plan proceeds — cw-recover-any4-b0scratch1 launched
  FROM SCRATCH at exact main c60c7ac; any2b stays killed/preserved
  and is designated "comparison evidence only", NOT resumed.
  (2) not directly addressed; any2b's positive det onefoot/park
  evidence remains recorded in its ledger/W&B note and the B0-B7
  forced eval metrics now give per-bucket denominators that will
  settle the warm-vs-scratch question empirically. CLOSED — executed
  same cycle; see STATUS.md WAITING-ON clearance + hw/STATUS.md
  "Now" entry. 
- rulebook change: 

## q_20260815T2240Z — OPEN
- cycle: operator-kick 20260815T221231Z (Codex-relayed order, MCP
  operator lane)
- operator order: run eval_model G1/G1.1 on cw-dynrep-tf-state2-
  recovered1 + exact v5_mjx_fresh corpus; if PASS, launch matched
  walking/heading PPO A/B/C from THIS checkpoint, "GPU-only, W&B
  tracked, new append-only names".
- conflicted with: nothing hard. Two interpretation notes, executed
  per best precedent rather than declined: (1) "GPU-only" — the A/B/C
  framework (`train_ppo_transfer`) is SB3 `device="cpu"` by design
  and every prior cohort (futurewalk, risewalk) ran it on the GPU
  pods' CPU cores; I read "GPU-only" as "on the GPU fleet, never the
  controller" and launched on train-7/8/11. If the operator meant
  CUDA-resident PPO, that is new code (SB3 device=cuda + GPU encoder
  forward) — say the word and I'll build it under a new name. (2) The
  kick preamble said "skip eval steps for runs already logged", but
  the order said "immediately run" the gate — I re-ran it (cheap,
  ~3 min on train-11's CPUs): fresh PASS, numbers identical to the
  20:4x record.
- what was executed: gate re-run PASS (eval_g1_test_order_
  20260815T2219.json); pod_tfwalk.sh cohort launched, A `11zsrpl9`
  train-8 / B `f086dlfd` train-7 / C train-11 (first attempt
  `9e4eimd8` died silently ~63s in, no OOM/no traceback, retried
  once via setsid — see dynrep/STATUS.md); all six script-owned live
  runs registered in the ledger (dynrep-tfwalk-{A,B,C}-s5,
  risewalk-single2-s{5,6,7}).
- ANSWER (operator): —
- rulebook change: —
