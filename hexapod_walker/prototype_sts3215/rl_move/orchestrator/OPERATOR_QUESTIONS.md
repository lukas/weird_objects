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

## q_20260815T2250Z — OPEN
- cycle: operator-kick 20260815 ~22:2x-22:5xZ (executed
  fb_20260815T222943_d019de: any4 stop/preserve + any5 mjxcert
  from-scratch launch)
- operator order: launch cw-recover-any5-mjxcert-scratch1 "on exact
  main 3589f41018001528e7ce8848f82ee589e86e6d3d".
- conflicted with: nothing — but that full 40-char SHA does not exist
  in the repo or on origin (typo/transcription artifact in the note).
  The real main HEAD commit with the matching short prefix AND the
  exact stated title "Use deterministic MJX recovery certification"
  is 3589f418ca883404ee52fdc59350733a01ae49e0; its content matches the
  directive (recover-cert flags, CERT/* metrics, telemetry-only
  rollout EMAs; test_recover_cert.py 3/3, recover bank 19/19 re-run
  green this cycle). Executed on that commit (pod actually synced at
  snapshot fa7a3047, whose only deltas over 3589f418 are run-doc md
  files, the ledger, and dynrep's pod_tfwalk.sh — recover/cert code
  bit-identical).
- what was executed: full directive — any4 stopped/preserved (verdict
  + OUTCOME note, checkpoint kept on train-1), any5 launched FROM
  SCRATCH on train-1 (W&B 1tmepjxh, no --init-from/transplant,
  --recover-cert-every 1000000 --recover-cert-envs 8, out-name
  ppo_goal_cw_recover_any5_mjxcert_scratch1), verified live via /proc
  + "[recover-cert] armed" log line + advancing steps.
- ANSWER (operator): —
- rulebook change: —

## q_20260815T2255Z — OPEN
- cycle: operator-kick 20260815 ~22:4x-22:5xZ (corrected GPU-only
  tfwalk relaunch, order 20260815T224355Z /
  fb_20260815T222316_26b670)
- operator order: relaunch the Transformer walking A/B/C transfer
  cohort GPU-only under append-only names dynrep-tfwalk-gpu1-{A,B,C}-
  s5; treat 9e4eimd8 + both old C attempts as ABORTED/NON-EVIDENCE;
  no gate rerun; no CPU fallback or encoder/data substitution.
- conflicted with: (1) nothing mechanical — but the order only
  explicitly aborted the old C attempts, while the root cause (the
  trainer hard-coding device=cpu) equally compromises the old A
  (11zsrpl9) and B (f086dlfd) attempts; I marked all three ABORTED/
  NON-EVIDENCE (A/B as "class stopped by the C finding") and KILLED
  the old B trainer found still alive on train-7 — without that kill
  the ordered B relaunch was mechanically impossible (pod_tfwalk.sh
  refuses a second trainer per pod). Confirm A/B non-evidence
  reading. (2) SB3 emits its generic "PPO on GPU with MlpPolicy will
  be slower" advisory on arm A — executed --device cuda anyway per
  the explicit GPU-only order (matched-triple comparability + the
  B/C encoder arms are where CUDA pays). (3) launcher bypass for a
  script-owned cohort — precedented (08-15 22:2x q entry), ledger
  entries created mechanically with proc_match/wandb_match so watcher
  checkups work.
- what was executed: full order — old cohort aborted in ledger;
  train-7 synced (was f4978e72 → dfe6e78e); CUDA torch 2.11.0+cu128
  installed+recorded on train-7/8 via pod_torch_capability.py (both
  were 2.13.0+cpu — arm-A/B pods would have failed the CUDA
  assertion); G1/G1.1 PASS record verified readable on all 3 pods
  (NOT rerun); launched A train-8 (W&B h9yy9fll), B train-7
  (dg5oj5hs), C train-11 (dx4yw04i); every log prints "[device] CUDA
  required and active: NVIDIA H200" before W&B init; C anchor
  tensors on CUDA (anchor_batch_to_torch device=cuda); steps
  verified advancing on all three (86k/12k/49k at check).
- ANSWER (operator): —
- rulebook change: —

## q_20260816T2140Z — OPEN
- cycle: operator-kick 08-16 ~21:xx UTC (dynrep joint-PPO directive)
- operator order: fb_20260816T203212_af7c64 — corrected dynrep joint-
  PPO experiment; "reuse the recovered corpus whose aggregate SHA-256
  is 6762fe81a069...f1c70cf0. Do NOT rerun the offline collector."
- conflicted with: nothing procedural — but the quoted aggregate
  SHA-256 is NOT reproducible: 8 aggregation variants over the 81
  v5_mjx_fresh shards on train-11 (per-shard sha256 sorted/unsorted/
  with names/with meta.json, cat-of-bytes in several orders) all
  yield different digests; the method behind 6762fe81 was never
  recorded in the repo (it entered via the 20260815T2212 order's gate
  text).
- why the cycle would have declined: could not literally verify the
  quoted hash.
- what was executed: the order's intent (SAME corpus, no recollection)
  is satisfied by provenance instead: identical directory/shard set
  used by cw-dynrep-tf-state2-recovered1 pretraining and the
  metrics1-C anchors, shard mtimes untouched since 08-15 19:03-19:46
  UTC (pre-dating both). Recorded a CANONICAL reproducible aggregate
  in the new ledger entries for future bit-exact checks:
  `cat shard_*.npz | sha256sum` (shard name order) =
  a0bb722ecdd38ebb013ff8dcedf28edd6ffb38510e17e0f7fb570540c03a717a.
- ANSWER (operator): _pending_
- rulebook change: _pending_

## q_20260817T0140Z — OPEN
- cycle: operator KICK 2026-08-17 (fb_20260817T005114_775298 focus
  note — joystick-walking gated canary)
- operator order: fb_20260817T005114_775298 (GPT-5 Codex, explicitly
  approved by Lukas): implement critic-audit + safer PPO update path
  + bounded terminal cost + calibrated height gating + curriculum +
  best-ckpt retention, then launch a 2-3M canary of the joystick
  command-switch line.
- conflicted with: arch/STATUS.md 08-16 closure ("the instantaneous
  command-switch reward/curriculum recipe itself is the blocker …
  no further chunks or variants of this exact recipe are queued from
  arch; the only remaining lever is a redesign of the
  command-tracking task itself, which is a multitask-track call")
  and the multitask PAUSE (CURRENT_TRUTHS 08-13).
- why the cycle would have declined: the canary is a follow-up on the
  closed arch joystick line, and task-redesign was routed to a paused
  track.
- what was executed: the order in full — the canary is NOT the closed
  exact recipe (it changes the update path, terminal-cost shape,
  height gating, and adds the in-run curriculum the closure said was
  missing); mechanisms landed cfg/flag-gated default-off with tests
  (test_value_learning.py, test_walk_cmd_metrics.py, JOYCANARY bank)
  and cw-arch-joystick-canary1 launched on arch with pre-registered
  promotion/refusal gates (no 40M clone without canary PASS).
- ANSWER (operator): _pending_
- rulebook change: _pending_

## q_20260817T1930Z — OPEN
- cycle: operator kick 2026-08-17 ~19:09 UTC (recover any15 launch)
- operator order: MCP focus note 20260817T190903Z (+ amendment
  fb_20260817T185822_6b9bfc) — launch cw-recover-any15-retentionrollback-cont1
  at exact SHA 7d39a25 with per-promotion checkpoints + timed retention
  rollback; never retry the any14 stubs.
- conflicted with: SIM SPRINT banner (operator 08-17): "No new
  research-track launches ... the fleet's single deliverable is reliable
  rising + walking in the MuJoCo sim"; also hw-track recovery arm launched
  while the sprint scopes work to rise/walk reliability.
- why the cycle would have declined: recover-ladder training is not one of
  the named rise/walk session reliability gaps; under the sprint I would
  have parked this arm.
- what was executed: cw-recover-any15-retentionrollback-cont1 launched on
  hexapod-mjx-train-0 (PID 1832018, W&B xqcqvb3u, SHA 7d39a25 synced, 45
  recovery tests green); first promotion B1@1.05M verified (checkpoint ZIP +
  curriculum JSON on pod and on W&B, RECOVER_GUARD/promotion_checkpoint_saved=1);
  stale auto-retry cw-recover-any14-retentiongate-cont1-r2 closed KILLED per
  the never-retry order.
- ANSWER (operator): —
- rulebook change: —

## q_20260817T2200Z — OPEN
- cycle: operator-kick cycle 2026-08-17 ~21:5x-22:xx UTC (fb_20260817T210422_9df9c7 execution)
- operator order: fb_20260817T210422_9df9c7 — launch cw-dynrep-livewalkrise1 (live command-rich CUDA online-predictor + boundary-gated critic snapshot) and cw-dynrep-criticD-40m1 (40M frozen-critic-D walk) in parallel.
- conflicted with: (1) SIM SPRINT ruling (CURRENT_TRUTHS 08-17 ~18:05 UTC): "No new research-track launches (dynrep, ...) unless the arm directly serves that goal"; dynrep STATUS header: "NO NEW LAUNCHES on this track unless an arm directly serves reliable rise+walk"; also the 08-17 ~15:xx entry recorded "E CLOSED; NO launch/extension per operator order" for the online-predictor line. (2) The E verdict itself (online adaptation REDUCED the transfer benefit at 1M, pre-registered gate FAIL).
- why the cycle would have declined: the E-lineage online-predictor mechanism has one clean pre-registered FAIL on record and the sprint ruling bans new dynrep launches; a non-operator cycle would have parked both arms as [operator]-gated.
- what was executed: both arms built and launched per the order (the order is newer, operator-stamped, and explicitly frames both as serving the SIM SPRINT rise+walk deliverable; the livewalkrise design also addresses E's measured failure mode via boundary-gated versioned snapshots, so it is a NEW mechanism, not an E re-run). Code committed to main; preflight banks green; canary-first staging for arm A.
- ANSWER (operator): _pending_
- rulebook change: _pending_
