# dynrep — Dynamics-representation pretraining

**08-17 ~06:xx UTC (operator-kick cycle, directive
fb_20260817T052333_e5ae09 executed — Lukas: "ok try it"): the
decoupled predictive-CRITIC transfer is BUILT and the E canary is the
live test.** The genuinely-new mechanism after three actor-side
misses: the actor is the proven scratch-A raw-obs policy (bit-identical
init to `PPO("MlpPolicy")` at the same seed — test-locked), it never
sees a transformer latent and shares no params/optimizer state with
one; only the CRITIC adds a stop-gradient predictive-latent residual
behind a learned gate initialized exactly to zero (start = bit-exact
scratch A, test-locked). Two transformer instances: the ONLINE
predictor (condition E) trains continuously with its own Adam on fresh
rollout windows + 25% v5 rehearsal; an inference-only SNAPSHOT feeds
the critic and may change ONLY via a drift-guarded EMA strictly
between rollout+PPO iterations (identity bit-checked + version
asserted across rollout/GAE/all PPO epochs; out-of-band mutation
RAISES; oversized candidates logged + skipped). Condition D keeps the
snapshot frozen forever. Code: `dynamics/predictive_critic.py`
(+trainer conditions D/E, runner `pod_tfwalk_critic.sh`); tests:
`test_dynrep_predictive_critic.py` 8/8 (actor/raw-critic bit-match to
A, zero-gate no-op, obs→z parity with the audited B wiring, PPO grads
can't touch either transformer, predictor updates leave actor
untouched with the explicit zero-action-KL proof metric, snapshot
updates between-iterations-only, drift-guard skip, small-checkpoint
round-trip) + all prior dynrep banks green (32 total). tfwalk-joint1's
checkpoint-exclusion lesson applied (runtime never pickled; E's online
predictor saved separately as `dyn_online_<name>.pt`). Staging per the
directive: ≤100k one-seed E canary first (hard gates: CUDA, actor
independence, snapshot stability, W&B advancing, ~60MB saves), then
D/E seeds 5/6/7 to the pre-registered 1M decision gate vs the existing
config-equivalent scratch-A controls (joint1 A-s6/s7 + metrics1 A-s5
at its 1M eval point; prior B/C are NOT controls — they changed actor
inputs). PASS for E = predictor heldout within 15%, actor KL ~scratch
A, critic EV/sample-efficiency + heldout walking better than BOTH A
and D across seeds without gait regression; otherwise record that
predictive representation does not help PPO here and STOP the line.

**08-17 ~01:xx UTC (operator-kick diagnosis cycle, directive
fb_20260817T005323_22be43 executed — NO relaunch): tfwalk-joint1
RESOLVED. Corrected C FAILS the pre-registered 1M gate on ALL THREE
conditions; per the gate, STOP adding complexity. Now/Next: idle,
`[operator]` decision on any next dynrep experiment.**
- Science (matched data at 1M; A/B seeds 6/7 finished clean, C-s7 has
  the complete 1M eval series in W&B): (1) heldout prediction ended
  2.680 vs pretrained 2.286 (+17.2%, outside the 15% band, regressing
  late from ~2.46); (2) combined-update action-KL 0.070 / approx_kl
  0.054 vs 0.02 target & 0.04 guard with 98 rejected updates on C-s7;
  C-s5/s6 had the aux PERMANENTLY STOPPED by 500k (degenerate no-aux
  arms); (3) C walk 288.9 < B 308.8/318.5 < scratch A 334.9/360.8,
  C best heldout 473.7@550k then regressed (B-s7 525.9). Scratch A
  also keeps the cleanest gait (slip 0.54-0.64, roll 2.5-3.2 deg,
  slew_sat ~0.5 vs B/C 0.79-0.93). Three cohorts in a row (gpu1,
  metrics1, joint1) now agree: the pretrained dynamics transformer
  does not transfer to walking under any tried mechanism.
- Operations (root-cause proven, distinct from the science): all
  three C arms were `kill -9`ed by the LEFTOVER risewalk-era
  `pod_memwatch.sh` on train-11/4/5 (kill lines + pids + timestamps
  match: 22:50:06Z/23:04:44Z/23:50:42Z) when checkpoint saves spiked
  container memory over its 85GiB threshold — JointAuxPPO pickled the
  rehearsal corpus + online windows into SB3's `data` blob (12.5GB per
  zip; A=4MB, B=56MB). C-s5/s6 died mid-ck500000, C-s7 mid-ck1000000
  AFTER logging its complete 1M eval; those three zips are 0-byte
  husks (the C-s7 "1M checkpoint" never existed). The A/B pods have no
  memwatch — asymmetric guard deployment, worth an operator look.
  Wrapper `set -e` swallowed the deaths (no phase_fail/FAIL flag) =
  why the ledger stayed RUNNING.
- Fixes landed this cycle (tests 8/8 new bank + 20/20 all dynrep
  banks; real C-s7 best zip round-trip verified on train-9):
  `JointAuxPPO._excluded_save_params` (aux runtime never pickled),
  `ScaledLRPPO.set_parameters` optimizer param-group rebuild (plain
  `.load()` of two-group checkpoints used to raise), pod_tfwalk_joint
  phase_fail manifest + POD_TFWALK_JOINT_FAIL on nonzero trainer exit.
- Artifacts preserved: controller `artifacts/tfwalk-joint1/` (all A/B
  zips, every valid C checkpoint's policy+optimizer tensors, all 7
  eval CSVs/logs/manifests, memwatch kill evidence, full C-s7
  best-550k zip md5 5c59d375... + second copy on train-9); W&B
  summary+history cached for all 7 runs; ledger statuses corrected
  (4 FINISHED / 3 FAIL with crash_cause), OUTCOME notes on all 7 run
  pages. Orphans cleared on train-4 (stale risewalk launcher bash +
  duplicate memwatch); one memwatch guard left running per C pod.

**08-16 ~21:xx UTC (operator-kick cycle, directive
fb_20260816T203212_af7c64 executed): condition C REBUILT as a joint
PPO+auxiliary update and the corrected 1M A/B/C cohort is the live
test.** The directive's root-cause read of metrics1 is accepted: the
old AnchorCb mutated the SHARED transformer out-of-band (multiple Adam
steps between rollout collection and the PPO update), invalidating the
old-policy/value assumptions — C's late regression came with approx_kl
~0.085-0.089 vs A/B ~0.02 while the anchor loss stayed flat. Built and
landed this cycle (`joint_aux.py`, `online_windows.py`, trainer rewire;
`test_dynrep_joint_aux.py` 6/6 + all prior dynrep banks green + CPU
end-to-end C smoke): transformer capacity unchanged; 50k encoder-frozen
head warmup; future-state loss inside every PPO minibatch (same
backward/optimizer, encoder 0.1x LR group); aux batches = online
rollout windows (new capture wrapper, collector frame contract) + 25%
rehearsal from the untouched v5_mjx_fresh corpus; total action-KL
guard (target 0.02 / rollback guard 0.04 / stop after 3 consecutive
rejections) with full rollback of params+optimizer; latent drift +
accepted/rejected logging; heldout prediction quality re-measured on
the corpus val split at every heldout eval; periodic checkpoints +
best-by-heldout-walk retention. Out-of-band mutation now RAISES
(bit-exact guard; regression test drives a rogue AnchorCb-style
callback). Cohort `tfwalk-joint1` (runner `pod_tfwalk_joint.sh`):
1M-step pre-registered decision checkpoint, seeds 5/6/7 for C, seeds
6/7 for A/B; metrics1 seed-5 A/B reused at their 1M eval points
(config-equivalent: same trainer defaults, same task/eval cadence/seed;
A/B training paths untouched by the new code — verified by the A smoke
+ code inspection). Gate: corrected C must preserve phase-1 heldout
prediction quality, keep total action-KL near target without late
regression, and beat B on walk return + gait quality at 1M across
seeds; if it cannot beat B, STOP adding complexity and record that.
Corpus note: the directive's quoted aggregate SHA (6762fe81...) is not
reproducible by any standard aggregation; identity established by
provenance and a canonical hash recorded (OPERATOR_QUESTIONS
q_20260816T2140Z).

**Late-cycle addendum (~21:5x UTC): guard-attribution fix + C restart.**
First C-s5 attempt exposed a real design artifact: PPO's OWN approx_kl
runs ~0.03 early in walk training, so every combined update breached
the 0.04 guard and the un-attributed consecutive-reject counter
permanently stopped the auxiliary ~50k steps in (C degenerating to a
no-aux arm). Fixed (b91c16ba, tests 7/7): a rejection advances the
permanent-stop counter only when the no-aux retry lands UNDER the
guard (breach attributable to the auxiliary); rollback + rejected
logging unchanged, `aux/action_kl_retry` now logged. All three C arms
killed and relaunched on the fixed code (attempt-2 W&B: rmucm5m0 /
mjhi4knh / 9k7n9svg). A-s6 already FINISHED clean at 1M (walk 334.9,
best-by-heldout checkpointing verified in production).

**08-16 ~06:3x UTC (triage cycle): the metrics1 matched triple landed
and TRIAGED — first real behavioral verdict for the dynrep hypothesis,
and it's a clean MISS.** All three `dynrep-tfwalk-metrics1-{A,B,C}-s5`
finished their 2M-step budget clean (POD_TFWALK_DONE, ~65/90/95 min).
Matched-triple read at the full budget: **scratch PPO (A, no
pretrained representation at all) wins on every axis** — highest
final walk return (371 vs B 350 / C 262), cleanest gait (slip_m 0.52
vs 0.91/0.85, peak_roll 2.7° vs 7.5°/6.8°, slew_sat 0.46 vs 0.94/0.75
— B and C are frequently saturating joint-speed commands), and far
better heldout hold-task reward (146 vs 0.7/1.3, i.e. B/C forgot how
to stand/hold almost completely). The pre-registered 1M→2M slope
question for C (continual anchor fine-tuning) answers in the WRONG
direction: at 1M C briefly LED the triple (386 vs A's 281), but by 2M
it REGRESSED to dead last (262) while A and B kept climbing — the
anchor's own prediction loss stayed flat (~1.98, matching pretraining,
so the encoder didn't break), so the regression looks like the walk
and dynamics-prediction objectives fighting each other in the shared
policy, not a broken world-model. Frozen (B) fares no better than
scratch either. **Verdict: at this 2M budget, neither frozen nor
continually-anchored reuse of the pretrained dynamics representation
gives PPO a sample-efficiency or gait-quality win over training from
scratch — continual anchoring looks actively harmful past 1M.** One
miss, not two — per the two-miss rule this doesn't yet close the
dynrep hypothesis, but it is the first real evidence against it (G1/
G1.1 predictor-quality gates were never in question; this is the
downstream "does it help RL" question, and so far: no). Full metrics:
ledger entries `dynrep-tfwalk-metrics1-{A,B,C}-s5`. Next: either a
lighter-touch use of the representation (auxiliary loss instead of
frozen/anchored features) or an operator call on whether to keep
pushing this lever — not simply re-running the same recipe longer
(RESEARCH_RULES "two misses" discipline once a second matched arm
also misses).

**08-16 ~04:3x UTC (operator-kick cycle, order 20260816T042655Z
executed): gpu1 cohort corrected to FINISHED + fresh 2M metrics
cohort LIVE.** The three `dynrep-tfwalk-gpu1-{A,B,C}-s5` trainers all
exited cleanly at their 1M budget (~23:23-23:38 UTC 08-15,
POD_TFWALK_DONE + checkpoints on-pod) but the ledger rows were stale
RUNNING — corrected to FINISHED, no triage verdict (the learning
question is superseded by the rerun below). Per the order: synced
b823cc76 (transfer-v2 metrics contract — `rollout/*` per-rollout
callback, `SCORE/*` at evals, VecMonitor) to train-7/8/11, ran the
three focused tests on CUDA train-8 (`test_dynrep_transfer_metrics`
2/2, `test_dynrep_ppo_anchor` 3/3, `test_pod_trainers_scan` 3/3),
then STAGED-launched the append-only rerun `dynrep-tfwalk-metrics1-
{A,B,C}-s5`: 2M PPO steps from scratch (2x the prior cutoff,
specifically to see whether C is still learning past 1M), seed 5,
eval every 10k on rise/hold/walk + heldout, `--device cuda`, fresh
W&B IDs, NO init from any gpu1 checkpoint. A (train-8, `jf0tfsqh`)
launched first; the required schema (all 7 rollout/* keys + counts,
SCORE/walk_total_reward, eval/walk/return, train/* diagnostics) was
mechanically verified present on W&B before B/C went out. B (train-7,
`psiz3y6x`, frozen encoder md5 9df48f68 = train-11's) and C
(train-11, `axw76nij`, same encoder + anchor batches from the
original recovered `v5_mjx_fresh` 8.3G corpus, no recollection)
verified: "[device] CUDA required and active: NVIDIA H200" pre-W&B,
steps advancing on all three, same schema, C `anchor/loss` present
with untouched pretrained anchor loss 1.908 (~pretraining val ≈2.0,
tripwire clean). Triage when they land: matched-triple gate incl.
the 1M→2M slope question, in the ledger entries.

**08-15 ~23:0x UTC (checkup cycle): the 22:37 watcher alarms on all
six dynrep runs are RESOLVED — no live run was actually unhealthy.**
The three tfwalk alarms were the operator/Codex CPU-compliance kills
(see 22:5x entry below; gpu1 relaunch cohort is the live test) plus a
checkup crash on the missing `/tmp/train_<run>.log`; the three
`risewalk-single2-s{5,6,7}` DEADs were FALSE (trainer `--name` is
per-phase, e.g. `rw_rise_C_s5`; all three verified alive, ~108 min
CPU each). Fixed the machinery so script-owned cohorts stop
false-alarming: checkup now honors optional ledger fields
`proc_match`/`wandb_match` (regex; absent = old behavior, bit-exact),
`_pod_trainer_pid` learned `train_ppo_transfer`, and the GPU fps
floor carve-out extends to `train_ppo_transfer` (8-env SB3, healthy
~50-560 fps vs the 19-20k MJX floor). Fields set on the risewalk
entries; end-to-end smoke: `checkup --run risewalk-single2-s5` now
HEALTHY rc=0 (was DEAD, then SUSPECT). Launcher scan tests 14/14
green. My interim A/B retries (pre-CPU-finding) were superseded and
killed by the gpu1 relaunch — recorded as non-events.

**08-15 ~22:5x UTC (operator-kick cycle, corrected order
20260815T224355Z executed): the 22:2x tfwalk cohort was a GPU
COMPLIANCE FAILURE — all three arms ran `train_ppo_transfer` with
device hard-coded to CPU (fb_20260815T222316_26b670; Codex stopped
C). All three old attempts (`11zsrpl9`/`f086dlfd`/`9e4eimd8`) are
ABORTED/NON-EVIDENCE in the ledger; the stale old-B trainer found
still alive on train-7 was killed this cycle. RELAUNCHED clean on
the CUDA-required trainer (c4f5b211 + append-only names 66d024a):
`dynrep-tfwalk-gpu1-A-s5` train-8 (W&B `h9yy9fll`),
`-B-s5` train-7 (`dg5oj5hs`), `-C-s5` train-11 (`dx4yw04i`) — every
log prints "[device] CUDA required and active: NVIDIA H200" BEFORE
W&B init, C's anchor tensors are built on CUDA, steps verified
advancing on all three. The saved G1/G1.1 PASS gate record was
verified readable on each pod (NO gate rerun, per order). CUDA
torch (2.11.0+cu128) was installed+recorded on train-7/8 via
pod_torch_capability.py (both were still 2.13.0+cpu).**

**08-15 ~22:2x UTC (SUPERSEDED by the 22:5x entry above — cohort
was CPU-compromised) (operator-kick cycle, order 20260815T221231Z
executed): G1/G1.1 RE-VERIFIED and the Transformer-encoder
walking/heading A/B/C transfer cohort is LAUNCHED.** Per the order:
re-ran `rl_move.dynamics.eval_model --split test` on the exact
`cw-dynrep-tf-state2-recovered1.pt` + recovered `v5_mjx_fresh` corpus
on train-11 — **G1 PASS + G1.1 PASS at every horizon** (k=1 model MSE
0.0788 vs ridge 0.1389 / persistence 0.2314; identical to the 20:4x
record; report pulled to `logs/ckpt_eval/cw_dynrep_tf_state2_
recovered1/eval_g1_test_order_20260815T2219.json`, gate record written
to each launch pod as `logs/cw-dynrep-tf-state2-recovered1_gate.txt`).
Gate passed → launched the matched walk-task (commanded velocity/
heading) PPO triple via the new `pod_tfwalk.sh` (snapshot
`exp/cw-dynrep-tfwalk-abc`, one condition per pod, design matched to
the GRU futurewalk benchmark: 1M steps, seed 5, dr 0.3, eval
rise/hold/walk + held-out suites): **A scratch** train-8 (W&B
`11zsrpl9`), **B frozen TF encoder** train-7 (`f086dlfd`, encoder
md5-verified identical to train-11's), **C anchored TF encoder +
v5_mjx_fresh anchor batches** train-11 (first attempt `9e4eimd8` DIED
silently ~63s in — whole process group gone, NO traceback, NO cgroup
OOM (`memory.peak` 42.5GiB of 96Gi, `oom_kill 0`), no memwatch kill;
retried once via `setsid` full detach). All arms ledger-registered
(`dynrep-tfwalk-{A,B,C}-s5`) with hypothesis+gate; the live GRU
`risewalk-single2` cohort (train-4/5/6, condition-C rise phase,
verified via `check_cohort` + W&B advancing) registered post-hoc as
`risewalk-single2-s{5,6,7}` per the order's "register all live runs".
NO substitution of `dyn_scale_M_h16_large` or any older encoder
anywhere in the tfwalk cohort; no recollection performed.

**08-15 ~20:4x UTC (triage, earlier cycle): `cw-dynrep-tf-state2-recovered1`
FINISHED and is a clean PASS — the first TRANSFORMER-architecture
dynrep encoder to clear every binding gate (prior G1/G1.1 passes were
all GRU, e.g. `dyn_scale_S/M_h16`).** The recovered flaf42k7 corpus
(10.24M fresh GPU-collected windows) trained the unchanged 13.62M-param
causal Transformer to its full 40k-step budget with NO divergence
(train/val/test total 2.08/2.24/2.26, val/train ratio 1.03,
`generalization/overfit_alarm` never fired) — the exact opposite of
predecessor `telnzd5r`, which the operator killed at 21k/40k on a
5.53→8.22 held-out blowup. This run's own training loop only logs a
persistence baseline; the two BINDING gates (G1/G1.1 legacy+revised,
and G3) were NOT part of the training run and were run this cycle
against the saved checkpoint (`rl_move.dynamics.eval_model --split
test` + `probe_latents.py`, both cheap/no-training): **G1/G1.1 PASS
outright at every horizon** (beats persistence AND matched linear
ridge at k=1/2/5, beats unchanged-latent at k=10/25 — e.g. k=1 MSE
0.079 vs ridge 0.139, persistence 0.231; no G1.1 tolerance even
needed) and **G3 PASS** (linear probes from z recover roll/pitch/gyro
R²=0.90–0.99, body velocity R²=0.90–0.95, chassis height R²=0.98,
per-foot contact bal-acc 0.97–0.98, all far above the shuffled-chance
floor). One honest gap: `cos_yaw_rel` R²=−0.04 (degenerate; `sin_
yaw_rel` only 0.72) — command-frame yaw phase isn't cleanly encoded,
everything else is. Evidence: `logs/ckpt_eval/
cw_dynrep_tf_state2_recovered1/{eval_g1_test,eval_g1_g3dump_test}.json`
+ `g3_probe.txt`. Does not change the product baseline (no policy
shipped) and was NOT wired into a PPO condition A/B/C comparison this
cycle — that would be a genuinely new, not-yet-pre-registered
experiment (6 of the 9 nominally-"free" GPU pods are actually busy
with the GRU-encoder A/B/C cohort, invisible to `capacity.py`/`ops.sh
census` because `train_ppo_transfer` lives under `rl_move.dynamics`,
not the `rl_move.sim.train_ppo_*` pattern those tools match — confirmed
live via direct `kubectl exec` /proc reads on train-4/5/6/7/8/9).
**Next candidate (not launched, needs its own hypothesis+gate):** wire
this Transformer encoder into a condition B/C PPO transfer arm and
compare sample-efficiency/gait-quality against the running GRU-encoder
cohort — same A/B/C ladder the operator already asked for, new axis
(architecture) instead of scale.

**08-15 ~20:2x UTC (independent discovery, this cycle): the
`risewalk-single` cohort (seeds 5/6/7 on train-4/5/6, launched ~18:19
UTC) was DEAD, silently, for ~45 min — a THIRD distinct code bug in
the same-day scaling work, this time in the PPO side, not the
pretraining side.** `pod_risewalk.sh`'s manifest showed
`phase_start condition C` immediately followed by `done seeds_done=0`
on all three pods (~19:33 UTC); `ops.sh census`/`ops.sh procs` missed
it because `train_ppo_transfer` matches the `*train_ppo*` filter only
while alive — once dead it just looked like 3 more idle pods (no
SUSPECT/DEAD checkup fired because the process exited cleanly with a
traceback, not a hang). Root cause (`rl_move/dynamics/logs/
risewalk_s5.log`): `KeyError: 'contact_now'` in `model.dynamics_loss`,
called from `train_ppo_transfer.py`'s condition-C `AnchorCb.
_on_training_start`. `dynamics_loss` grew "current"-state heads
(`contact_now`/`current_now`/per-horizon `current`, commit `6a8560c0`
"Add GPU transformer dynamics pretraining", 17:15 UTC) but the PPO
-side batch converter (`anchor_batch_to_torch`) was never updated to
forward those keys from `WindowSampler.batch()` — a real gap, not
config drift: it silently breaks EVERY condition-C PPO run launched
on code synced after 17:15 UTC. `dynrep-futurewalk-C-s5/6/7`
(train-7/8/9, still healthy) never hit it only because they started
at 16:33 UTC, before the regression, and a running Python process
doesn't reload edited source. Fixed same cycle (`anchor_batch_to_torch`
hoisted to module scope + now forwards `contact_now`/`current_now`/
`current`/`priv_mask_now`; new `test_dynrep_ppo_anchor.py` drives the
REAL `WindowSampler` + a tiny real `DynamicsModel` through the exact
`AnchorCb` path with no GPU/pod dependency — fails on the pre-fix code,
green after; full dynrep+dynamics suite still green), snapshot
`exp/cw-dynrep-fix-anchor-batch-current`. Retried once (DEAD ->
clean-up-and-retry-once per the shutdown protocol): killed the
orphaned multiprocessing zombies, synced the fix to train-4/5/6,
verified the G1/G1.1-PASS encoder+dataset still present, relaunched
`risewalk-single2` (same seeds, one per pod, `pod_memwatch.sh`
alongside) — all 3 confirmed live via W&B under the now-unique
per-attempt name (`rw_rise_A_s5.0815-2022Z` etc., commit `d93ee431`'s
attempt-stamp fix already covers the collision), condition A rise
phase advancing normally. Not this cycle's assigned run (that was
`cw-dynrep-tf-state2-fresh3-data`, already fully triaged+superseded by
a concurrent cycle before I reached it — see the entry below); this is
a genuinely new, independently-discovered dead cohort, fixed and
retried per the 08-14 drain-before-backoff rule.

**08-15 ~20:1x UTC: `cw-dynrep-tf-state2-fresh2` died a SECOND time
(pod train-10 hard-OOMKilled at the 96Gi cgroup limit, ~65s after its
stage-1 collection cleanly logged `data/complete=1` meeting the gate --
10,240,039 train windows, reuse 1.9999x, wandb `flaf42k7`), and this
time ROOT-CAUSED, not just retried blind: `data.load_dataset` indexed
`z["frames"]`/`z["actions"]`/`z["priv"]` freshly INSIDE the per-episode
loop, and `NpzFile.__getitem__` allocates the complete member array on
every call with no caching -- a ~2048-episode shard re-allocated its
full frames/actions/priv arrays up to 2048 times each, exhausting a
96Gi pod loading an ~8-9GiB corpus. This ALSO FALSIFIES the earlier
"stacking, not a collector leak" read on the FIRST fresh death (train-1):
fresh2 died solo, no co-resident job, so the bug was always in
load_dataset. Fixed directly (commit `3cd6c57a`, "Load each dynrep
shard member once" -- caches each npz array once per shard). Same
cycle added durable forensics regardless of root cause: `memutil.py`
(host RSS + cgroup memory.current/max), wired into `collect_mjx.py`'s
periodic/final logs and `train.py`'s `main()` now calls `wandb.init`
BEFORE the memory-heavy load/stats/sampler/model steps (previously
after -- meaning a crash in that exact stretch, like this one, left
literally no durable trace; the stage-2 W&B run for fresh2 never even
existed) so every future crash there leaves a `mem/*` breadcrumb;
`pod_memwatch.sh` poll tightened 60s->10s. A `launch_run.py respec`
retry (`cw-dynrep-tf-state2-fresh3`) then hit a SEPARATE, previously
latent launcher bug: `respec` unconditionally appended `--out-name`
(a ppo-only convention, unguarded for dynrep sources) so
`fresh_pipeline`'s stage-2 `os.execv` into `train.py` crashed on
argparse; fixed (`respec` now skips `--out-name`/`--init-from-source`
for dynrep/dynrep-fresh sources) and recorded FAILED. Superseded in
real time by a concurrent operator/session action that reasoned the
same way one level further: the `v5_mjx_fresh` corpus survives on
shared storage across the pod that died collecting it, so no
recollection is needed at all -- `cw-dynrep-tf-state2-recovered1`
launched straight against `rl_move.dynamics.train` (skipping
`fresh_pipeline` entirely), confirmed RUNNING (wandb `vt2ovznc`,
train-11) and registered in the ledger this cycle (it had none, having
been launched outside `launch_run.py`). This is the line to watch now,
not fresh3.

**08-15 ~19:1x UTC (operator-kick cycle: cw-dynrep-tf-state2-fresh
verification): the operator's fresh-data Transformer launch DIED and
was retried once — now RUNNING as `cw-dynrep-tf-state2-fresh2` on
train-10.** Chain: the ordered train-3 launch was REFUSED (pod busy
with cw-arch-tf-joymodes-scratch1-acq1); a concurrent cycle re-placed
it on train-1 at 18:36, where stage-1 collection OOM-killed the WHOLE
pod at 18:52 (96Gi limit, 1.18M/10.24M windows, W&B wttfxanc crashed,
overlay + shards lost — third pod-OOM this week). Confound: the
launcher placed it next to a heavyweight co-resident eval
(cw-mt-b1-dualgru1) because it only counts trainers when calling a pod
free. Retry-once per protocol: identical recipe, clean pod train-10,
`pod_memwatch.sh` (85GiB guard) alongside, CUDA-torch capability
installed+recorded on train-10 first. Measured on the retry: collector
memory grows ~2GiB per 1M windows over a ~6GiB base → ~50GiB projected
peak, safely inside limits — implicating STACKING (eval + collector),
not a collector leak, for the train-1 death. Retry passed the parent's
death point at 19:09 and was at 2.34M windows / reuse 8.7 by 19:13,
five-actor + 4-level-DR recipe verified from config, both gate metrics
(`data/train_windows`, `data/planned_window_reuse`) logging on W&B
`flaf42k7`. Fleet repair in the same cycle: train-1 recreated on
g131eec, bootstrapped, CUDA-torch reinstalled + re-recorded (the old
record died with the pod). Stage 2 (13.62M-param CUDA Transformer,
val + contact Brier/ECE vs the telnzd5r divergence) starts when
collection hits 10.24M windows — verification continues this cycle.

**08-15 ~18:1x UTC (orchestrator repair): the A/B/C `rw_rise_C_s5/6/7`
cohort that the operator emergency-launched at 16:06 on train-10 died
with the pod (OOMKilled same-day — train-10 went `Failed`,
self-recreated ~18:04 UTC with an empty overlay fs, all in-flight
checkpoints/logs lost), exactly the 08-14 OOM-incident mechanism:
`pod_risewalk.sh` runs `SEEDS="5 6 7"` as 3 PARALLEL
subshells on one pod, and the encoder+`v3scale_large` anchor dataset
loads once per process — the same multiplication `futurewalk-C` hit
before its 16:33 fix (one seed per pod).** `check_cohort.py --cohort
risewalk` on the (dead) pod showed no manifest / no live process — a
genuine NOT-LAUNCHED state per the 08-14 directive, not just "known,
leave it." Repaired same-mechanism-as-futurewalk-C: pulled the G1/
G1.1-PASS `dyn_scale_M_h16_large.pt` + `v3scale_large` (359M) +
gate record off the still-running `dynrep-futurewalk-C-s5` pod
(train-7, matched paths, no need to touch the dead pod) onto 3
genuinely-idle GPU pods (train-4/5/6, confirmed via `/proc` — no
hidden dynrep process, <15GiB baseline of ~96GiB), synced code via
`snapshot.sh --sync` (the pods' dynamics tree predated the
manifest-wiring + train_ppo_transfer W&B-init landed 16:34), and
relaunched **one seed per pod** (`COHORT_NAME=risewalk-single`,
`SEEDS=5`/`6`/`7` on train-4/5/6 respectively) with
`pod_memwatch.sh` running alongside on each (85GiB kill-one-job
guard, in case the single-process footprint still surprises us).
Verified live: `/proc` shows `train_ppo_transfer --condition A --task
rise --seed {5,6,7}`, manifests writing (`risewalk-single_manifest.jsonl`,
`start`/`seed_start`/`phase_start` events present),
`check_cohort.py` reports `live_train_ppo_transfer=1` on each,
W&B runs live (`rw_rise_A_s5` zwjf3jc2 + siblings, `l2k2/
hexapod-balance`), memory 12-15GiB/pod after ~20s (nowhere near the
96GiB limit with only one process). Names differ slightly from the
original launch (`rw_rise_A_s{5,6,7}` starting condition A first —
same script, same per-seed A→B→C→(walk warm-start) sequence, just
one seed's 6-phase chain per pod instead of 3 seeds' worth
competing for one pod's RAM) — triage this as `risewalk-single`
when it reports, not against the original `rw_rise_C_s5-7` naming.
Old train-10 is back `Running` (recreated manually by a concurrent
orchestrator cycle via `kubectl delete`+`apply` ~18:04 UTC, restart-
Policy:Never bare pods do not self-recreate — not "self-recreated" as
first logged here) and free; not reused for the risewalk cohort by
design (no evidence yet the 1-process-per-pod fix is sufficient
long-run, don't put a second copy on the pod that just OOMed).

**08-15 ~18:1x UTC (separate concurrent cycle, kick_20260815T172346_ea7b20
review):** the kick asked the orchestrator to execute a fresh CUDA
causal-Transformer dynrep pretrain (`cw-dynrep-tf-state1`, arch=
transformer, 40k steps, full 14-priv-label data `v4priv14`, PID 137013
collecting on train-10). Found already fully executed by the
operator's own live Codex session, not by this orchestrator: the run
launched+verified RUNNING on train-3 (train-10 having died mid-
collection, see above — the "PID 137013" collection is unrecoverable,
no persistent volume), substituting a quick single-seed 400-episode
`v4priv14_recovery` set (genuine 14-priv-label, not legacy 4-label —
kick's hard requirement honored) to unblock the architecture proof;
the operator then watched it live and KILLED it at ~21k/40k with a
real verdict already on the ledger + W&B notes: confirmed overfitting
(train total <0.75, held-out total 5.53->8.22 from step 1000), next
step named ("preserve the best checkpoint, add early stopping and
stronger regularization, then rerun under a new append-only name").
Nothing left to triage. Left the named regularization/early-stopping
follow-up to the operator's own session (actively engaged on this
exact file/run this minute — avoid duplicate-edit collision on
train.py). This cycle's distinct contribution: repaired train-10
(independently of the risewalk-repair cycle above; different
pods, no collision) and relaunched the FULL 12-seed x 400-episode
(4800 ep) `v4priv14_full` collection the kick actually asked for
(pod_scale_sweep.sh recipe, `noslip:0.05` actor-share required,
`pod_memwatch.sh` riding shotgun this time), so the operator's named
rerun has real full-scale data instead of the 400-ep recovery set
when they get to it. In flight at cycle end (~245/400 eps per seed);
not blocking anything, first free pod picks up triage when it lands.

W&B: tag `track:dynrep`, run prefix `cw-dyn-`. Design doc + binding
gates: **rl_docs/DYNREP.md** (read before triaging anything here).
Code + runbook: `rl_move/dynamics/README.md`.

**Goal:** learn a task-independent latent of the hexapod's body
dynamics by self-supervised action-conditioned multi-horizon
prediction on diverse sim rollouts (failures included), then test
whether PPO reusing that representation (frozen / continually
anchored) learns new motor skills with fewer env steps than PPO from
scratch. Primary metric: sample efficiency on a NEW task.

## Now (08-14 ~02:2x UTC: operator next-steps EXECUTION — two agent
## sessions in parallel, division of labor below)

- **COORDINATION (08-14 ~02:2x UTC):** two concurrent local sessions
  are executing the operator's DYNREP_NEXT_STEPS directive. Session A
  (this update) has LANDED (commit 08ba040): **G1.1 revised gate
  recorded prospectively** (eval_model.py `--k1-ridge-tol` 0.05,
  `gate_g1_1_pass` in the report JSON + console line, rationale in
  rl_docs/DYNREP.md — historical v2pod FAIL verdicts unchanged),
  model/train scaling knobs (hidden/act_hidden/gru_layers, stored in
  ckpt config), merge_shards.py (parallel per-seed collection +
  --require-actor drift guard), pod_v3_pipeline.sh, pod_scale_sweep.sh
  + analyze_scale.py, and snapshot.sh --sync now EXCLUDES
  rl_move/dynamics/{datasets,models,logs} from the code tarball.
  Session B (uncommitted, in flight as of 02:1x UTC) has collect.py
  hard-fail-on-degraded-mix, pod_pilot_rep2.sh (v2pod2 recollect +
  encoder, gated on ORIGINAL G1 from the report JSON — correctly NOT
  the revised G1.1, no post-hoc weakening), and the mode_seq
  canonical-segment-frames fix (trans-dagger2) + verify/eval tooling.
  **Division to avoid double-launch: the drift-fix replication runs
  via session B's pod_pilot_rep2.sh (train-11 left free for it;
  pod_v3_pipeline.sh stands down as the duplicate); session A runs the
  SCALING sweep on train-10 (claimed 02:2x UTC) + the eval
  instrumentation (gait/transition quality, rise canary, held-out
  dynamics) + representation probes.** Re-sync code to a pod only
  between pipeline stages, never mid-cohort.
- **NEW drift finding (08-14, train-11 meta.json + collect WARNINGs):
  the v2pod drift was WORSE than recorded — `ppo_goal_cw_stance_dr10.
  zip` was ALSO missing on the pods**, so the stance actor's 0.20
  share fell back to RANDOM: v2pod's actual mix was random 0.50 /
  tripod 0.25 / walk 0.25 (zero stance-champion episodes, zero noslip)
  vs the recipe's 5-actor mix. Both G1-failed pod encoders trained on
  that. The stance champion (md5 da1d912a…) is now pushed to train-11
  AND train-10; collect preflights hard-require both champions +
  noslip_gait.
- **SCALING SWEEP LAUNCHED (train-10, 02:13 UTC, STAGE=all,
  hands-off):** 12 parallel collects (seeds 0-11, 4800 eps; small=s0-2
  merged with the noslip guard) then the 12-cell matrix S/M/L
  (~0.8/5.9/17M params) x H16/H48 x small/large, each cell trained
  40k matched steps + gate-evaluated (G1.1 AND legacy G1 recorded).
  Logs: `logs/scale_all.log`, per-cell `logs/dyn_scale_*_{train,gate}`,
  summary `logs/scale_summary.txt` (analyze_scale.py). Triage rule:
  the deliverable is the prediction scaling CURVE + probe quality —
  no cell earns PPO wiring without the normal gate + A/B/C.
  **GPU RELAUNCH (02:41 UTC):** the pods' system torch is CPU-ONLY
  (`2.13.0+cpu` — collects were fine, but the training matrix was
  ~3 steps/s). Killed the CPU sweep after collect/merge finished,
  built `/workspace/venv_torchgpu` (`--system-site-packages` +
  `pip install --ignore-installed torch` → `2.13.0+cu130`,
  `cuda.is_available()=True`; plain `pip install torch` is a no-op
  because the venv sees the system CPU wheel), and relaunched
  STAGE=sweep with `PYTHON=/workspace/venv_torchgpu/bin/python
  PYTHONUNBUFFERED=1` → ~59 steps/s on the H200. New log:
  `logs/scale_sweep_gpu.log`. NOTE for future pod training: use this
  venv (or recreate it the same way) for any torch training on the
  mjx-train pods; system python3 trains on CPU silently.
- **FIRST DRIFT-FIXED GATE VERDICT (03:54 UTC): dyn_scale_S_h16_small
  — LEGACY G1 PASS outright** (beats persistence AND matched ridge at
  every horizon; G1.1 tolerance not even needed). The SMALLEST cell
  (~0.8M params, H16, 1200 eps) on the drift-fixed 5-actor mix passes
  the gate both v2pod encoders failed — strong confirmation that the
  v2pod G1 FAILs were dataset drift (periodic-heavy mix strengthening
  the ridge baseline), not model capacity. Session B's v2pod2
  replication remains the formal original-recipe/original-gate
  confirmation.
- **POD OOM INCIDENT + FULL RERUN (08-14 ~09:40 UTC OOM, 12:33 UTC
  relaunch):** train-10 was OOMKilled at its 96Gi pod limit ~3h after
  the sweep finished, i.e. during the chained holdwalk cohort (exact
  offender unknown — pod logs died with the pod). The train pods mount
  NO persistent volume (/workspace = container overlay fs), so the
  ENTIRE first sweep run was lost: v3scale datasets, all 12 cell
  checkpoints + gate records, scale_summary. The one result that
  survives (committed here + in the eval JSON quoted above) is
  dyn_scale_S_h16_small's legacy-G1 PASS. Recovery: pod deleted +
  re-applied from coreweave_pods_mjx_scaleout.yaml, re-bootstrapped
  (bootstrap_train_pod.sh), stance champion re-pushed (md5 da1d912a…),
  GPU venv rebuilt, and the whole pipeline relaunched (STAGE=all sweep
  + chain watcher). NEW GUARD: pod_memwatch.sh runs alongside — logs
  container memory.current + top-RSS process every 60s to
  logs/memwatch.log and above 85GiB kill -9's the single largest
  python, so a runaway loses ONE job loudly instead of the whole pod
  silently. LESSON: pull artifacts (gate JSONs, summary, champion-cell
  checkpoints) off-pod promptly; nothing on /workspace survives a pod
  kill.
- **SCALING SWEEP COMPLETE (rerun, 16:37 UTC): 12/12 cells PASS the
  ORIGINAL legacy G1** (every size x history x data cell beats
  persistence AND matched ridge at every horizon on the drift-fixed
  mix — the G1.1 tolerance was never needed once the data was right).
  Scaling curve (k1 model/linear MSE, lower better): DATA dominates
  (M h16: 0.126 on 4800 eps vs 0.165 on 1200), HISTORY helps modestly
  (M large: h48 0.116 vs h16 0.126), SIZE saturates at M — L (~17M)
  is consistently slightly WORSE than M (~5.9M) at matched 40k steps,
  and on small data size does nothing. Verdict: spend future encoder
  compute on data + context, not parameters. L_h48_large trained
  clean this run (peak container ~57GiB) — the 08-14 OOM culprit
  therefore remains unidentified; memwatch stays on. Artifacts pulled
  OFF-POD to the laptop (logs/scale_run2/: summary, sweep log, all 12
  gate records + eval JSONs; models/: dyn_scale_M_h16_large.pt md5
  195285c0…, dyn_scale_M_h48_large.pt md5 06fce9c5…).
- **A/B/C COHORT LAUNCHED (chained automatically 16:38 UTC):**
  pod_chain_abc.sh picked dyn_scale_M_h16_large (per the fixed
  preference order; near-best cell) + v3scale_large, SEEDS="5 6 7" —
  hold 150k then walk 1M per condition, eval-tasks hold,walk,rise +
  --eval-heldout. Logs: logs/holdwalk_s{5,6,7}.log + per-run eval
  CSVs. ETA ~3h/seed (subshells parallel). Triage: analyze_pilot
  --phase2 walk after pinning a threshold from the first curves; the
  rise-retention canary and heldout suites are the primary hypotheses.
- **A/B/C COHORT CLAIMED BY SESSION A (train-10, chained):**
  `pod_chain_abc.sh` (nohup'd 03:02 UTC, `logs/chain_abc.log`) waits
  for POD_SCALE_SWEEP_DONE then launches pod_holdwalk.sh with the
  first gate-passing cell in a FIXED preference order declared before
  those verdicts were known (M_h16_large first — mid capacity, short
  history, large data), SEEDS="5 6 7" (fresh seeds per operator rule),
  DATA=v3scale_{large,small} matched to the cell. Session B: do NOT
  also launch a holdwalk cohort; key any rep2 follow-up to v2pod2
  explicitly.
- **EVAL INSTRUMENTATION LANDED (train_ppo_transfer.py, smoke-tested
  locally incl. every new column):** (1) per-task gait/transition
  QUALITY metrics at every eval point — slip_m, fwd_m, peak_roll/
  pitch_deg, peak_gyro_dps, contact_sw_per_s, slew_sat (joint-ticks at
  the safety rate limit) + slew_sat_all (>=6 joints simultaneously —
  the takeoff posture-snap signature), mean_h_m, dh_m (rise-gain
  canary signal), vx_rmse; (2) task "rise" (p_rise=1.0, belly/bridge
  starts) usable as trained task AND as --eval-tasks retention canary;
  (3) --eval-heldout: fixed held-out dynamics suites on the trained
  task every --heldout-every (50k) — dr10 (broad DR 1.0), lat2x,
  vel07, db25, tq07 via the campaign's cfg dr.<field> override
  mechanism (isolated axes, same as the cw-walk-latjit25 precedent).
  pod_holdwalk.sh phase 2 now runs --eval-tasks hold,walk,rise +
  --eval-heldout and accepts G1 OR G1.1 on the gate record;
  pod_risewalk.sh (NEW) is the operator's actual-objective benchmark:
  rise 500k from scratch -> walk 1M warm-started, rise retention as
  the first-class hypothesis (expected strongest-positive pattern: A
  loses rise + rolls/slips more, C retains rise + cleaner walk).
- **G3 probes: first real result (probe_latents.py, laptop
  dyn_v2_obs latents, 4096 held-out windows):** ridge probes from z
  recover roll/pitch R² 0.96/0.97, gyro x/y/z R² 0.97/0.96/0.92, and
  **per-foot contact at 0.90-0.95 balanced accuracy (chance 0.50) —
  with contact forces NOT in the obs input set**: the encoder infers
  support state from proprio/action history. n_feet_on R² 0.47 (count
  is harder linearly). This is the strongest G3 evidence yet that the
  latent organizes attitude, angular rate and support state.
- **Launch ordering for the cohorts:** pod_pilot_rep2.sh (session B,
  train-11) must land its ORIGINAL-G1 verdict first. On PASS:
  pod_risewalk.sh (ENC/DATA pointed at v2pod2, seeds 1-3) on a free
  pod is the priority cohort per the directive ("the next dynrep task
  should NOT be another narrow hold->lower transfer"); pod_holdwalk
  seeds can follow on remaining capacity. On FAIL: the drift
  hypothesis is disconfirmed — escalate to the operator, do NOT
  gate-shop with G1.1, do not launch A/B/C.

## Previously (08-13 ~19:4x UTC: retry G1 FAIL — replicated; dataset
## fix is OPERATOR-side, no third seed)

- **The pre-registered seed-retry FINISHED and G1-FAILED the same
  way (`pod_pilot_rep_retry.sh`, tag exp/dynrep-podrep-retry1,
  encoder `dyn_v2pod_obs_s1`, seed 1): lost to the linear baseline
  at k=1 ONLY — model_mse 0.1718 vs linear 0.1673 (~2.7%), k=2/5
  and both latent horizons beat persistence AND linear, exactly the
  seed-0 pattern.** Seed-plumbing was ruled out before accepting
  the replication: the two checkpoints are md5-distinct, best val
  differs (2.9357 vs 2.9599), k=2/k=5/latent numbers differ — the
  4-dp k=1 match is rounding coincidence (physical q 0.35° vs
  0.36°). **Per the pre-registered fork this CLOSES training
  variance as the explanation: two independent seeds losing at k=1
  on the same pod dataset makes the KNOWN dataset drift (noslip
  actor's 10% share silently falling back to tripod on the pod —
  `noslip_gait.py` is laptop-only; more-periodic tripod data is
  exactly what strengthens a ridge regressor) the prime suspect,
  and the fix is OPERATOR-side: push `noslip_gait.py` to the pod
  or revise the v2 recipe. NO third seed (pre-registered).** The
  DYNREP.md hard gate held throughout: no PPO cohort ran on either
  failed encoder, so the pod direction-of-effect check vs the
  laptop A/B/C read has still not happened; hold→walk stays
  hard-blocked (its runner aborts without a G1 PASS on record).
  Artifacts on train-11:
  `rl_move/dynamics/logs/pod_pilot_rep_retry.log`,
  `logs/dyn_v2pod_obs_s1_gate.txt`,
  `logs/eval_dyn_v2pod_obs_s1_20260813_191445.json` (+latents npz),
  `models/dyn_v2pod_obs_s1{,_final}.pt`.
  Expected if the laptop read holds, once a cohort runs on a fixed
  dataset: phase-1 final hold C > B > A; no phase-2 speed
  separation; retention A >= C > B.
- **hold->walk transfer pair: code LANDED this cycle, launch-blocked
  only on the rep landing + triage** (operator ordering). What
  landed: `walk` task in train_ppo_transfer.py (p_walk=1.0 pin, same
  env family, one variable per run preserved), `--eval-tasks`
  (default keeps the pilot CSV schema bit-exact; hold->walk cohorts
  pass hold,walk for free retention curves), eval-every default
  25k -> 10k (operator: NEW cohorts need <= 10k, seeds >= 5),
  `pod_holdwalk.sh` (seeds 1–5, reuses the rep's hold checkpoints
  for 1–3, trains hold for 4–5, walk at WALK_STEPS default 1M,
  hard-aborts without a G1 PASS on record), and
  `analyze_pilot --phase2 walk --phase2-threshold <thr>` (threshold
  pinned after first curves — walk return scale here is unmeasured).
  Smoke-tested end-to-end on the controller (env pinning, 1k-step
  A-condition runs both tasks, warm-start, both analyzer modes;
  legacy CSV schema verified unchanged).

## Previously (08-13 am: local 3-seed sweep on the REPAIRED hold task)

- **Hold-task fix first** (train_ppo_transfer.py `--term-penalty`,
  default 30): the 08-12 pilot's phase 1 was degenerate — every
  condition learned to tip at ~tick 35 because ending the episode
  beat holding badly. A one-time terminal penalty (TRAINING ONLY;
  evals always run the raw env, so CSVs stay comparable) removes the
  escape. On the repaired task all 18 runs hold genuinely: early-term
  rate 0.00 at every eval point, hold return −228 → strongly positive.
- **Local seed sweep DONE (seeds 0/1/2 × A/B/C × hold→lower, 150k
  each, laptop encoder + datasets/v2; run names `pilot_*_s{seed}`;
  aggregate with `python -m rl_move.dynamics.analyze_pilot`):**
  - Phase 1 (hold, from scratch): pretrained representations win on
    FINAL performance — **C 159±41 > B 111±42 > A 57±62** (mean ±
    half-range; one A seed never reaches positive hold return).
    Steps-to-threshold differences are within eval granularity (25k).
  - Phase 2 (lower, warm-started): **no acquisition-speed separation**
    (steps to lower≥250: A 75k±25k, B 83k±38k, C 83k±13k) and final
    lower is comparable (C 332±18, A 318±27, B 317±45).
  - Retention REVERSES the 08-12 single-seed read: **A keeps hold best
    (96±15), C 68±33, B worst (55±4)**. Plausible mechanism: B's
    trainable capacity is only the 0.07M head, so adapting to lower
    must overwrite the very weights that held; A's full 0.33M policy
    can host both. The 08-12 "B retains best" was an artifact of the
    degenerate phase-1 task (retention of a suicide policy).
  - C's anchor loss again stayed ~2.0–2.2 through both phases —
    the anchored encoder never left the predictive objective.
  - **Honest verdict so far: representation pretraining helps LEARN
    the task better (phase-1 final), not learn-it-faster or
    retain-it-better at this budget/task pair.** The brief's primary
    metric (sample efficiency on a NEW task) is NOT yet supported;
    higher final performance + C-best is. Curves:
    `logs/pilot_sweep.png`.
  - Do NOT pool these with the pod replication below (different
    encoder/dataset provenance) — but it DOES run the same repaired
    task (term-penalty default 30 rides in 4d26954), so it is a
    clean direction-of-effect check.

## Previously (08-12 late: PPO wiring + v2 + pilot)

- **Frame layout v2** (breaking, re-collected + retrained same night):
  q is stored RELATIVE to the episode's settled `q_nom`, because the
  policy obs contract is (q−q_nom) with q_nom captured per episode at
  reset — an encoder pretrained on absolute q (v1) can never be fed
  from the deployed obs. `ep_qnom` kept in shard metadata.
- **Residual short-horizon heads** (delta-state parametrization,
  model.py): the obs-input variant was losing to the full-history
  linear baseline at k=1 by ~2-3% (1-step dynamics is locally
  linear). Predicting the state DELTA from the newest frame makes
  persistence the head's zero output; k=1 joint-pos RMSE dropped
  1.7 deg -> 0.32 deg. **G1 PASS at every horizon for BOTH v2 models**
  (`dyn_v2`, `dyn_v2_obs` 40k steps + cosine LR decay; reports in
  `rl_move/dynamics/logs/`). G2 (obs input set) met.
- **A/B/C wiring landed** (`sb3_encoder.py`, `train_ppo_transfer.py`,
  `run_pilot.sh`): scratch vs frozen-z vs slow-LR encoder + offline
  dynamics anchor. Dual-task eval CSV at every eval point = retention
  curves for free.
- **sb3 gotcha (cost a debugging round, keep forever):**
  `ActorCriticPolicy(ortho_init=True)` (the default) orthogonally
  re-initializes every Linear INSIDE a custom features extractor
  after construction — it silently wiped the pretrained encoder
  (GRUs survive, so it looked half-trained). Fix:
  `DynFeaturesExtractor.reload_pretrained()` after fresh PPO
  construction; the anchor callback now prints the untouched anchor
  loss at start (must match pretraining val, ~2.0) as a tripwire.
- **Pilot cohort DONE (single seed, directional only)** — hold from
  scratch 150k, then lower warm-started 150k, matched everything;
  eval CSVs `logs/ppo_pilot_*_eval.csv`:
  - Phase 1 (hold) is a degenerate testbed at this budget: ALL three
    conditions found the tip-early attractor (no alive bonus ->
    ending the episode beats holding badly; ep_len collapses to ~35).
    Known reward-family attractor, not a condition discriminator —
    pod-scale phase should use the campaign's hold stack or a
    survival-gated variant.
  - Phase 2 (lower transfer) is where the signal is. Steps to lower
    return ≈200: **B ~50k, C ~110k, A ~117k** — the frozen
    pretrained z acquired the new task ~2.3x faster than scratch.
    Final lower return: **C 279 > A 260 > B 228**. Hold retention at
    end (return / early-term): **B 68.5 / 0.00, C 36.0 / 0.00,
    A 23.6 / 0.25** — both pretrained conditions kept the old task
    terminate-free, scratch didn't.
  - C's anchor loss stayed 2.04-2.19 throughout (pretraining val
    ~2.0): PPO fine-tuning never pulled the encoder off the
    predictive objective.
  - Reading AT THE TIME: consistent with the track hypothesis (B
    fastest acquisition + best retention; C best final performance).
    **SUPERSEDED by the 08-13 3-seed sweep above** — on the repaired
    task the acquisition-speed and retention advantages do not
    replicate; only "pretrained → better final hold" survives.

## Current numbers (v2, 08-12 night)

- Dataset `datasets/v2` (local, gitignored): 1,200 episodes / ~258k
  steps (~2.9 sim-hours), 3 collect seeds; actor mix random /
  walk-champion / stance-champion / tripod / noslip; ~30% of episodes
  end in falls/trips (kept on purpose); DR ∈ {0, 0.3, 0.6, 1.0}.
  (`datasets/v1` = retired absolute-q layout; delete freely.)
- `dyn_v2` (full input): k=1/2/5 state MSE 0.110/0.153/0.178 vs
  persistence 0.268/0.399/0.717 and matched linear 0.128/0.172/0.196;
  latent MSE 0.154/0.165 at k=10/25 vs unchanged-z 1.06/1.27.
- `dyn_v2_obs` (the PPO transfer candidate): 0.140/0.168/0.188 vs
  matched linear 0.154/0.191/0.213 — PASS everywhere. k=1 joint-pos
  RMSE 0.32 deg, tilt 0.14 deg, contact acc 0.93.
- G3 first probe (v1-era, re-run pending on v2): linear probes from z
  recover roll/pitch at R² 0.97/0.98, feet-on count R² 0.57.

## Next

- **OPERATOR SUGGESTIONS (08-13 ~19:5x UTC) — what to actually test
  with the representation, in this order:**
  1. **Does C produce a BETTER gait than scratch — not merely higher
     return?** Compare loaded-foot slip, roll, contact sequencing,
     joint slew saturation, servo currents, falls, and gait videos
     (A vs C, matched budgets). Given the track's current problems,
     gait quality matters more than sample efficiency.
  2. **Does C improve the stand→walk handoff?** This is suddenly a
     beautiful dynrep test. Start every policy from the deployed
     standing state at zero velocity, then engage walking. A
     recurrent representation has the history needed to know "I have
     just been standing with six feet loaded," which a plain
     instantaneous policy has less access to. Measure peak roll and
     simultaneous slew saturation during the first second.
  3. **Does the representation make the policy robust to
     actuator/model mismatch?** Randomize latency, servo speed,
     structural compliance, and contact; compare A vs C under
     HELD-OUT dynamics. This gets much closer to the actual reason
     for building a world model: sim-to-real robustness rather than
     leaderboard return.
- **OPERATOR DIRECTIVE (08-13 13:1x UTC, local sweep done — next
  pod work, in order):**
  1. The in-flight train-11 replication runs the REPAIRED hold task
     already: 4d26954 carries `--term-penalty` default 30 and
     `pod_pilot_rep.sh` doesn't override it. So its phase 1 is
     honest — triage it as a direct direction-of-effect check
     against the laptop sweep (still do NOT pool numbers: different
     encoder/dataset provenance). Expected if the laptop read holds:
     phase-1 final hold C > B > A; no phase-2 speed separation;
     retention A >= C > B.
  2. **If more seeds are queued, fix the eval granularity first:**
     eval-every <= 10k (the laptop's 25k grid can't resolve
     steps-to-threshold differences), seeds >= 5. — DONE 08-13
     ~13:3x: eval-every default is now 10k in train_ppo_transfer.py;
     pod_holdwalk.sh runs seeds 1–5.
  3. **Harder transfer pair next: hold -> walk** (the brief's real
     ladder). lower is too close to hold to discriminate — all three
     conditions transferred at the same speed locally. Walk budgets
     don't fit the laptop; this is the pod's job. — CODE READY 08-13
     ~13:3x (see Now); launch after the rep triage.
- **Seed replication IN FLIGHT (08-13 ~12:3x UTC, train-11 idle
  CPUs, `pod_pilot_rep.sh`):** the operator's code push (4d26954)
  unblocked the track, but datasets/models are laptop-local, so the
  pod pipeline regenerates the v2-recipe dataset + obs encoder
  (`datasets/v2pod`, `dyn_v2pod_obs` — deliberately NOT named v2;
  G1/G2 gates enforced before PPO wiring, hard-stop on FAIL), then
  runs the A/B/C cohort for seeds 1–3 in parallel. Do NOT pool the
  operator's s0 with these (different encoder/dataset provenance);
  compare direction-of-effect instead. Recipe drift to remember at
  triage: noslip actor share fell back to tripod (noslip_gait.py is
  laptop-only; collect.py degrades gracefully since 08-13). Log:
  `logs/pod_pilot_rep.log`; per-seed `logs/pilot_rep_s{1,2,3}.log`;
  summary `logs/pilot_rep_summary.txt` when done.
- Then pod-scale budgets for the
  brief's real task ladder (stand → forward walk → yaw → recovery) —
  local Mac budgets cannot reach walking. The laptop `--term-penalty`
  fix (or the campaign hold stack) should carry over: without it
  phase 1 measures suicide speed, and the 08-13 local sweep shows the
  conclusions flip once the task is honest.
- Online-window dynamics anchor for C (currently offline-replay).
- G3 proper on v2: per-foot contact probes + 2D embedding of
  standardized trajectories (upright/fallen, tipping direction, gait
  phase).
- Latent-size ablation (64/128/256) only after the first A/B/C
  comparison lands.
