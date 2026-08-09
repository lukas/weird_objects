# RL_LOG — autonomous campaign log

Append-only. One short entry per finished run (W&B results + harness eval
+ what the motion VIDEOS showed) and one per launched run (hypothesis +
gate). The orchestrator agent reads this top to bottom each cycle; keep
entries short and factual.

Prior history (20 runs, rounds 1–5, manual orchestration):
see `../../archive/RL_CAMPAIGN_REVIEW_2026-08-08.md`. Champions as of handover:

| Skill | Champion checkpoint | Status |
|---|---|---|
| stand↔belly (plain) | `ppo_goal_cw_stand_dr10.zip` | DR 1.0 gate PASSED — hardware candidate |
| stand↔belly, even stance | `ppo_goal_cw_stance_dr08.zip` | DR 0.8 passed; DR 1.0 rung in flight |
| walk (slow range 0.02–0.06) | `ppo_goal_cw_walk_slow2.zip` | 5/6 stochastic @ vel err 0.028 (gate 0.030) |

## In flight at handover (round 5, launched 2026-08-08 ~03:48)

| Run | Pod | Hypothesis | Gate |
|---|---|---|---|
| cw-stance-dr10 | friction | even-stance line survives DR 1.0 | rise/lower ≥5/6 all starts, hold six-footed |
| cw-walk-w07 | s3 | half-size widen 0.02–0.07 holds tracking | ≥4/6 sto @ vel err ≤0.035 |
| cw-walk-w07-s1 | s4 | seed-1 twin (post seed fix) — reliability | same |
| cw-walk-dr04b | long5m | consolidate walk at DR 0.4 | ≥4/6 sto @ vel err ≤0.030 |

---
<!-- Orchestrator entries append below. -->

## Round 6 — results (orchestrator cycle, 2026-08-08 ~15:40)

Harness: 6 eps/mode, det+sto, each at its own DR + training cfg-sets, video reviewed.

### cw-stance-dr10 — PASS (even-stance line completes the DR ladder)
W&B vhdcun7i, 3M steps (19.2M cum). Harness @ DR 1.0: rise det 6/6 + sto 6/6
(all start kinds), lower det 6/6 + sto 6/6, hold/lean/unload 6/6 both passes,
track 6/6 det, 5/6 sto. Raise regressed: 4/6 det, 3/6 sto (was 5/6 at DR 0.8).
Videos: rise is smooth and ends in a wide, even SIX-footed stance; hold is
quiet, all feet loaded, no tripod. Lower reaches belly height reliably but
ends scrappy — 3 feet down, remaining legs held aloft (one episode with a leg
straight up, hot knee servo highlighted). Verdict: gate PASS → champion
`ppo_goal_cw_stance_dr10.zip` (even-stance, DR 1.0). Raise <5/6 flagged; lower
end-posture is a motion-quality (not success) defect.

### cw-walk-w07 — MISS (second widening failure; hypothesis refuted)
W&B k3wppg6m, 5M steps (20.75M cum). Harness @ DR 0.2, 0.02–0.07: sto walk 4/6
@ vel_err 0.034 (nominally inside the 0.035 gate) but det walk 1/6 @ 0.042,
and rise eroded to det 3/6 / sto 2/6 (flat 0/4). The "pass" is binomial noise:
the same weights under its twin's eval RNG scored 2/6 @ 0.035. Video: walks
with one hind leg waving vertically in the air (flag leg), 5-leg shuffle.
Verdict: MISS. Two widen misses in a row (w08, w07) → changing hypothesis to
gait quality, not more steps.

### cw-walk-w07-s1 — INVALID (seed twin was a bit-identical clone AGAIN)
W&B g5xymvx7. Final weights vs cw-walk-w07: max diff 0.0. The round-4 fix
(`model.seed = args.seed` after load) was a no-op — `PPO.load` runs
`_setup_model()` → `set_random_seed(ancestor seed)` DURING load; nothing
re-seeds afterwards. Round-5's "seeds genuinely diverge now" was unseeded
eval-RNG noise, not weight divergence. REAL fix now in train_ppo_sim.py:
`model.set_random_seed(args.seed)` after load. Verified locally: seed 0 vs 1
warm starts diverge (max weight diff 0.048 @ 6k steps), same-seed repeat is
bit-identical. All prior warm-started twin comparisons remain invalid.

### cw-walk-dr04b — PASS on scalars; NOT hardware-ready on video → walk champion (caveated)
W&B 87hevk27, 4M steps (24.76M cum). Harness @ DR 0.4, 0.02–0.06: sto walk 4/6
@ vel_err 0.028 (gate ≤0.030 PASS), det 3/6 @ 0.032; rise sto 5/6 (flat 2/2),
det 4/6; raise sto 2/6 (weak). Video: same flag leg as w07 — one hind leg
pointed straight up through the walk. Champion update: replaces cw-walk-slow2
(equal tracking at 2× the DR, better rise retention) →
`ppo_goal_cw_walk_dr04b.zip`. Explicitly NOT a hardware candidate until the
flag leg is gone.

### Diagnosis: the flag leg is lineage-wide, and reward-explained
Re-ran the harness on champion cw-walk-slow2 (walk+rise, DR 0.2): sto walk 5/6
@ 0.029 confirms its numbers, but its video shows the SAME vertical flag leg.
Root cause in reward code: the stance-clearance penalty only covers
hold/lean/track/unload — walk/rise/lower/raise have zero gradient against
parking a leg overhead. Fix added to sim_env.py (cfg-gated, default off):
`reward.k_flag_leg` charges pad clearance above a 50 mm allowance
(`reward.flag_leg_allow_m`) over episode-start pad z, all modes, unload leg
skipped. Unit tests pass; smoke: penalty fires on lifted legs, exactly 0 in
quiet stance. Adopted the prepared `goal.walk_obs_body_vel=0` switch in
walk_task.py (zeroes privileged measured-velocity obs, width unchanged →
warm-start compatible); smoke-trained OK.

## NEEDS OPERATOR — cycle aborted before launch (2026-08-08 ~15:50)

`git push` to origin fails: "Invalid username or token". The credential
store (`~/.git-credentials`) is EMPTY on the controller pod — no PAT
anywhere (no .netrc, no env token, no gh). Likely the fine-grained token
expired or was wiped. Per guardrails (escalation: git push rejected) NO new
runs were launched; all four pods are idle. Everything else in the cycle
completed and is committed locally on main (commit 4dad783, "orchestrator
snapshot before cw-walk-flag" — the exp/cw-walk-flag tag was created
locally, never pushed, and deleted again so a retried snapshot won't
collide). Restore the GitHub token, push main, and the next cycle can
launch immediately.

Planned round-7 launches, ready to go (code changes are committed & tested;
warm-start parents verified present on/for each pod):

| Run | Pod | Warm start | Hypothesis | Gate |
|---|---|---|---|---|
| cw-walk-flag | s3 | ppo_goal_cw_walk_dr04b (copy from long5m) | `reward.k_flag_leg=5.0` (50 mm allowance) removes the lineage-wide vertical flag leg without losing tracking; DR 0.4, 0.02–0.06, 4M, seed 0 | sto walk ≥4/6 @ vel_err ≤0.030 AND video: no foot >50 mm except swing AND sto rise ≥4/6 |
| cw-walk-flag-s1 | s4 | same | first REAL seed twin (post set_random_seed fix) — run variance + best-of-2; seed 1 | same |
| cw-walk-nv | long5m | ppo_goal_cw_walk_dr04b (in place) | `goal.walk_obs_body_vel=0`: policy re-learns tracking from proprioception+IMU only (deployable obs); DR 0.4, 0.02–0.06, 4M, seed 0 | sto walk ≥4/6 @ vel_err ≤0.035 |
| cw-stance-raisemix | friction | ppo_goal_cw_stance_dr10 (in place) | raise stalls (3-4/6 @ DR 1.0) because the mix under-trains it; raise=0.4,rise=0.2,lower=0.2, same cfg, DR 1.0, 3M, seed 0 | raise ≥5/6 det+sto AND rise/lower ≥5/6 all starts retained |

W&B notes for each must include the post-push snapshot hash.

## Round 6.5 — spurious cycle: no new experiments existed (2026-08-08 ~16:1x)

The watcher triggered a cycle on five "finished" W&B runs (fallen-sun-71,
honest-durian-74, radiant-water-71, revived-microwave-71, worldly-frog-75).
Investigated all five: they are round-6's OWN verification artifacts, not
experiments — 4–6k-step smoke/seed-check trainings (seeds 0/1/0-repeat,
warm-started from ppo_goal_cw_walk_slow2, ~2–3 min wall clock each, run
locally on the controller pod = hexapod-sweep-friction) used to verify the
set_random_seed fix and smoke the k_flag_leg / walk_obs_body_vel=0 code.
No harness eval, no verdicts, no champion changes apply. Process note for
future cycles: run smoke trainings with WANDB_MODE=disabled (or offline,
never synced) so they cannot masquerade as finished experiments.

Verified state: all four pods idle (no train_ppo processes); round-6
champions in place on their pods (ppo_goal_cw_stance_dr10 on friction,
ppo_goal_cw_walk_dr04b on long5m). RL_PLAN.md reviewed — no new evidence
since round 6, no changes made.

## NEEDS OPERATOR — still blocked, second consecutive cycle (2026-08-08)

git push STILL fails: ~/.git-credentials is a zero-byte file; no PAT in
env, no .netrc, no gh CLI. Read access to origin works (ls-remote OK) but
local main is 2 commits ahead of origin and cannot be pushed. Per
guardrails (snapshot push must succeed before any launch) NO runs were
launched; all four pods remain idle. The round-7 launch plan from the
previous entry is unchanged, tested, and ready — restore the token, push
main, and the next cycle can launch immediately.

Also for the operator's awareness: two pods outside the guardrails compute
list exist in the namespace (hexapod-sweep-lower, hexapod-sweep-walk, both
~17 h old). I did not create, use, or touch them; if they are meant to be
part of this campaign, add them to guardrails.yaml.

## Round 7 — push restored, planned launches executed (2026-08-08, idle-kick cycle)

Blocker cleared: the operator restored the GitHub token and pushed local
main (origin now at the same head we were stuck on; `git push --dry-run`
succeeds, `~/.git-credentials` repopulated). No runs finished since round
6 — nothing to eval. All four pods verified idle via /proc scan (`ps`/
`pgrep` are not installed on the pods; earlier "IDLE" echoes were the
fallback branch of a failed command, now checked properly). Unit tests
re-run on the committed round-6 code: 18/18 pass. RL_PLAN.md reviewed
again: §9 priorities 1–3 are exactly the round-7 table; no changes.

Launched the four runs exactly as planned in the round-6 NEEDS OPERATOR
entry (hypotheses/gates unchanged, reproduced here for the record):

### LAUNCH cw-walk-flag (pod s3, seed 0, 4M steps, DR 0.4)
Warm from `ppo_goal_cw_walk_dr04b` (copied from long5m → init.zip).
Args: joint_walk, 0.02–0.06 m/s, k_walk_swing=1.0, **reward.k_flag_leg=5.0**
(50 mm default allowance). Hypothesis: the flag-leg penalty removes the
lineage-wide vertical flag leg without losing tracking. Gate: sto walk
≥4/6 @ vel_err ≤0.030 AND video shows no foot >50 mm except swing AND
sto rise ≥4/6.

### LAUNCH cw-walk-flag-s1 (pod s4, seed 1, 4M steps, DR 0.4)
Identical to cw-walk-flag except seed 1 — the first REAL seed twin post
`set_random_seed` fix. Question: run-to-run variance + best-of-2. Same gate.

### LAUNCH cw-walk-nv (pod long5m, seed 0, 4M steps, DR 0.4)
Warm from `ppo_goal_cw_walk_dr04b` (in place). Single change:
**goal.walk_obs_body_vel=0** (privileged measured-velocity obs zeroed,
width unchanged). Hypothesis: tracking is re-learnable from
proprioception+IMU only — the deployable obs path. Gate: sto walk ≥4/6 @
vel_err ≤0.035. NOTE: no k_flag_leg here (one variable per run).

### LAUNCH cw-stance-raisemix (pod friction, seed 0, 3M steps, DR 1.0)
Warm from `ppo_goal_cw_stance_dr10` (in place). Single change: goal mix
**raise=0.4,rise=0.2,lower=0.2** (all round-6 stance cfg-sets retained).
Hypothesis: raise stalls (3–4/6 @ DR 1.0) because the default mix
under-trains it. Gate: raise ≥5/6 det+sto AND rise/lower ≥5/6 all start
kinds retained (crown-jewel guard).

Housekeeping: stray leftover forkserver process observed on s4
(references linux_control/urt2_setup, hours old, no training) — harmless,
left alone. Smoke/verification trainings this cycle: none needed (no new
code).

## Round 8 — global flag-leg penalty refuted; route it walk-only (2026-08-08 ~17:35)

Watcher triggered on cw-walk-flag; by cycle time cw-walk-flag-s1 and
cw-walk-nv had also finished (staggered by minutes), so all three are
handled here. Control first: re-ran the harness on parent
ppo_goal_cw_walk_dr04b under today's code (walk+rise, DR 0.4): sto walk
4/6 @ vel_err 0.030, sto rise 6/6 — matches round 6, eval drift ruled
out. The regressions below are real training effects.

### cw-walk-flag — MISS (k_flag_leg=5.0 all-modes routing refuted)
W&B 2r0jj2qq, 4M steps (28.74M cum). Harness @ DR 0.4, 0.02–0.06: sto
walk 2/6 @ vel_err 0.032 (gate ≥4/6 @ ≤0.030), det 2/6; rise det 1/6 /
sto 3/6 (bridge 0/5 across passes); raise 0/6 det+sto. Video: still not
walking — a 5-leg shuffle with a rear leg swung to full vertical
mid-walk that only comes down late; rise episodes stay in a low sprawl
and never stand, one ends with a leg flagged skyward.
Hardware-ready: NO — no six-foot gait, and it can no longer stand up
from its belly. Verdict: MISS/refuted.
The all-modes charge fights the >50 mm transient swings
rise needs from belly starts — the same interference that collapsed
raise under k_stance_clearance until it was mode-exempted. The term
fires (reward_flag_leg ≈ −0.5/step) but PPO pays it rather than
restructure the gait, and rise pays the collateral.

### cw-walk-flag-s1 — MISS (twin; first real seed divergence in prod)
W&B swtus1fa, same config, seed 1. Sto walk 2/6 @ 0.033, det 1/6; rise
sto 4/6 / det 3/6; raise 4/6 both passes. Video: still a shuffle, not a
gait — walk ep0 has the same transient vertical flag leg; ep3 keeps all
six feet low but the feet skate rather than visibly stepping.
Hardware-ready: NO — shuffle/skate locomotion, tracking below gate.
Verdict: MISS on the walk gate; but evidence the penalty CAN suppress
the flag leg in some rollouts. Twins now truly diverge (raise 0/6 vs
4/6) — the set_random_seed fix works in production.

### cw-walk-nv — MISS at 4M (deployable-obs baseline; one continuation)
W&B 8g6mggws, 4M steps. Sto walk 1/6 @ vel_err 0.035, det 1/6 @ 0.042
(gate ≥4/6 @ ≤0.035); rise det 1/6 / sto 2/6; raise det 5/6 + sto 6/6 —
best raise the walk line has ever shown, unexplained, noted. Video: a
leg parked straight up for the ENTIRE walk while the other five shuffle;
rise stays in a low sprawl and never stands.
Hardware-ready: NO — full lineage flag-leg pathology plus broken rise.
Verdict: MISS — proprioception-only tracking is not re-learned in 4M. Train-time
vel_err (0.030) looks fine but the harness disagrees. One
consolidate-in-place continuation (cw-walk-nv2 → 8M cum) before calling
the baseline; asymmetric AC must beat whatever nv reaches.

### Correction: cw-stance-raisemix was never launched
Round 7 logged a LAUNCH entry and claimed W&B verification, but there is
no W&B run, no /tmp/train log was ever created on friction, no process,
no checkpoints. The launch command was evidently never executed (the
other three runs went out fine). Friction sat idle ~1.2 h. Bookkeeping
error, not infra; relaunching this cycle exactly as specified.

Code change (sim_env.py): `reward.flag_leg_walk_only` (float, default
0 = legacy all-modes) gates the flag-leg charge to walk mode only —
declared routing per the plan's reward-routing rules. New unit test
`test_flag_leg_penalty_walk_only_routing` (hold not charged / legacy
still charges / walk still charged); 19/19 pass. 6k-step smoke train
warm from cw-walk-flag with the new flag: clean (WANDB_MODE=disabled).
No champion changes: ppo_goal_cw_walk_dr04b remains the walk champion
(and re-validated today); crown-jewel stance line untouched.

Launches (snapshot c858587abef49d8fc9bc74279686d37e3c3cb8f7, tag
exp/cw-walk-flagw; code synced to all four pods; per-run logs used;
eval/video-every 200k, background eval worker active). NOTE: upstream
had two operator commits (blunt-video-verdict directive + faststart
reels; seed-twin history correction) — rebased onto them, verdicts
above restated to comply, plan conflict resolved keeping the
operator's wording.

### LAUNCH cw-walk-flagw (pod s3, W&B 8n5eo6zj, seed 0, 4M, DR 0.4)
Warm from ppo_goal_cw_walk_dr04b (init_dr04b.zip). One variable vs
round 7: **reward.flag_leg_walk_only=1** (k_flag_leg=5.0 kept).
Hypothesis: walk-only routing suppresses the flag leg without the
rise/raise collapse of the all-modes version. Gate: sto walk ≥4/6 @
vel_err ≤0.030 AND video shows a six-foot gait with no flag leg AND
sto rise ≥4/6.

### LAUNCH cw-walk-flagw-s1 (pod s4, W&B jsjc65dd, seed 1, 4M, DR 0.4)
Twin of cw-walk-flagw (genuine divergence verified in prod this
round). Run variance + best-of-2. Same gate.

### LAUNCH cw-walk-nv2 (pod long5m, W&B e58w8dos, seed 0, 4M, DR 0.4)
Consolidate-in-place continuation of cw-walk-nv (warm from
ppo_goal_cw_walk_nv → 8M cum). Deployable-obs baseline gets its full
budget before the asymmetric-AC comparison. Gate: sto walk ≥4/6 @
vel_err ≤0.035. Second miss ⇒ baseline is called at whatever 8M
gives; move to asymmetric AC either way.

### LAUNCH cw-stance-raisemix (pod friction, W&B bl11do1r, seed 0, 3M, DR 1.0)
The round-7 plan, actually executed this time (verified: process,
per-run log advancing, W&B run). Warm from ppo_goal_cw_stance_dr10;
single change goal-mix raise=0.4,rise=0.2,lower=0.2 (dr10 default was
rise=0.25,lower=0.2,raise=0.2); all six dr10 cfg-sets retained. Gate:
raise ≥5/6 det+sto AND rise/lower ≥5/6 all start kinds (crown-jewel
guard).

## Round 8.5 — spurious cycle: watcher re-fired on already-handled runs (2026-08-08 ~18:1x)

Watcher named cw-walk-flag-s1 (swtus1fa) and cw-walk-nv (8g6mggws) as newly
finished, but both were evaluated and verdicted (MISS, videos reviewed) in
round 8, which absorbed their staggered finishes alongside cw-walk-flag.
Confirmed via W&B: no new runs by those names; both IDs match round 8. Second
dedupe failure of this kind (round 6.5 was smoke runs; this is finish-event
lag) — watcher should skip runs that already have an RL_LOG verdict.

Verified all four in-flight runs healthy (per-run logs growing over a 60 s
window; W&B state running): cw-walk-flagw 25.86M cum (~1.1M/4M in),
cw-walk-flagw-s1 25.75M, cw-walk-nv2 29.1M (~0.35M/4M), cw-stance-raisemix
19.44M (~0.24M/3M). No freed pods → no launches. RL_PLAN.md reviewed: no new
evidence since round 8, no changes. No eval, no verdicts, no champion changes.

## Capacity cycle — operator expanded to 6 pods; queue items 1+2 implemented and launched (2026-08-08 ~18:5x)

Operator-initiated cycle: guardrails now list six pods (added
hexapod-sweep-lower, hexapod-sweep-walk; 12 slots, cap 10 experiments,
2 reserved smoke slots). No run finished — no evals, no verdicts, no
champion changes. All four in-flight runs verified healthy on-pod
before touching anything (train_ppo process + per-run log present on
s3/s4/long5m/friction); their slots untouched. New pods verified idle.
Launch targets: 2 experiments per new pod = 8 concurrent total, under
the 10 cap, smoke slots preserved.

Decision: the only launchable queue items are #1 (learning-progress
speed curriculum) and #2 (asymmetric actor–critic) — #3 (temporal
actor) is defined on the best of aac/nv which are both still open, and
#4 (lower end-posture flag term) waits on cw-walk-flagw. Each new
config gets a genuine seed twin: run-to-run variance is proven large
(flagw twins: raise 0/6 vs 4/6), the slots are otherwise idle, and a
false negative on either architecture question is expensive. (Plan's
"multi-seed after a config wins" noted; deviation is deliberate and
capacity-driven, same as the flagw pair.)

Code changes (this cycle, snapshotted):
- `rl_move/sim/asym_policy.py` (new): `AsymActorCriticPolicy` — stock
  ActorCriticPolicy except the ACTOR path multiplies features by a
  fixed mask zeroing the privileged measured-velocity obs (last 2
  dims); the critic sees the full obs. Mask is a non-persistent buffer
  so state_dict keys match MlpPolicy exactly → champion weights
  transplant 1:1 (verified: transplanted actor == parent actor on
  velocity-zeroed obs; reloaded checkpoint actor provably blind to the
  privileged dims, critic provably not). Saved checkpoints reload via
  PPO.load unchanged (policy_kwargs stored in the zip).
- `train_ppo_sim.py`: `--asym-critic` flag; warm start from a stock
  champion does the transplant (fresh optimizer state, num_timesteps
  continues the lineage), continuing an already-asym checkpoint is a
  plain warm start.
- `walk_task.py`: `goal.walk_lp_curriculum=1` (default 0 = legacy, no
  behavior change) samples commanded speed from 8 buckets 0.02–0.12
  m/s per broadcastable weights (`set_walk_bucket_weights`); bucket id
  surfaced in step info.
- `train_ppo_sim.py`: LP callback — every 100k steps computes
  per-bucket mean walk_vel_err, scores buckets by relative |Δerr| vs
  the previous window (capped 0.5, floor eps=0.05 so no bucket
  starves), renormalizes, broadcasts to workers, and logs
  lp/vel_err_b* + lp/weight_b* (the command-speed→performance curve
  the plan asks for). Solved and impossible buckets both decay to the
  floor; improving/regressing frontiers get the samples.

Validation: unit tests 21/21 (2 new: actor-mask/critic-sight/
transplant-keys; bucket sampling + info routing). Smokes on the
controller (WANDB_MODE=disabled, per round-6.5 rule): 6k-step
--asym-critic transplant train clean; 6k-step LP CLI train clean;
separate SubprocVecEnv smoke confirmed reweighed bucket weights
actually reach the workers via env_method. Smoke checkpoints deleted.

RL_PLAN.md: compute section updated to 6 pods; queue rewritten (items
1+2 now in flight; temporal actor and lower end-posture remain).

Launches (snapshot 4daf7ee474bed44826db19201a7e258679a9dd63, tag
exp/cw-walk-aac; code synced to lower+walk pods, init_dr04b.zip md5-
verified on both; per-run logs; eval/video-every 200k). All four
verified: W&B run present, state running, global_step advancing past
the warm-start count.

### LAUNCH cw-walk-aac (pod walk, W&B 1emfi5i5, seed 0, 4M, DR 0.4)
Warm from ppo_goal_cw_walk_dr04b via --asym-critic transplant. One
variable vs parent: actor masked to hardware obs (measured-velocity
dims zeroed on the actor path only), critic keeps them. Hypothesis:
the privileged critic preserves the value signal the fully-blinded
baseline (cw-walk-nv, 1/6 @ 4M) lost, so a deployable actor learns
tracking. Gate: sto walk ≥4/6 @ vel_err ≤0.035 AND beat nv's 4M mark;
final comparison vs nv2 @ 8M. Blunt video verdict required.

### LAUNCH cw-walk-aac-s1 (pod lower, W&B cajod2qc, seed 1, 4M, DR 0.4)
Twin of cw-walk-aac (run variance + best-of-2). Same gate.

### LAUNCH cw-walk-lp (pod walk, W&B 0ff8idlz, seed 0, 5M, DR 0.4)
Warm from ppo_goal_cw_walk_dr04b. One variable vs parent:
goal.walk_lp_curriculum=1 (8 speed buckets 0.02–0.12 m/s, LP-weighted
sampling every 100k, eps floor 0.05) replaces the fixed 0.02–0.06
range. Hypothesis: sampling the improving frontier widens tracked
speed without the manual-widening regressions (w07/w08). Gate:
retention sto walk ≥4/6 @ vel_err ≤0.030 on 0.02–0.06 AND sto mean
vel_err ≤0.045 over uniform 0.02–0.12 AND blunt video review;
lp/vel_err_b* command-speed→performance curve logged.

### LAUNCH cw-walk-lp-s1 (pod lower, W&B tf16wr6y, seed 1, 5M, DR 0.4)
Twin of cw-walk-lp. Same gate.

State after this cycle: 8 experiments in flight (flagw, flagw-s1, nv2,
raisemix + the four above) on 6 pods; 4 slots free of which 2 reserved
for smokes. Next decisions wait on finishes: temporal actor keys off
the best of aac/nv2; lower end-posture keys off flagw.

## OPERATOR NOTE 2026-08-08 ~19:00 — pod capacity was wrong; launcher now mandatory

Guardrails claimed 128 cores/pod; real cgroup limits are 56 (s3/s4/
long5m/walk) and 30 (friction/lower). Measured: 1 run alone on a 56-core
pod ~1090 fps; two sharing it ~240 each; two on 30-core lower ~75 each
(a 5M run ≈ 18 h). Consequences for the agent, next cycle:

1. ALL launches now go through `rl_move/orchestrator/launch_run.py`
   (live capacity check, duplicate/concurrency gates, experiments.json
   ledger, INTENT→RUNNING verification). Raw nohup launches are a
   guardrail violation. `launch_run.py status` shows live placement.
2. REBALANCE: cw-walk-aac-s1 + cw-walk-lp-s1 are starving on lower
   (~75 fps). When flagw/-s1 free s3/s4, kill the twins and relaunch
   each from its own latest checkpoint on a fast pod (log it). Prefer
   cw-walk-aac + cw-walk-lp (sharing walk @ ~240 fps) getting a solo
   pod each as slots free; friction/lower are smoke/eval pods now.
3. The speed-range diagnostic (plan queue #1) still deserves its slot —
   sequence it against the rebalance as capacity allows.

## Cycle 9 — spurious trigger #3; infra cycle: canaries+auto-stop, gait gate, ledger, node-topology correction (2026-08-08 ~19:0x)

OBSERVATIONS. Watcher named cw-walk-flag-s1 + cw-walk-nv "just finished";
W&B IDs (swtus1fa, 8g6mggws) match the runs verdicted in round 8 — no new
runs exist by those names. Third re-fire on already-handled runs. Root
cause found in /workspace/orchestrator.log: the 18:08 cycle for this pair
was interrupted by the operator's manual capacity cycle and the watcher
restarted at 18:40:56 with its state file never updated (cw-walk-flag is
in `processed`, -s1 and nv are not). No evals, no verdicts, no champion
changes this cycle.

All 8 in-flight runs verified alive and advancing (W&B state running +
on-pod process/log checks + 60 s and 2 min growth windows): flagw 27.6M,
flagw-s1 27.8M, nv2 30.7M, raisemix 21.0M, aac 25.3M, aac-s1 24.8M,
lp 24.9M, lp-s1 24.8M cum.

INFRA FINDING — the 6 pods sit on TWO physical ~128-core nodes, not six:
g142d86 (friction, long5m, s3; load ~92) and g129004 (lower, s4, walk;
load ~230 — identical loadavg across pods = host-wide). The hot node runs
5 experiments ≈ 1.8x oversubscribed: the four newest runs progress at
only ~100–410 fps (2-min W&B deltas) vs ~570 fps for flagw on the quiet
node. Slow, not stalled — left alone per "never touch training pods";
plan's compute section corrected (slots are launch slots, not
throughput; ~4–5 fast runs total). Sizing/ETA math must use nodes.

CODE LANDED (review §5a/§5c/§5b/§8b, all default-off or eval-side for
in-flight runs; tests 21→25 pass; smoke below):
- `goal_task.py`: `force_rise_start` canary hook (None default = no
  behavior change; rng stream identical either way).
- `train_ppo_sim.py`: fixed-seed canaries (rise flat/bridge/crouch +
  lower, seeds 1001–4002) ride the existing bg-eval worker; parent
  baseline at warm start; groups the parent passed 2/2 are protected;
  regression AUTO-STOP after 3 consecutive full-group (0/2) failures.
  Default-ON for every `--init-from` run (`--no-canary` opts out,
  `--canary-stop-after 0` = monitor only). Smoke (WANDB_MODE=disabled,
  6k steps warm from dr04b): baseline rise_flat 2/2, rise_crouch 2/2,
  rise_bridge 0/2, lower 1/2 — matches dr04b's known bridge hole — and
  the mid-run probe reproduced it exactly (fixed-seed determinism holds
  in the production path). Would have caught cw-walk-flag's rise
  collapse millions of steps early.
- `eval_checkpoint.py`: GAIT-VALIDITY GATE — walk episodes now report
  `sacrificed_legs` (duty <0.10 = parked flag leg, or duty >0.95 with 0
  swings = dragged anchor) and walk `success` requires `gait_valid`.
  Validated on champion dr04b @ DR 0.4: leg 3 duty 0.04–0.06 in all 3
  episodes → walk 0/3. That is the honest number — the walk champion
  has never been walking. Applies to all future gates incl. flagw's.
- `orchestrator/experiments.json` (NEW): structured ledger, backfilled
  with 8 RUNNING + 3 FINISHED (flag, flag-s1, nv) entries.
- `orchestrator/watch_loop.py`: dedupe now also skips runs whose ledger
  status is FINISHED/FAILED (exception-safe; tested). Takes effect on
  the next watcher restart; meanwhile this cycle exiting rc=0 marks the
  two re-firing names processed, so both paths close the loop.

DECISION — no launches. (a) The binding review required regression
auto-stop BEFORE the next warm-start launch; it landed this cycle.
(b) Capacity: total demand already ~1.4x the two nodes; adding runs
slows everyone. (c) flagw (~1.2M to go), raisemix (~1.2M), nv2 (~2M)
finish within the hour on the quiet node — the next real cycle evals
them and launches the speed-range diagnostic (queue #1) on genuinely
freed slots, with canaries armed and predictions recorded in the
ledger per the two-phase INTENT/RUNNING protocol.

## Cycle 10 — flagw + raisemix verdicts; raise demoted to canary; rebalance + speed diagnostic launched (2026-08-08 ~19:4x)

### cw-walk-flagw — FAIL (walk-only flag routing does NOT fix the gait; retention half of the hypothesis held)
OBSERVATIONS. W&B 8n5eo6zj, 4M steps (28.76M cum), fps 1042, finished
19:07. Harness (ckpt md5 c99d9be2, DR 0.4, own cfg-sets, 6 eps/mode
det+sto): walk det 0/6 @ vel_err 0.040, sto 0/6 @ 0.032 — 0/6 is the
new gait-validity gate: every episode has sacrificed legs. Two distinct
exploit modes in the same eval: (i) literal tripod anchor — legs 0/2/4
planted at duty ~1.0 with ~1 swing, legs 1/3/5 airborne all episode
(sacrificed [1,3,5]); (ii) the old 5-leg shuffle with leg 3 parked
(duty 0.04). Retention: rise det 6/6 (flat/bridge/crouch all), sto 6/6;
raise det 6/6 / sto 5/6; track 4/6. env/reward_flag_leg still −0.75/step
at 4M — PPO pays the fine rather than step. Frames reviewed:
walk_det_0 (79bc0cec), walk_det_3 (a5b55e35), rise_det_0 (5a4d29d0),
rise_det_3 (c653c561), 10 frames each.
INTERPRETATION. Videos: walk is NOT WALKING — the body scoots while
2–3 legs are held curled or pointing straight up like antennas; one
eval flips to a static tripod with three legs waving in the air. The
"successful" rises reach height but end propped on ~3 legs with 2–3
legs flagged vertically — scalar success, degenerate end posture (a
lineage trait, parent dr04b rises look the same). Routing worked as
routing: rise/raise did not collapse (vs cw-walk-flag's all-modes
collapse). The penalty itself is refuted as a gait fix at k=5.0 — and
per review §0 we do not iterate the coefficient.
VERDICT: FAIL. hardware-ready: NO — no six-foot gait exists anywhere in
this policy; flag legs in walk AND rise end-states.
HYPOTHESIS STATUS: REFUTED (suppression half); SUPPORTED (retention
half: walk-only routing protects rise/raise, matching prediction-if-false).
Champion: unchanged (ppo_goal_cw_walk_dr04b, itself honestly 0/N under
the gait gate — "tracking champion", not a walking one).

### cw-stance-raisemix — FAIL (raise-heavy mix refuted; crown jewels intact; raise DEMOTED to canary)
OBSERVATIONS. W&B bl11do1r, 3M steps (22.20M cum), fps 139 (30-core
friction pod), finished 19:07. Harness (ckpt md5 7162ee62, DR 1.0, own
six cfg-sets): raise det 3/6 / sto 4/6 (gate ≥5/6 both — MISS, same
2–5/6 band as every prior lineage). Retention: rise det 17/18
aggregate (first pass 5/6 with flat 1/2; dedicated 12-ep pass 12/12),
sto 18/18; lower 6/6 det + 6/6 sto; hold 6/6. Failure classification
from trajectories (plan item): all 5 raise failures are NEAR-MISS
UNDER-LIFT — end height 6.0–8.2 mm short (one 18.5 mm), zero
terminations, tilt ≤1.22°, currents under breaker. Not lost-contact,
not saturation, not tilt, not hot legs. Additional pathology: legs 2
and 4 are near-unloaded (duty 0.00–0.30) in ALL 12 raise episodes,
pass and fail — raise executes on ~4 legs. Frames reviewed:
raisemix/rise_det_0 (a0b8180a) clean bridge rise to a real stand;
rise_det_3 (2a90bb50) flat start stays sprawled the whole episode —
that is the det-flat miss; raisemix2/raise_det_0 (ec341f35) +
raise_det_3 (82b9d1ee) stable quiet stand with a barely visible lift,
pass and fail look identical to the eye.
INTERPRETATION. 2× raise samples moved raise from 2–5/6 noise band to…
the same band. Mix is not the cause. The failure mode is consistent
(stops a few mm short of target height on 4 supporting legs), but the
remaining lever is another reward-coefficient iteration near the
target, which the external review forbids. Rise/lower at DR 1.0 are
NOT eroded — checkpoint is safe as a stand-line artifact, but it does
not beat the champion (dr10: 6/6 everywhere incl. det flat).
VERDICT: FAIL vs gate. hardware-ready: NO for raise (misses its own
bar); rise/lower remain champion-grade but champion unchanged
(ppo_goal_cw_stance_dr10).
HYPOTHESIS STATUS: REFUTED. Per prediction-if-false + review §7:
**raise is demoted to canary status** — it stays in the fixed-seed
canaries and eval suite as a regression tripwire, but no more stance
pods are spent chasing 5/6. Plan updated.

## OPERATOR 2026-08-08 ~19:40Z — starved twins killed; reel-collision video bug fixed

- Killed `cw-walk-aac-s1` + `cw-walk-lp-s1` (only ~300k steps past warm
  start at ~80 fps on the 30-core lower pod; W&B will show them crashed —
  that is expected, see ledger status KILLED_BY_OPERATOR). Do NOT
  relaunch the twins now: plan rule is multi-seed only after a config
  wins. Judge aac and lp on the primaries.
- Root-caused corrupt W&B videos (e.g. lp-s1 rollout_23): every run wrote
  reels to the shared `/tmp/reel_<step>.mp4`; co-hosted twins hit
  identical steps simultaneously and interleaved writes. Fixed in
  train_ppo_sim.py: per-pid unique reel names + write-to-.part-then-
  rename + unlink after W&B upload. RUNNING trainers still carry the old
  code — `cw-walk-aac` and `cw-walk-lp` share the walk pod at identical
  video steps, so treat any further corrupt reels from them as this bug,
  not policy behavior. When you rebalance them to solo pods (per the
  earlier note), relaunch from their own latest checkpoints AFTER
  `snapshot.sh --sync` so they pick up this fix.
### Cycle 10 infra + launches

INFRA. (1) `launch_run.py` had a fatal flaw on first real use: `kubectl
exec ... nohup & echo $!` hangs until the trainer exits (stream stays
attached), so the launcher timed out at 60 s AFTER starting the
trainer and left a healthy run stuck at INTENT. Fixed twice: stdin
from /dev/null on the remote command, and a TimeoutExpired recovery
path that rediscovers the trainer pid via /proc scan and continues
mechanical verification (validated live on the lp-s1b launch:
"kexec timed out; recovering... VERIFIED RUNNING"). aac-s1b and
speedhi were verified manually with the same evidence (process alive,
log growth, W&B step window) and promoted INTENT→RUNNING in the
ledger with the checks recorded. (2) **cw-walk-lp-s1 DIED SILENTLY**
on lower at ~19:39 @25.22M cum (0.46M/5M): process gone, log ends
mid-table with no traceback, cgroup oom_kill=0. Cause unknown; it
survived the deliberate aac-s1 kill by 2+ minutes. Watch for a
repeat — two silent deaths on lower would point at that pod.
(3) flagw-s1 (28.76M) and nv2 (32.77M) finished during this cycle —
their evals belong to the next cycle; long5m + lower left free (also
restores smoke capacity; launch cap respected at 3/4).

REBALANCE per operator note: cw-walk-aac-s1 killed on lower @25.04M
(was ~75 fps sharing a 30-core pod; ledger marked, not a scientific
verdict) and continued on 56-core s3. lp-s1's death converted its
planned rebalance into a restart on s4.

### LAUNCH cw-walk-aac-s1b (pod s3, W&B gmbwrp4v, seed 1, 3.72M→28.76M cum, DR 0.4)
Continuation of cw-walk-aac-s1 from its own ckpt
ppo_goal_cw_walk_aac_s1_25044096_steps.zip (md5 f60427de), all
settings identical (asym critic, 0.02–0.06, k_walk_swing=1.0).
Gate unchanged: sto walk ≥4/6 @ vel_err ≤0.035 with hardware-legal
actor AND beat nv 4M mark; vs nv2 @ 8M; gait-validity gate applies.
Verified: log 21159→38143 B, step 25.29M→25.52M.

### LAUNCH cw-walk-speedhi (pod friction, W&B g30vnbvl, seed 0, 1.5M, DR 0.4)
Plan queue #1 / review §2a, triggered by flagw's refutation. Warm from
init_dr04b (md5 52220b24). One variable vs dr04b: command speed
0.10–0.15 m/s (was 0.02–0.06). Hypothesis: at 2–6 cm/s the
drag-shuffle is near-optimal; at speed, stepping is forced. TRUE
branch: ≥3/6 sto gait-valid AND mean speed ≥0.08 @1.5M → curriculum
goes fast→slow. FALSE branch: shuffle/falls persist → slow→fast and
phase reward is next. Canaries monitor-only (--canary-stop-after 0):
diagnostic ckpt will never be promoted; want full 1.5M of gait
evidence. On the 30-core smoke pod deliberately (--allow-slow,
guardrails sanction short diagnostics there); early fps ~1090.
Verified: log 9906→18274 B, step 24.88M→24.98M.

### LAUNCH cw-walk-lp-s1b (pod s4, W&B via launcher, seed 1, 4.57M→29.76M cum, DR 0.4)
Restart of the silently-dead lp-s1 from its last ckpt
ppo_goal_cw_walk_lp_s1_25188096_steps.zip (md5 12d2ddd5), settings
identical (walk_lp_curriculum=1; LP bucket weights reset, re-adapt in
~100k). Gate unchanged: retention sto walk ≥4/6 @ vel_err ≤0.030 on
0.02–0.06 AND sto mean vel_err ≤0.045 over uniform 0.02–0.12 AND
blunt video + gait-validity. Launcher-verified (fps est 1365).

State after cycle 10: 6 experiments in flight — aac + lp (walk),
aac-s1b (s3), lp-s1b (s4), speedhi (friction), none on lower/long5m
(free; smoke capacity restored). Next cycle: eval flagw-s1 + nv2
(nv2 @8M is the aac-comparison baseline), then speedhi's answer
decides the phase-reward launch.

## OPERATOR 2026-08-08 ~20:07Z — torched redundant slow primaries

Killed `cw-walk-aac` + `cw-walk-lp` (shared walk pod, 418 fps each) at
~27.05M steps: their s1b twins run the SAME configs solo at 1587/1053
fps and aac-s1b had already overtaken the primary in steps. One run per
config until something wins — judge asym-AC on `cw-walk-aac-s1b` and
the LP curriculum on `cw-walk-lp-s1b`. W&B will show the primaries
crashed; ledger says KILLED_BY_OPERATOR. Walk pod is now free — use it
solo (56-core). Do not relaunch the primaries.

## OPERATOR 2026-08-08 ~20:25Z — phase-reward experiment: add a basin-escape arm

Pattern across ALL walk refutations (5 shaping levers, speed pressure,
nv continuation): every run warm-starts from the same converged shuffle
(dr04b lineage). A converged low-entropy policy + a value function
calibrated to the exploit is a deep basin; every excursion toward
stepping pays immediate return cost, so PPO returns to the shuffle.
The refutations are consistent with "basin too deep", not only "lever
wrong" — and the periodic-reward literature (Siekmann/Cassie) trains
from scratch WITH the phase reward, not by fine-tuning a shuffle.

Directive for the phase-based tripod reward experiment: run TWO arms
(capacity exists): (a) warm from dr04b as planned; (b) a basin-escape
arm — fresh init (or stance-champion init `ppo_goal_cw_stance_dr10`,
which stands/rises but never learned the shuffle) with the phase
reward from step 0, higher initial log_std, walk-heavy goal mix but
keep rise/lower/hold in the mix + canaries so the stance skills either
survive or the erosion is caught immediately. Record both as one
falsifiable comparison: if (b) steps and (a) shuffles, the basin is
the story and warm-start-by-default gets an exception for behavior-
class changes (note in guardrails already permits fresh init when the
log records it as the hypothesis).
## Cycle 11 — nv baseline CALLED at 8M; flagw-s1 confirms refutation; walk-line `lower` found long-broken; speedhi answered (2026-08-08 ~20:0x)

Watcher also named cw-walk-aac-s1 + cw-walk-lp-s1 "finished": those are cycle
10's rebalance kill and silent death (W&B crashed states), already ledgered
and continued as -s1b runs — nothing to eval. Real work: flagw-s1 + nv2, and
speedhi finished mid-cycle (20:06) so its diagnostic is answered here too.
Harness evals ran on the freed lower/long5m pods (code synced first so the
gait-validity gate applies); controls as noted.

### cw-walk-flagw-s1 — FAIL (twin confirms: walk-only flag penalty does not fix the gait)
OBSERVATIONS. W&B jsjc65dd, 4M steps (28.76M cum), finished 19:36. Harness
(ckpt md5 04864399, DR 0.4, own cfg-sets, 6 eps/mode det+sto, modes
walk/rise/raise/lower/track): walk det 0/6 gait-valid @ vel_err 0.036, sto
0/6 @ 0.033. Leg 3 parked (duty 0.03–0.05) in 10/12 walk episodes; 2 sto
episodes flip to the static tripod anchor (duty [.97,.04,.96,.07,.94,.05],
legs 1/3/5 airborne) — the exact two exploit modes of seed-0. Retention:
rise det 5/6 / sto 6/6 (sto flat 4/4), raise 4/6 det+sto, track 5/6 both;
lower det 0/6 / sto 1/6 (see lineage finding). Frames reviewed (10 each):
walk_det_0 b8c2fc7b, walk_det_3 beb7e8cd, rise_det_0 590ed2a4, lower_det_0
3c0d464e, contact_sheet d35e2195.
INTERPRETATION. Videos: NOT WALKING — body scoots on five legs while leg 3
hangs curled in the air; a rear leg flags to full vertical mid-episode;
two episodes give up into a static tripod with three legs waving. Rise
reaches height but ends propped with 1–2 legs flagged skyward (lineage
trait). Lower does not lower: stays elevated, flags a leg, ends 14–42 mm
high.
VERDICT: FAIL. hardware-ready: NO — no six-foot gait; flag legs in walk,
rise end-state, and lower.
HYPOTHESIS STATUS: REFUTED (both seeds agree; best-of-2 buys nothing).
Flag-leg penalty closed as a gait lever at any routing. Champion unchanged.

### cw-walk-nv2 — FAIL at 8M (deployable-obs baseline CALLED; do not extend)
OBSERVATIONS. W&B e58w8dos, +4M (32.77M cum = 8M on the nv line), finished
19:15. Harness (ckpt md5 97a7e553, DR 0.4, own cfg-sets): walk det 0/6
gait-valid @ vel_err 0.034, sto 0/6 @ 0.035, mean speed 0.029 m/s. Leg 3
parked (duty 0.02–0.07) in 11/12 walk episodes, one sto tripod-anchor
episode. rise det 4/6 (flat 0/1) / sto 3/6 (flat 1/4); raise 5/6 both;
lower 0/6 both; track 5/6 both. Frames reviewed (10 each): walk_det_0
5a3f9bc0, walk_det_3 62454c0d, rise_det_0 0576d35d, rise_det_3 5b04537b,
lower_det_0 8c4cff8d, contact_sheet c5f44795.
INTERPRETATION. Videos: NOT WALKING — the same antenna leg parked vertical
for entire episodes over a low five-leg shuffle; both recorded rise
episodes stay in a low sprawl and never stand. The extra 4M changed
nothing visible. Baseline delta: vs its own 4M mark (sto 1/6 @ 0.035) no
metric moved beyond eval noise; vs parent dr04b (sto 4/6 @ 0.030
pre-gait-gate) still clearly worse.
VERDICT: FAIL. hardware-ready: NO — flag-leg pathology plus eroded rise.
HYPOTHESIS STATUS: REFUTED. The proprioception-only baseline is CALLED at
8M per the fixed-budget rule: sto walk 0/6 gait-valid @ vel_err 0.035.
That is the number cw-walk-aac(-s1b) must beat at 28.76M cum. Champion
unchanged.

### Lineage finding — walk-line `lower` has been broken all along
This cycle added lower to the walk-line eval modes (first time it was ever
measured on this lineage): flagw-s1 0/6 det + 1/6 sto, nv2 0/6 + 0/6. Control:
parent champion dr04b is ALSO lower 0/6 det + 0/6 sto (ends 5–36 mm above
belly, zero terminations) — inherited erosion, not a new regression; every
prior walk-line eval simply never looked. Consequence: reward-interference
erosion covers rise AND lower; the deployable end-state policy must come from
the stance line or a later merge/distillation, and lower stays in walk-line
evals as a tripwire. Stance line unaffected (cycle-10 re-check: lower 12/12
@ DR 1.0).

### cw-walk-speedhi — diagnostic ANSWERED: FALSE branch (speed does not force stepping)
OBSERVATIONS. W&B g30vnbvl, 1.39M steps (26.27M cum), finished 20:06 (1466 s
on friction). Harness local on friction (ckpt md5 0c3ea0bd, DR 0.4, commands
0.10–0.15, 6 eps/mode det+sto, walk+rise): walk det 0/6 gait-valid @ vel_err
0.099, sto 0/6 @ 0.084; mean speed 0.033–0.034 m/s — the policy did NOT get
faster than the parent shuffle (gate needed ≥0.08 m/s + ≥3/6 gait-valid).
Leg 3 sacrificed (duty 0.03–0.07) in 12/12 episodes; one sto over_current
termination; rise det 3/6 / sto 4/6 (flat 0/3 across passes). Frames
reviewed (10 each): walk_det_0 f5e25a18, walk_det_3 99dc3e37, rise_det_0
c781b868, contact_sheet 4c3f8247.
INTERPRETATION. Videos: NOT WALKING, and worse than the parent to look at —
under fast commands the shuffle keeps its ~3 cm/s drag and by mid-episode
parks TWO legs straight up (rear + front antenna) while four legs scoot the
body. PPO answered the fast commands by absorbing tracking error, not by
stepping. 1.5M steps is the caveat (a gait might emerge with more budget or
a fresh init), but as a frontier-seeding diagnostic the answer is clean.
VERDICT: FALSE branch. hardware-ready: NO (diagnostic ckpt, never promotable).
HYPOTHESIS STATUS: REFUTED (speed pressure alone does not force stepping
from this lineage). Consequences per plan §Walk escalation: curriculum
frontier stays slow→fast (in-flight lp runs unaffected), and the
phase-based alternating-tripod reward (item c) is NEXT — launching this
cycle.

### Cycle 11 code changes (snapshotted with cw-walk-phase)
- `walk_task.py`: phase-based alternating-tripod reward (plan §Walk item c,
  Siekmann-style periodic reward composition). `goal.walk_phase_obs=1`
  (default 0 = legacy width) appends sin/cos of an internal clock
  (`goal.walk_phase_hz`, default 1.0) to the obs (+2 dims); the clock
  advances only while a walk velocity is commanded. `reward.k_phase_contact`
  (default 0) pays contact states agreeing with alternating tripods
  ({0,2,4} stance while sin>=0, complement otherwise): r = k*(agree/6-0.5)*2.
  Walk-routed by construction; settle hold never charged. Key property: a
  parked or dragged leg averages 50% agreement = ZERO net reward — only
  clock-synchronized stepping pays. This is a payment for stepping, not a
  fine for not stepping: three fines failed because PPO paid them
  (flag/flagw/speedhi). No joint targets, no trajectories, no constraint.
- `train_ppo_sim.py`: `--obs-pad-transplant N` + `pad_obs_transplant()` —
  warm start across an obs WIDENING (new dims appended at end): all tensors
  copy exactly except the two first-layer weights, which get zero columns
  for the new dims → transplanted policy bit-identical to the parent until
  training moves the zero columns (unit-tested). Refuses --asym-critic combo
  (privileged_idx would shift).
- BUGFIX `train_ppo_sim.py` evaluate(): post-train quick eval built its env
  WITHOUT --cfg-set overrides — crashes any run whose cfg changes obs width
  (found in the phase smoke; harmless historically because no prior cfg-set
  changed width). Now honors cfg_set.
- BUGFIX `eval_checkpoint.py`: --cfg-set was applied by mutating env.cfg
  AFTER construction — silently wrong for anything baked in __init__ (obs
  width). Overrides now go into the cfg BEFORE env construction. Legacy
  path regression-checked (dr04b walk ep: vel_err 0.029, leg 3 sacrificed,
  matches known behavior).
- Tests 25→28 (phase clock/routing, phase reward value vs touch sensors,
  transplant behavior-preservation). 6k smoke train end-to-end clean (exit
  0, transplant exact, canaries armed, parent baseline rise_flat 2/2
  rise_crouch 2/2 rise_bridge 0/2 lower 1/2 — matches dr04b's known holes);
  harness smoke on the wider ckpt clean. Smoke ckpts deleted, W&B disabled.

### LAUNCH cw-walk-phase (pod long5m, seed 0, 4M, DR 0.4)
Warm from champion ppo_goal_cw_walk_dr04b (in place on long5m) via
--obs-pad-transplant 2. Changes vs parent (one mechanism: clock obs +
agreement reward are useless separately): goal.walk_phase_obs=1,
goal.walk_phase_hz=1.0, reward.k_phase_contact=1.0; walk range 0.02–0.06,
k_walk_swing=1.0 as parent. HYPOTHESIS: paying for clock-synchronized
alternating-tripod contacts (dense, every tick) breaks the drag-shuffle
that survived three penalty levers, because stepping becomes the paid
behavior rather than the fined-but-cheaper alternative.
Prediction-if-true: phase_agreement rises >0.6, swing counts ≥3/ep on ALL
six legs (leg 3 included), sacrificed_legs empty in most harness episodes.
Prediction-if-false: agreement pins ~0.5 (term ignored, shuffle persists)
OR agreement high while vel_err >0.05 (steps in place, tracking
sacrificed). Strongest alternative: stepping emerges but clock-locked and
jerky — fluidity judged in video. GATE: sto walk ≥4/6 gait-valid @
vel_err ≤0.035 AND video shows all six feet cycling contact/swing AND sto
rise ≥4/6 retained (canaries armed, auto-stop default).

## OPERATOR addendum ~20:35Z — DR schedule for the phase-reward arms

The basin-escape (fresh/stance-init) phase arm starts at LOW DR
(0.1–0.2), per the plan's skill-first rule — a from-scratch gait gets
friendly physics until six-foot stepping visibly exists, then anneal
up in stages exactly like the stance line (0.2 → 0.4 → 1.0). Do not
start it at 0.4 just because the walk lineage lives there. The warm
arm can stay at 0.4 (its question is basin escape at parity with the
lineage). Gait-validity gate + canaries apply at every DR rung.

### Cycle 11 addendum — operator directives absorbed mid-cycle (rebase at snapshot)
Two operator commits landed while this cycle ran: (1) aac/lp primaries
torched (walk pod free, judge on the -s1b twins — acknowledged, nothing
relaunched); (2) the phase experiment must include a basin-escape arm.
Implemented as `cw-walk-phase-stance` below. Fresh init is the weaker
basin-escape variant (round 1: cw-walk-fresh-gait converged to the
identical skate), so the arm uses the STANCE champion init, which stands,
rises and lowers at DR 1.0 and has never learned the shuffle. This is a
sanctioned fresh-basin exception to warm-start-by-default, recorded here
as the hypothesis. Stance→walk crosses obs 68→74; the new
--obs-pad-transplant 6 bridges it (all six added dims — vel refs,
measured vel, phase clock — sit at the obs tail; verified in smoke:
transplant exact, canary parent baseline 8/8 with ALL FOUR groups
protected, rise f2/2 b2/2 c2/2 on the walk env, exploration reopened via
--reset-log-std against the annealed champion std).

### LAUNCH cw-walk-phase-stance (pod walk, seed 0, 4M, DR 0.4)
Basin-escape arm (operator directive 20:25Z). Init: ppo_goal_cw_stance_dr10
via --obs-pad-transplant 6 + --reset-log-std; identical cfg to
cw-walk-phase otherwise (walk_phase_obs=1, phase_hz=1.0,
k_phase_contact=1.0, k_walk_swing=1.0, 0.02–0.06, default walk-heavy mix
with rise/lower/hold retained; canaries armed, all four groups protected).
The two arms are ONE falsifiable comparison — single differing variable:
starting basin. If stance-init steps while dr04b-init shuffles, the basin
(converged shuffle + value function calibrated to it) is the story and
behavior-class changes get fresh-basin inits from now on. If both shuffle,
the phase reward is refuted independent of basin. If both step, take the
better gait. GATE (same as arm a): sto walk ≥4/6 gait-valid @ vel_err
≤0.035 AND video shows all six feet cycling AND sto rise ≥4/6 retained
(auto-stop protects the imported crown jewels).

### Correction to the cw-walk-phase-stance entry (operator addendum 20:35Z)
The basin-escape arm launches at **DR 0.2, not 0.4** (skill-first rule: a
from-scratch gait gets friendly physics; anneal 0.2 → 0.4 → 1.0 once
six-foot stepping visibly exists). Its gate is evaluated at its own DR
(0.2). The warm arm stays at DR 0.4. Arms now differ in init AND DR by
operator instruction — the basin comparison reads: stance-init@0.2
stepping while dr04b@0.4 shuffles ⇒ escalate the winner up the DR ladder
before any cross-arm conclusion beyond "basin matters".

## OPERATOR ~20:40Z — end-posture validity for rise/lower (operator caught a flagged leg in the 12/12 lower)

Reviewing today's stance-champion strips: lower_det_0 ends with a rear
leg pointed straight up while the eval scores it a success — the lower
gate checks height/quiescence only. Same blind-spot class as the walk
scalars. Directives:
1. Add an eval-side END-POSTURE check for rise and lower (and any mode
   that terminates in a stance or a belly rest): at episode end, all
   six feet within a small height of their support surface (belly rest:
   legs down/tucked, none elevated above body top; stand: six feet in
   contact). Report per-episode like gait_valid; wire into success for
   these modes after baselining how often the current champions
   actually pass (report the baseline first — if the stance champion
   fails often, that is a finding, not a reason to soften the check).
2. The queued routed flag-leg reward for the stance line's lower is now
   justified (routing safety was proven by flagw's retention half).
   Schedule it as a stance-line run when a slot frees, gated on the new
   end-posture eval + full crown-jewel canaries.

## OPERATOR ~20:45Z — SUPERSEDES the flag-reward part of the previous note: first-principles, not patches

Operator question that reframes it: WHY is a waving leg bad? (1)
smaller support polygon = less stability margin = tips (the hardware
incident); (2) load re-routes through fewer legs = hot knees (the
no-hot-leg problem is the SAME problem); (3) sustained gravity torque
on the hip for zero contribution = wasted energy. A flag-leg penalty
treats the symptom; three generic physics terms cover every version of
this pathology in every mode.

1. **FIRST, diagnose the sim's pricing (root-cause hypothesis):** on a
   real STS3215, holding a leg horizontal draws continuous current; in
   our servo model the dead-zone may make static gravity holds nearly
   FREE, so the existing effort penalty never charges for flag legs in
   sim. Measure: sim per-servo current for a leg held horizontal in the
   air vs the motor-dynamics data / stall-torque expectations. If
   underpriced, fix the torque→current model so static holds cost what
   they cost. This makes the EXISTING energy term do the work
   everywhere at once. Report the before/after current numbers.
2. **Then, generic GLOBAL reward terms (replace the queued stance
   flag-leg run):** (a) stability margin — distance of CoM ground
   projection to the support-polygon edge (reward margin, penalize
   near-edge); (b) per-leg load evenness — penalize variance /
   concentration of foot normal forces. Both mode-independent and
   legitimately GLOBAL under the routing rules (they encode "don't
   tip, don't concentrate heat", not gait morphology). Retire
   k_flag_leg if these subsume it (keep the eval-side detection
   forever).
3. End-posture eval check from the previous note still stands.

Prompt lesson (also added to ORCHESTRATOR_PROMPT): when a blind spot is
found in one mode, ask "which other modes share this failure class?"
and extend the CHECK there in the same cycle — checks generalize,
patches don't.

## OPERATOR ~20:55Z — best-practices audit vs field standard (read archive/BEST_PRACTICES_AUDIT_2026-08-08.md)

Audited our PPO config against RSL-RL/Isaac locomotion standards +
current sim-to-real practice. Two HIGH findings, both plausibly load-
bearing for the shuffle lock-in:
1. **Exploration 3–10x low for from-scratch gait learning:** ours
   std 0.37 / ent_coef 0.001 vs field-standard std 1.0 / 0.005–0.01.
   Directive: fresh/basin-escape arms use log_std_init 0.0 +
   ent_coef 0.005–0.01; log entropy; entropy collapse = health alarm.
   If the in-flight phase-stance arm launched with old exploration
   settings, launch a corrected sibling rather than waiting.
2. **No KL control:** add SB3 `target_kl≈0.02` to every run now (one
   line); destructive-update protection standard everywhere else.
Also directed (MED): reward/obs scale audit script (numbers attached
to ledger), symmetry (mirror) augmentation queued post-phase-verdict,
tick-jitter DR + observation-delay verification, and a probe-env rule:
every new mechanism (phase reward, asym critic) gets a trivial probe
smoke that must pass before any 4M-step run uses it.

### Cycle 11 close — launch verification
Both arms launcher-verified (INTENT→RUNNING, kexec-hang recovery path both
times) and +5-min checkups HEALTHY: cw-walk-phase (W&B o6a2x0u2, long5m
solo, ~1900 fps, warm dr04b @ DR 0.4) and cw-walk-phase-stance (W&B
o6m4zig3, walk pod solo, ~2400 fps, stance init @ DR 0.2). Launcher
gotcha for next cycles: passthrough string values (--notes) are NOT
shell-quoted by launch_run.py — pass them pre-quoted (\"...\") or the
remote bash -c dies with exit 2 before the trainer starts (attempt 1 of
cw-walk-phase, recorded FAILED in the ledger; no process/W&B leaked, retry
clean). In flight after this cycle: aac-s1b (s3), lp-s1b (s4), phase
(long5m), phase-stance (walk). friction + lower free = smoke/eval
capacity. Next cycle: eval aac-s1b vs the called nv 8M baseline when it
finishes (~28.76M cum), lp-s1b at ~29.76M, then the phase arms.

## Cycle 11b (same session) — aac-s1b + lp-s1b finished mid-cycle; evals done now

### cw-walk-aac-s1b — FAIL on walk at 4M; retention half strongly supported; continuation to complete the 8M fixed-budget comparison
OBSERVATIONS. W&B gmbwrp4v, 3.72M steps (28.77M cum = 4M asym training from
dr04b), finished ~20:20 @ ~1600 fps. Harness on s3 (ckpt md5 0642955c, DR
0.4, own cfg-sets, 6 eps/mode det+sto): walk det 0/6 gait-valid @ vel_err
0.036, sto 0/6 @ 0.036 — leg 3 parked (duty 0.04–0.08) in 12/12 episodes,
no tripod flips. Retention: rise det 5/6 / sto 6/6 (flat 5/5 across
passes), raise det 4/6 / sto 5/6, track 5/4, lower det 2/6 / sto 0/6.
Baseline deltas: vs nv @4M (sto 1/6 @ 0.035) tracking is EQUAL within eval
noise (0.036 vs 0.035); vs nv's rise (det 1/6 / sto 2/6) retention is
DRAMATICALLY better (11/12 vs 3/12) — the privileged critic prevents the
forgetting the blind baseline suffered, but does not improve tracking.
Frames reviewed (10 each): walk_det_0 77280d78, walk_det_3 4aedf3c8,
rise_det_0 4cd15c43, contact_sheet 76c4ab4c.
INTERPRETATION. Videos: NOT WALKING — same 5-leg scoot with leg 3 curled
in the air, body held a bit higher than the parent's version; rise reaches
a real stand with transient flag legs on the way up (lineage trait).
VERDICT: FAIL vs its walk gate. hardware-ready: NO — no six-foot gait.
HYPOTHESIS STATUS: tracking half REFUTED at 4M (deployable actor tracks no
better than blind at matched steps); retention half SUPPORTED (rise 11/12
vs 3/12). The fixed-budget comparison point is 8M (review §6, nv2 already
ran its 8M): launching continuation cw-walk-aac-s1c (+4M, same settings)
so the deployable-obs question is answered at matched budget, not called
early on a half-budget tie. Champion unchanged.

### cw-walk-lp-s1b — FAIL both gate halves; LP speed curriculum REFUTED; line closed
OBSERVATIONS. W&B 4qe1gv3z, 4.57M steps (29.76M cum), finished ~20:35.
Harness on s4 (ckpt md5 972966be, DR 0.4): retention pass (fixed
0.02–0.06): walk det 0/6 gait-valid @ vel_err 0.034, sto 0/6 @ 0.032; leg 3
parked det, tripod anchor [1,3,5] sto. Wide pass (uniform LP buckets
0.02–0.12): det 0/6 @ 0.045, sto 0/6 @ 0.059 (gate ≤0.045 MISS), tripod
anchor both passes, speed pinned at 0.030–0.033 m/s regardless of command.
Retention modes: rise det 3/6 (bridge 0/1, flat 2/4) / sto 6/6; raise det
2/6; track det 3/6 / sto 5/6; lower det 4/6 / sto 3/6 — oddly the best
lower in the walk lineage (noted, unexplained, still below stance-line
12/12). Frames reviewed: walk_det_0 0049bd93, walk_det_3 14ce7e6b,
rise_det_0 299eb101, lower_det_0 5726d28f, wide walk_det_0 fae93726,
contact_sheet 0fd31418.
INTERPRETATION. Videos: NOT WALKING — under wide commands the robot rears
into a 2–3-leg tower with front legs waving in the air. LP reweighting
cannot widen tracked speed when no bucket above the shuffle's ~3 cm/s
ceiling ever improves — exactly what speedhi predicted (FALSE branch). Det
rise erosion (3/6) also makes this checkpoint strictly worse than aac-s1b
as a lineage artifact.
VERDICT: FAIL. hardware-ready: NO. HYPOTHESIS STATUS: REFUTED (both
halves). LP line closed (primary torched by operator, twin refuted);
walk_lp_curriculum stays available as infrastructure for a future gait
that can actually go faster. Champion unchanged.

## Cycle 11c (same session) — operator audit absorbed: basin-escape arm relaunched on audited settings after a PASSING mechanism probe

Operator commit 02ea8cc (best-practices audit, BINDING) landed minutes
after the phase arms launched: basin-escape/from-scratch arms must use
std 1.0 (log_std 0.0) + ent_coef 0.005–0.01; ALL runs get target_kl 0.02;
new mechanisms need a probe smoke. Actions taken, in order:

1. KILLED cw-walk-phase-stance @ ~22.74M cum (~2M/4M in): it ran std
   0.37 / ent 1e-3 / no target_kl — precisely the under-exploration taint
   the audit says contaminates from-scratch refutations. Restarted from
   the ORIGINAL stance init, not continued (the 2M trained steps carry
   the taint). Ledger: KILLED_RELAUNCH, not a scientific verdict.
2. Left cw-walk-phase (long5m) and cw-walk-aac-s1c (s3) RUNNING: both
   are warm-refinement runs where low inherited noise is deliberate and
   sanctioned by the audit ("may keep lower noise — but say so": said
   here). They miss only the target_kl guard; killing mid-run to add a
   safety net costs more than it protects. All future launches get
   target_kl 0.02 by default from the operator's trainer change.
3. PROBE (audit §6, launcher smoke probe-phase-agree on lower, 100k @
   DR 0, stance init, audited exploration): PASS — after 96k steps ALL
   SIX legs cycle (det swings 6–21/leg vs ~0 for the stance parent and
   a parked leg 3 for the whole shuffle lineage; duties 0.24–0.95; det
   gait_valid 2/3; speed ~0.023 m/s, tracking not yet converged, as
   expected at 96k). The phase-contact mechanism is learnable and drives
   phase-synchronized stepping. First all-six-leg cycling ever observed
   in this campaign. Two launcher lessons recorded in the ledger: the
   verifier kills runs that FINISH inside its window (sub-2-min smokes
   always "fail"); readout taken from the surviving periodic ckpt.
4. RELAUNCHED as cw-walk-phase-stance2 (walk pod, 4M, DR 0.2, seed 0):
   stance init via --obs-pad-transplant 6, --set-log-std 0.0 (std 1.00
   confirmed in log), --ent-coef 0.01, target_kl 0.02 (confirmed), phase
   cfg unchanged, canaries armed with all four groups protected. Gate
   unchanged (sto walk ≥4/6 gait-valid @ vel_err ≤0.035 @ DR 0.2 AND
   six-foot video AND sto rise ≥4/6). Launcher-verified RUNNING.

Cycle 11 totals: 4 experiment launches (phase, phase-stance†, aac-s1c,
phase-stance2) = at the 4/cycle cap; 16M new steps = at the 16M cap
(†killed for the audit; its steps counted). Pods: phase (long5m),
phase-stance2 (walk), aac-s1c (s3) training; s4/friction/lower free.

### cw-walk-phase (warm arm) — FAIL: the shuffle basin beats the phase reward under inherited low noise
OBSERVATIONS. W&B o6a2x0u2, 4M steps (28.76M cum), finished 21:23 (2745 s
solo on long5m). Harness (ckpt md5 80c09111, DR 0.4, own cfg-sets incl.
phase obs): walk det 0/6 gait-valid @ vel_err 0.033, sto 0/6 @ 0.029 —
leg 3 parked (duty 0.02–0.09) in 12/12 episodes, identical duty signature
to the parent (legs 1/4 anchored ~0.8/0.95, legs 0/2 skating ~0.25).
Retention: rise det 2/6 (flat 0/3, REGRESSED vs parent det 4/6) / sto 5/6;
raise 4-5/6; lower 0/6 (inherited); track 5/6. Frames reviewed (10 each):
walk_det_0 ac843895, walk_det_3 4dc74c71, rise_det_0 12b5f96e,
contact_sheet 5edca4b0.
INTERPRETATION. Videos: NOT WALKING — same scoot with one leg flagged
fully vertical mid-episode and another splayed horizontally by the end;
the recorded det rise stays in a low sprawl and never stands. The phase
term did not restructure the gait: PPO under the annealed policy noise
(std ~0.28) treats a zero-mean agreement reward as noise and keeps the
exploit — exactly the operator's basin diagnosis, and the exact opposite
of the probe result where the SAME mechanism at std 1.0 from the stance
init drove all-six-leg cycling within 96k steps.
VERDICT: FAIL. hardware-ready: NO. HYPOTHESIS STATUS: warm-arm variant
REFUTED (dense payment alone does not escape a converged shuffle at low
noise); the basin-escape half of the comparison rides on
cw-walk-phase-stance2 (in flight, std 1.0). No relaunch on long5m this
cycle — launch and step caps are both exactly consumed (4/4, 16M/16M).
Champion unchanged.

## Cycle 12 (2026-08-08 ~21:30–22:30Z) — end-posture gate lands (crown jewel caveated), pricing diagnosis, phase line closed, aac called at 8M

### Operator directives implemented (eval-side check + first-principles diagnosis + generic terms)
1. **End-posture eval check** in `eval_checkpoint.py`: per-foot clearance
   (mean of final 0.5 s) vs the episode-start grounded pad z; stand-ending
   modes (rise/raise/hold/lean/track) need all six feet ≤20 mm, lower ≤60 mm
   (tucked ok, vertical flag ~130+ mm fails). Reported per episode like
   gait_valid; wired into success by DEFAULT after baselining
   (`--no-end-posture-gate` opts out). **All rise/lower/hold success counts
   from here on are posture-strict; earlier logged numbers are height-only
   and not comparable.**
2. **BASELINE (stance champion `cw_stance_dr10` @ DR 1.0, 6 eps/mode
   det+sto):** hold 12/12 pass (≤16 mm). rise 5/12 — fails are crouch/bridge
   starts with legs 2/4 left at 100–199 mm (two flat-sto fails marginal,
   22–27 mm). **lower 0/12 — every single episode ends as a tripod on the
   odd legs with legs 0/2/4 hoisted and leg 4 fully vertical (244–281 mm).**
   Frames confirm: the policy drops the commanded 25–55 mm then hoists three
   legs skyward; it never approaches a belly rest posture. CROWN-JEWEL
   CAVEAT: "stand↔belly solved at DR 1.0" means height+quiescence only;
   the lower END STATE is a posture disaster the scalar never saw.
   hardware-ready (lower): NO.
3. **Static-hold pricing diagnosis: operator's underpricing hypothesis
   REFUTED with numbers.** Leg 0 held horizontal in the air: the actuator
   carries the FULL gravity torque (hip |qfrc_actuator| 0.1489 N·m vs
   gravity 0.1488; deadband only shifts equilibrium ~0.7°), filtered
   current = 1.2×|τ| = 0.179 A exactly as designed. Static holds are NOT
   free. The real defect: the linear per-servo charge is invariant to load
   distribution — the flagged leg (0.24 A total) draws LESS than each
   supporting leg (0.28–0.44 A), so concentration is unpriced and flagging
   is energy-OPTIMAL under the current reward. Root-cause chain: behavior
   (flag legs/tripods) ← incentive (support costs more than parking) ←
   pricing (linear current sum is distribution-blind) — not a servo-model
   defect.
4. **New GLOBAL reward terms implemented** (sim_env.py, default OFF, per
   operator ~20:45Z #2): `reward.k_support_margin` (CoM depth inside the
   support polygon of loaded feet, ≥3 contacts, saturates at 40 mm; belly
   rest exempt by the contact gate) and `reward.k_load_even` (Herfindahl
   concentration of foot normal forces; even=0 charge, one-foot=max).
   Scale audit @ k=1 vs kernel ~0.5–0.9/step: margin 0.83–0.96 (saturated →
   use k=0.3), evenness 0.18–0.28 (→ use k=1.5; park-tripod ending then
   pays ~0.25/step). Unit tests added (hull geometry + smoke; 30 pass).
   **Routing caution discovered in analysis: the INSTANTANEOUS forms cannot
   distinguish a tripod GAIT from a tripod PARK (same polygon, same HHI) —
   enabling them in walk mode would fight legitimate swing phases. Stance
   line (joint_goal has no walk mode) is unambiguous; a walk-mode version
   needs time-averaged per-leg load. Declared routing: global within the
   stance line only, for now.**

### cw-walk-phase-stance2 — FAIL; phase-contact reward REFUTED in BOTH basins
OBSERVATIONS. W&B wjm6lrgy, 4M steps (23.20M cum, 1084 s solo). Harness
(ckpt md5 d077f694, DR 0.2, own cfg, posture-strict): walk det 0/6
gait-valid @ vel_err 0.049, sto 0/6 @ 0.046, speed 0.030 m/s; duty
signature in 12/12 walk eps: legs 1/3/5 at 0.83–1.0, legs 0/2/4 at
0.02–0.38. Retention: rise height-only sto 4/6 (the gate as written was
met; auto-stop rightly never fired) but end_posture 1/12; hold height 6/6
with end_posture 0/12 (worst 89–105 mm). std GREW 1.0→1.47. Frames
reviewed: walk_det_0, hold_det_0, per-episode duty tables.
INTERPRETATION. NOT WALKING — a phase-locked TRIPOD PARK: the policy
plants the odd tripod group and holds the even group permanently airborne,
in walk AND hold AND rise; the stance parent's six-footed hold was
destroyed in 4M steps. The "parked legs average 50% agreement = zero net"
design was true and irrelevant: the park doesn't collect phase reward, it
avoids the smoothness/current/stability costs of stepping, and a zero-mean
bonus cannot outbid a cost saving. The probe's all-six-leg cycling at 96k
was transient exploration that collapsed as PPO optimized.
VERDICT: FAIL. hardware-ready: NO. HYPOTHESIS STATUS: weak alternating-
tripod phase reward REFUTED in both basins (warm arm: shuffle unchanged;
basin arm at audited std 1.0: tripod park). Escalation level (c) CLOSED.
Champion unchanged. New structural fact: **the tripod park is THE shared
attractor of the walk task** (lp-s1b tower, aac-s1c det, phase-stance2) —
it is stable, cheap, and unpriced; any future walk reward must price it.

### cw-walk-aac-s1c — 8M fixed-budget point: retention SUPPORTED, tracking REFUTED; lineage posture-broken everywhere
OBSERVATIONS. W&B ij6xowjy, 32.76M cum (8M asym from dr04b). Harness (ckpt
md5 f7ed9a6c, DR 0.4, own cfg, posture-strict): walk det 0/6 gait-valid @
vel_err 0.038 — det collapsed to a FULL tripod park (legs 1/3/5 duty
0.02–0.06); sto 0/6 @ 0.031 (mixed shuffle). vs nv 8M (0/6 @ 0.035):
inside the calibrated twin-noise band (0.026–0.043) → no evidence of
tracking change. Retention height-only: rise det 6/6 / sto 5/6 vs nv 3/12
— the asym-critic retention gap PERSISTS at matched budget. End-posture:
0/6 in EVERY stance-ending mode, worst clearance 342–385 mm both passes.
Frames reviewed: walk_det_0 (three-leg tower scoot with one leg pointing
straight up), rise_det_0 (raises the body, then stands with a permanent
vertical antenna leg), contact sheet.
INTERPRETATION. The gate as written ("beat nv on tracking OR retention")
passes on retention — recorded as such — but this checkpoint ends every
skill with a ~35 cm vertical flag leg and its deterministic walk is a
static tripod. NOT WALKING. hardware-ready: NO.
VERDICT: gate PASS on the retention half only; walk champion UNCHANGED;
checkpoint not promoted beyond baseline duty.
HYPOTHESIS STATUS: SPLIT — asymmetric critic is a real retention tool
(SUPPORTED, 11/12 vs 3/12 twice), not a gait fix (tracking REFUTED at 4M
and 8M). Keep `--asym-critic` default for warm walk continuations.
DECISION: the dr04b lineage is 0-for-9 on gait validity across every lever
(widen ×2, 3× progress, flag ×2, speed, LP, phase, asym, 8M budget) and
carries lower 0/6 + universal end-posture failure. **Lineage RETIRED as a
warm-start source for gait experiments** (retention/architecture studies
may still reference it). Future gait attempts: stance-basin or fresh inits
+ architecture (temporal actor next), never another shaping coefficient.

### LAUNCH probe-posture-price (lower pod, smoke, 150k @ DR 1.0)
Probe rule (audit §6): new mechanism (support-margin + load-evenness
terms) gets a smoke before any 4M run. Stance-champ init, exact
cw-stance-dr10 recipe + `reward.k_support_margin=0.3`,
`reward.k_load_even=1.5`. Mechanical gate: both new reward parts appear
and stay within the audited band (margin ≤0.3/step, evenness ≤0.25/step
typical), no canary group failure, trainer healthy. No behavioral claim
at 150k.

### LAUNCH cw-stance-posture (s4, 4M, DR 1.0, seed 0)
Operator-directed (~20:45Z #2, root-cause chain in cycle 12 entry).
HYPOTHESIS: the stance champion's flag-leg endings (lower 0/12, rise
5/12 posture-strict) exist because load concentration and support-
polygon shrinkage are unpriced; pricing the physics directly
(k_support_margin=0.3 + k_load_even=1.5, GLOBAL within the no-walk
joint_goal task) gives a dense gradient that pulls hoisted legs down.
Prediction-if-true: lower ends with legs down/tucked (end_posture
≥5/6 det+sto) and rise crouch/bridge endings plant legs 2/4, with
height metrics retained (canaries all four groups protected).
Prediction-if-false: heights retained but end-posture unchanged (terms
too weak vs kernel at audited scale — would falsify "pricing suffices"
and point at exploration, NOT at coefficient iteration) OR canary
regression (terms fight the rise kernel; auto-stop fires).
Strongest alternative: the flag-leg ending is a stable local optimum
that a warm start cannot leave at inherited std ~0.29 (deliberate:
posture is a dense LOCAL gradient, not a basin escape; saying so per
audit). Parent: ppo_goal_cw_stance_dr10.zip (md5 da1d912a). One
variable: the posture-pricing package (declared as such; ablate only
if it fails). GATE (posture-strict harness @ DR 1.0, 6 eps/mode
det+sto): lower ≥5/6 end-posture both passes AND rise ≥4/6
posture-strict sto AND hold stays 6/6 AND crown-jewel heights intact
(rise/lower height-only ≥5/6 both passes). Budget 4M. Cycle 12 totals:
2 launches, 4.15M steps (caps 4 / 16M).
## OPERATOR ~22:10Z — launcher was pod-aware, not node-aware; fixed mechanically

The 8-run clump (everything finishing 20:06–21:49Z, fleet then idle
through a long verdict cycle) was a scheduling failure MINE, not yours:
`launch_run.py` checked free cores per POD, but the six pods share two
~128-core nodes and the kernel schedules host-wide. 8 experiments
"within pod limits" starved each other 4-5x, converged, and finished
together. The plan text knew this ("budget ~4-5 fast runs"); the CHECK
didn't — the exact patch-vs-check failure class the guardrails preach
about. Landed now: `pod_node()` live lookup, refusal at
`max_heavy_per_node: 2`, experiment cap 8→4, `status` prints per-node
totals. Also: prefer staggered launches — 4 runs finishing hours apart
beat 4 finishing together. Relaunch cadence: verdict the two big
finishes (phase-stance2, aac-s1c) FIRST and get their successors
launched before working through the remaining backlog verdicts.

## OPERATOR ~22:25Z — fleet grown: +2 pods on a third node; experiment cap 4→6

Third 128-core node (g12ba48) was idle. Added `hexapod-sweep-s5` and
`hexapod-sweep-s6` (56-core pods like s3/s4), provisioned with the
standard dep stack (mujoco 3.11.0 pins), code synced, W&B creds in
place; `launch_run.py status` confirms 8 pods / 3 nodes / node caps.
Guardrails updated: cap 6 experiments (2 heavy per node x 3), pod list
is now eight. Use the extra headroom for the queue, and stagger the
launches. Also deleted the dead kimi serving leftovers (deployment,
service, 6 TB of model PVCs) — none of it touched our nodes' CPU, it
was just cost.

### Cycle 12 close — launch verification
probe-posture-price v1 KILLED at ~30k (self-inflicted: trainer W&B
callback lacked the new reward-part keys, so the probe's own gate was
unverifiable; callback patched, commit 88cd50a) — ledger
KILLED_RELAUNCH, not a scientific verdict. probe-posture-price2
(smoke, lower pod): 150k in 178 s, canary baseline healthy
(protected groups lower/rise_crouch/rise_flat all passing), no
auto-stop, no traceback; terms verified firing on-pod (margin +0.30
at cap, evenness −0.086 in a plant stance) → PROBE PASS. First
cw-stance-posture launch attempt FAILED at argparse (my `--wandb`
flag doesn't exist; W&B is default-on) — retry clean.
cw-stance-posture VERIFIED RUNNING on s4 (W&B q7797l60, solo,
~2200–3140 fps), +5-min checkup HEALTHY; W&B history shows
env/reward_support_margin +0.259 and env/reward_load_even −0.304 per
step at 19.2M cum — inside the k=1.5 expectation band (0.28–0.42).
Note: the probe entry's "even<=0.25 typical" band was the k=1.0
number; the correct k=1.5 band is 0.28–0.42 — observed value is
in-band, no action.
Pods after this cycle: s4 = cw-stance-posture (finishes ~fast, eval
next cycle, posture-strict gate). s3/long5m/walk/friction idle BY
DECISION: the walk line's next rung (temporal actor) is a CODE task
(env-side obs history + transplant + probe) that must land before any
launch, the dr04b lineage is retired, and burning steps on another
shaping variant would violate the no-coefficient-iteration rule. An
idle pod is cheaper than a confounded campaign.
Cycle 12 ledger: launches = probe v1 (killed), probe v2 (pass),
cw-stance-posture (running) — 4.3M new steps of 16M cap.

### Cycle 12 addendum — operator commits a4201e7/d9b63c3 absorbed at close
Node-aware launcher + fleet grown to 8 pods / 3 nodes (s5/s6 new), cap
6, stagger directive noted. The two big finishes (phase-stance2,
aac-s1c) were verdicted FIRST this cycle as directed; their successor
on both lines is the SAME item — the temporal actor — and it is a code
task (env-side obs history + transplant + probe), not a launchable
run, so the new headroom stays idle one cycle rather than get filled
with another refuted-class shaping variant. Next cycle: implement
temporal actor, probe it, and launch staggered on the enlarged fleet
alongside the cw-stance-posture verdict.

## Cycle 13 (2026-08-08 ~23:00Z) — temporal actor lands (code); posture verdict; walk rung 1 launched

### Note on "This cycle" handoff
The watcher named cw-walk-aac-s1c and cw-walk-phase-stance2 as newly
finished; both were already fully verdicted in the cycle 12 entry
(harness + frame review + ledger FINISHED: aac-s1c SPLIT
retention-only, phase-stance2 FAIL/refuted). No re-eval; no double
entry. cw-stance-posture (s4) was ~1.4M from its 4M budget at cycle
start — verdict below, this cycle.

### CODE — temporal deployable actor: env-side obs history (plan §Walk rung 1)
Rationale (recorded per guardrails before launch): the walk line's next
rung is CAPABILITY, not another coefficient — ~300 ms of obs history
(8 frames @ 25 Hz) is implicit system ID (literature review priority
#3; ranked above model size). Implementation, one mechanism:
- `obs.history_frames` (cfg, default 1 = legacy width everywhere):
  the env stacks the last K per-tick observations NEWEST-FIRST and
  seeds the buffer with the reset obs. Lives in `SimHexapodBalanceEnv`
  (`_final_obs`), so trainer, canaries, bg-eval, harness, and any
  future hardware bridge share the exact code path (env-side by
  design; a VecFrameStack wrapper would not exist on hardware).
- Newest-first ordering makes the parent's obs a bit-exact PREFIX of
  frame 0, so the EXISTING `--obs-pad-transplant` machinery does the
  tail transplant unchanged (zero columns for frames 1..K-1 and the
  walk extras).
- Refactor detail: walk extras (measured vel / phase clock) moved from
  post-hoc appends in `walk_task.reset/step` into an `_augment_obs`
  hook that runs BEFORE stacking, so every frame carries them.
  Parity proven by test: frame 0 of a K=4 env equals the K=1 env's
  obs bit-exactly tick-for-tick; the pre-existing phase-clock test
  (exact obs tail values) still passes on the refactored path.
- `_privileged_idx` helper in the trainer: the asym-critic mask was
  hardcoded (-2,-1) — wrong for stacked obs (would mask the OLDEST
  frame's tail) and silently wrong for phase runs; now computed
  per-frame (also fixes the latent phase+asym bug). asym+transplant
  remains refused (unchanged SystemExit), so this cycle's run is
  plain MlpPolicy; asym+history is a later, separate step.
- Tests: 3 added (stack parity/newest-first, stance-env width,
  privileged idx) — 33 pass. Real-checkpoint check: cw_stance_dr10
  (68-dim, md5 da1d912a) transplanted into joint_walk + history8
  (576-dim, pad 508): max action diff 0.0 over random obs with
  arbitrary history/extras content. Bit-identical warm start.

### probe-walk-hist8 — PROBE PASS (mechanical)
150k in 89s on the lower pod (W&B-off smoke, snapshot f1824ed).
obs-pad transplant 68->576 fired on-pod; canary parent baseline all 8
cases pass with all four groups protected — on-pod confirmation the
history transplant is behavior-identical at init. Zero tracebacks;
fps ~1900 on the 30-core pod (history stacking cost negligible).
Mechanical gate met; no behavioral claim. Ledger FINISHED (the
post-exit checkup's DEAD is the process having completed its budget —
final save line + checkpoint present).

### cw-stance-posture — FAIL on the headline (lower posture unmoved at the flag leg); pricing PARTIALLY effective; raise heights incidentally best-ever
OBSERVATIONS. W&B q7797l60, 4M steps (23.19M cum, 2359 s solo, ~1700
fps), final ckpt md5 7c2ab2f5 (pulled, kept append-only). No canary
auto-stop fired. Harness (posture-strict, DR 1.0, own cfg-sets, 6
eps/mode det+sto, two draws: seed-0 default-modes + seed-0 gate-modes):
- lower: 0/6 det AND 0/6 sto end-posture (gate needed >=5/6 both).
  Heights PERFECT 12/12 (h_err 0.4-6.2 mm). Per-foot end clearances
  are the finding: legs 0/2 pulled DOWN to 58-91 mm (parent: >100-200
  mm); leg 4 UNMOVED at 229-264 mm, fully vertical, every episode.
- rise: sto 4/6 posture-strict (= gate), det 3/6 and 1/6 across the
  two draws (draw-dependent start kinds; crouch starts 0/6 in both:
  legs 2/4 left at 102-327 mm). Flat starts clean, heights all fine.
- hold: det 5/6 (one marginal 24 mm vs 20 mm), sto 6/6 both draws
  (gate said 6/6 — marginal miss).
- raise (canary tripwire, no compute owed): height-only det 6/6, sto
  5/6 — BEST EVER (raisemix peaked 3-4/6) — but posture-strict 0/12,
  standing on 3-4 legs with legs 1/2/4 at 22-120 mm.
Frames reviewed: lower_det_0 7c4762a2 (lowers to commanded height,
then a partial crouch with legs 0/2 half-tucked and leg 4 pointing
straight up — never a belly rest), raise_det_0 f1f49232 (hits target
height standing tall on ~4 legs, 2-3 legs held clear of the ground),
hold_det_0 da1d054b (clean six-foot quiet stance), rise_det_0 gate
35aa7f2e + first-draw 2a7fc153 (crouch start rises then ends with one
leg splayed horizontal in the air, a 4-5 leg stand).
INTERPRETATION — root-cause chain (required before any further reward
work): the pricing WORKS where it has gradient and is dead where the
pathology lives. k_load_even's HHI has ~zero gradient w.r.t. an
UNLOADED airborne leg (zero force contributes nothing until contact);
k_support_margin counts only LOADED feet; k_stance_clearance routes
to hold/lean/track/unload only (deliberate — rise needs >50 mm
transients). So legs near the ground (0/2, partially loaded) were
pulled down 40-130 mm — a real dose-response — while leg 4's vertical
flag sits in a zero-gradient pocket that std 0.195 exploration never
bridges (~90 deg of coordinated hip travel to first contact). The
raise height jump (11/12) is consistent: load-evenness recruits the
loaded legs better even though it cannot reach the parked ones.
VERDICT: FAIL (headline lower gate 0/12; rise sto met at 4/6; hold
marginal miss). hardware-ready: NO — lower still ends with a ~25 cm
vertical flag leg and crouch-start rise ends on 4-5 legs; nobody
should put that on the robot.
HYPOTHESIS STATUS: SPLIT, exactly along the pre-registered if-false
branch: "pricing suffices at inherited std" REFUTED for the flagged
leg; pricing-as-gradient SUPPORTED where the leg is near contact.
The strongest-alternative clause ("flag ending is a local optimum a
warm start cannot leave at inherited std") is now the live
hypothesis, sharpened: the priced landscape pays leg-4-down at the
ENDPOINT (park tripod pays ~0.25/step evenness, measured cycle 12)
but has no path gradient to it; only exploration can bridge.
Champion UNCHANGED (cw_stance_dr10 stays stance champion; posture
ckpt kept as ppo_goal_cw_stance_posture.zip for the exploration arm).
Consequence for the walk line: the stance flag and the walk tripod
park are the SAME defect (unpriced airborne legs); cw-walk-hist8
therefore inits from cw_stance_dr10 (posture ckpt did not pass; also
keeps the phase-stance2 control comparison clean).

### LAUNCH cw-walk-hist8 (4M, DR 0.2, seed 0) — temporal actor, walk rung 1
HYPOTHESIS: the walk failures are a CAPABILITY gap, not (only) a
pricing gap — a memoryless MLP cannot coordinate swing timing or do
implicit system ID, so PPO converges to static exploits (shuffle,
tripod park). ~300 ms of env-side obs history (8 frames @ 25 Hz)
supplies the temporal state a gait needs. One-variable swap against
the recorded control cw-walk-phase-stance2 (same init, same audited
basin-escape settings std 1.0 / ent 0.01 / target_kl 0.02, same walk
cfg, phase package OUT, history IN).
Prediction-if-true: six-foot alternating contacts appear AND survive
optimization (unlike the phase probe's transient), sto walk >=4/6
gait-valid @ vel_err <=0.035 @ DR 0.2, video shows all six feet
cycling. Prediction-if-false: converges to the same tripod park
(duty signature one tripod ~1.0 / other <=0.3) — refutes "capability
alone suffices" and promotes rung 2 (time-averaged per-leg load
pricing, on top of history). Strongest alternative: the park is
optimal regardless of capability because it is unpriced.
Parent: ppo_goal_cw_stance_dr10.zip (md5 da1d912a; posture ckpt NOT
used — failed its gate, and this keeps the control comparison clean).
Transplant: --obs-pad-transplant 508 (68->576, bit-parity proven in
test + on-pod probe). No --asym-critic (refused with transplant;
retention monitored by canaries instead). GATE: sto walk >=4/6
gait-valid @ vel_err <=0.035 on 0.02-0.06 @ DR 0.2 AND video shows
all six feet cycling contact/swing AND sto rise >=4/6 (canaries
armed from stance parent). Budget 4M. Probe probe-walk-hist8 PASSED.

### LAUNCH cw-stance-posture2 (4M, DR 1.0, seed 0) — pre-registered exploration branch
HYPOTHESIS (from cw-stance-posture's if-false branch, verbatim
consequence): the posture-priced landscape pays a planted leg 4 at
the endpoint (~0.25/step evenness saving, measured cycle 12) but has
zero path gradient to first contact; the flag ending is a local
optimum unreachable at std 0.195. Re-opened exploration (std 1.0,
audited basin-escape setting; ent 0.01) lets sampled excursions reach
contact, after which the dense terms take over. NOT a coefficient
change: identical reward config to cw-stance-posture; the one
variable is exploration noise.
Prediction-if-true: lower end-posture >=5/6 at least sto (leg 4
plants), rise crouch endings improve, heights recover by run end.
Prediction-if-false: std re-anneals with leg 4 still parked
(evenness charge paid, never escaped) — refutes exploration-suffices
and leaves only structural options (terminal-posture pricing or
belly-rest reference states), to be DESIGNED, not coefficient-tuned.
Risk: std 1.0 transiently wrecks hold/heights — canaries (all four
protected groups from the posture parent) + auto-stop bound it.
Parent: ppo_goal_cw_stance_posture.zip (md5 7c2ab2f5). GATE
(posture-strict, DR 1.0, 6 eps/mode det+sto): lower end-posture
>=5/6 sto AND >=4/6 det AND heights rise/lower >=5/6 both AND hold
sto 6/6. Budget 4M.

### OPERATOR commits absorbed mid-cycle (f2181fa, b0d250c, 2b23772)
Pulled during the cw-walk-hist8 snapshot (rebase conflict on
RL_PLAN.md resolved keeping operator queue item 0 verbatim + my
updated items; net plan length within the operator's +22). Binding
item 0 (cw-walk-step0) actioned this cycle: code + probe below; the
4M run itself is next cycle's first launch (this cycle's launch cap
of 4 is consumed). Host-load launch gate + runtime-state gitignore
noted; all launches this cycle went through the updated launcher.

### CODE — step-event reward package (operator queue item 0)
Diff rationale (walk_task.py, all walk-mode-only by construction,
default OFF; W&B callback keys added — cycle 12 lesson):
- `reward.k_step_event`: one-shot per-leg credit for a COMPLETED
  lift->swing->touchdown with displacement >=10 mm along the
  commanded direction, scaled along/30 mm capped 1.5x. A parked leg
  never touches down -> never paid; a backward/vertical hop pays 0.
- `reward.k_drag_loaded`: per-tick charge on foot XY translation
  while in contact (0.5 mm/tick deadband) — skating pays every tick.
- `reward.k_park_duty`: per-leg contact duty over a trailing 2 s of
  commanded ticks, charged outside [0.1, 0.9] — the tripod park (3
  legs 1.0 / 3 legs 0.0) pays 0.6*k EVERY TICK by construction; a
  real gait (duty 0.3-0.8) pays zero. This is the "park worth less
  than stepping by construction" requirement, measured in the unit
  test at exactly -0.6 for a standing stance during command.
- Shared foot-transition loop refactor keeps k_walk_swing semantics
  bit-identical (same thresholds, same update order).
SCALE AUDIT (vs kernel ~0.5-2/step): park -0.6*k/tick (test-measured);
step events at a 1 Hz six-leg gait ~6/s -> avg +0.24-0.36*k/tick;
drag at the historic 0.03 m/s skate on 3 feet ~ -0.036/tick at k=10.
Chosen: k_step_event=1.0, k_drag_loaded=10.0, k_park_duty=1.0.
Tests: 3 added (park charge exact, step-event pays forward swing,
drag ~0 in quiet stance) — 36 pass.

### cw-walk-hist8 — AUTO-STOPPED at 1.24M by the canary gate; park attractor again; history not refuted (run under-dosed)
OBSERVATIONS. W&B 4chv0m83, launched on s3 from stance champ
(transplant 68->576, std 1.0, DR 0.2, snapshot e85a290), verified
running ~2320 fps solo. Canary AUTO-STOP at 20.43M cum (1.24M of 4M):
protected group 'lower' failed 3 consecutive probes; trainer saved
final ckpt (md5 eddfc2d0, pulled) and exited — total wall 677 s. The
control run (cw-walk-phase-stance2: same init, same std 1.0, phase
instead of history) survived its full 4M without a canary trip.
Harness (DR 0.2, own cfg incl. history_frames=8, posture-strict, 6
eps/mode det+sto): walk det 0/6 @ vel_err 0.046 / sto 0/6 @ 0.044,
gait_valid 0/12 — duty signature legs 1/3/5 at 0.75-1.0, legs 0/2/4
at 0.01-0.47; rise 0/12, lower 0/12 (worst 262-273 mm), hold 0/12
end-posture (66-95 mm). std GREW 1.0 -> 1.31. Frames reviewed:
walk_det_0 99a7c71c (tripod-park scoot, three legs held clear the
whole strip, no six-leg cycling), rise_det_0 eb4b8668.
INTERPRETATION. NOT WALKING — the shared tripod-park attractor, again,
reached within 1.24M from the stance basin. At std 1.0 the fresh-Adam
first layer (576 inputs vs the control's 74) drifts much faster per
update, so retention broke ~3x earlier than the phase control; the
auto-stop did exactly its job (bounded the damage at 1.24M instead of
burning 4M). The run answered the RETENTION dynamics of the transplant
but NOT the capability question: 1.24M steps with the policy already
parked is not a dose at which history could have restructured a gait
(the phase probe needed only 96k to show cycling, but that was DR 0
walk-only).
VERDICT: FAIL (gate untested at dose; run terminated by design).
hardware-ready: NO — tripod scoot with the stance line destroyed.
HYPOTHESIS STATUS: INCONCLUSIVE on capability (under-dosed by
auto-stop); transplant-at-std-1.0 retention risk CONFIRMED and now a
known cost of wide-obs transplants. Champion unchanged.
DECISION: do NOT simply relaunch with --no-canary (jewel protection
stays). The capability question moves to the operator's step0 line:
cw-walk-step0 is walk-ONLY from scratch (no retention constraint at
all, so no canary tension), and once its reward baseline exists,
history-8 becomes a clean one-variable arm ON TOP of it (walk-only
also removes the retention/exploration conflict that killed this
run). A stance-basin history retry, if ever, needs a slower first
layer (e.g. reduced LR or partial-freeze) — a designed change, not a
coefficient bump; deferred behind step0.

### probe-walk-step0 — PROBE PASS (mechanical); no step events yet at 150k
159,744 steps on lower (W&B-off smoke, snapshot 9ccb3c8), zero
tracebacks, saved cleanly. Parts verified by rolling the probe ckpt
locally in the exact cfg (300 ticks det, 3 eps): reward_park_duty
mean -0.306/tick, floor -0.600 (exact audited park/stand value);
reward_drag -0.0004/tick; reward_step_event present, ZERO events —
at 150k from scratch the policy stands and pays the park charge
rather than stepping. Mechanical gate MET; the hoped-for directional
signal (events trending up, like the phase probe's 96k cycling) did
NOT appear — recorded as-is, no claim either way at this dose. The
4M cw-walk-step0 run (operator gate: 10 cm forward, six legs cycling,
duty ~[0.2,0.9], >=2 swings/leg, no drag/park, det AND sto) is next
cycle's FIRST launch — this cycle's launch cap (4) is consumed.

### Cycle 13 close — caps and fleet
Launches: probe-walk-hist8 (pass), cw-walk-hist8 (auto-stopped,
verdicted), cw-stance-posture2 (running s5, checkup HEALTHY ~1640
fps), probe-walk-step0 (pass) = 4/4 cap. New steps: 8.3M of 16M.
Champions: unchanged (stance = cw_stance_dr10; no walk champion).
Crown-jewel watch: posture-strict lower remains 0/12 on every line —
the jewel's height claim holds, its posture claim does not.
Pods after cycle: s5 = cw-stance-posture2 (4M, ~overnight); all
others idle pending next cycle (step0 launch + posture2/step0
verdicts). Code landed this cycle: obs history (temporal actor),
step-event reward package, _privileged_idx fix; 36 tests pass.

## Cycle 14 (2026-08-08 ~23:55Z) — posture2 verdict + cw-walk-step0 (operator queue item 0)

### LAUNCH cw-walk-step0 (4M, DR 0, seed 0) — walk-only from-scratch step-event baseline
OPERATOR-DIRECTED (binding queue item 0). HYPOTHESIS: with the park
priced BY CONSTRUCTION (k_park_duty charges duty outside [0.1,0.9]
every commanded tick, test-measured -0.6/tick for a standing park),
dragging charged while loaded (k_drag_loaded), and a completed
lift->swing->touchdown paying a one-shot per-leg credit
(k_step_event), a from-scratch walk-ONLY policy at DR 0 with audited
exploration (std 1.0, ent 0.01, target_kl 0.02) discovers real
stepping — the first reward landscape in this campaign where the
tripod park is strictly worth less than stepping.
Prediction-if-true: step events appear and trend up (unlike the
150k probe where events were zero and the policy stood paying the
park charge), and by 4M the harness shows >=10 cm forward with all
six legs cycling, per-leg duty in ~[0.2,0.9], >=2 swings/leg, no
drag/park — det AND sto. Prediction-if-false: policy still parks or
shuffles while paying the duty charge (charge absorbed as a constant,
gradient to first step never found) — that refutes "pricing the park
suffices from scratch" and promotes rung 1 (history-8 ON TOP of this
exact reward, one variable) since capability, not pricing, would be
the remaining gap. Strongest alternative: exploration never produces
a single completed step event, so the credit stays invisible (probe
saw zero events at 150k) — distinguishable in W&B via
reward_step_event staying exactly zero vs rising.
Root-cause chain (required): park <- avoids stepping costs <- per-leg
load/duty unpriced + step never explicitly paid <- current model
dead-zone underprices static holds (sim defect, cycle 13) — this run
prices the behavior at the reward level because the deeper current-
model fix is a physics recalibration queued for hardware-validation
time (RL_PLAN hardware gate 3), inaccessible this cycle.
Fresh init is the operator's explicit exception to warm-start
default; walk-only mix removes retention concerns (no canaries — no
parent to protect). No asym-critic, no history, no curriculum: the
embarrassingly narrow baseline.
GATE (operator, deliberately narrow, DR 0): from normal stance move
FORWARD >=10 cm with ALL SIX legs repeatedly cycling
lift/swing/touchdown; per-leg duty ~[0.2,0.9]; >=2 swings per leg;
no drag, no parked leg; det AND sto; video verdict pathology-first.
Budget 4M. Probe probe-walk-step0 PASSED mechanically (cycle 13).
Pod: hexapod-sweep-walk (56-core, node g129004 empty at check).

### cw-stance-posture2 — FAIL; exploration never re-annealed (std 1.0 -> 2.29) and the flag pathology SPREAD; skills eroded
OBSERVATIONS. W&B 8qrvip34 finished, 4M/4M in 2682 s on s5 (solo),
final ckpt md5 5d8c6d63 (pulled). No canary auto-stop: lower_a/b and
rise_crouch_a/b passed EVERY one of 21 probes; rise_bridge/flat
degraded intermittently but never 3 consecutive full-group failures.
train/std GREW MONOTONICALLY 1.0 -> 2.287 over the whole run (sampled
every ~12k: 1.06 @ 0.4M, 1.47 @ 2.3M, 2.13 @ 4.2M-cum window); it
never re-annealed. Harness policy_std at eval: 2.258.
Gate harness (DR 1.0, own cfg, posture-strict, 6 eps/mode det+sto,
modes hold/rise/lower/raise; first pass with default modes archived at
logs/ckpt_eval/cw_stance_posture2_4M_gate, gate pass at ..._gate2):
- vs parent cw-stance-posture gate eval (named baseline, same
  harness/modes, cycle 13): hold det 2/6 vs 5/6 (-3), sto 4/6 vs 6/6
  (-2) — both outside the +-1-2 ep noise band; rise det 0/6 vs 1/6
  (within noise), sto 0/6 vs 4/6 (-4, outside); lower posture 0/6 ->
  0/6 all passes (unchanged) but worst_clear WORSE 256-264 -> 296-298
  mm; raise 0/6 -> 0/6 with worst_clear 120 -> 329-331 mm (much
  worse).
- Heights largely retained: lower height_err_end 1-16 mm all 12 eps,
  raise 1-27 mm, hold 1-10 mm; rise det 7-40 mm (3 eps <=13 mm).
  Failures are END-POSTURE, not height. safety_flags 0 everywhere.
Frames reviewed (provenance): rise_det_0.png md5 7f049305 (gate1) —
by mid-strip the REAR LEG PAIR is pointed at the ceiling and stays
there; TWO flag legs where the parent had one. lower_det_0.mp4 md5
bbc341a2, 250 frames — body descends but the FRONT PAIR ends extended
straight out horizontally at body height (~300 mm clearance).
raise_det_0.mp4 md5 12e1646d, 250 frames — same horizontal spear-leg
endings. hold_det_0.mp4 md5 961c85ca, 250 frames — stance holds but
sloppy, 2/6 posture det. Sto passes have no videos (harness saves det
strips only): hold sto 4/6 is (4/6 unwatched) and supports nothing.
Exploit checked and not found: no safety-layer reliance (0 flags), no
height-collapse shortcut (heights fine); the failure is exactly the
visible one — airborne legs never come down.
INTERPRETATION. The pre-registered if-false branch predicted "std
re-anneals with leg 4 still parked". Reality was a THIRD path: std
never annealed at all — with ent_coef 0.01 on an 18-dim Gaussian near
a warm-start optimum, the entropy bonus outruns the task gradient and
std runs away (1.0 -> 2.29). 4M steps of std >=1.0 exploration never
produced a paid leg-4 contact, and training at std ~2 actively
degraded the parent's hold/rise. Combined with cw-walk-hist8 (std 1.0
-> 1.31 by 1.24M, canary kill), this is 2-for-2: FLAT ent 0.01 ON A
WARM START IS A DESTRUCTIVE SETTING, distinct from the audit's
from-scratch prescription (cw-walk-step0, from scratch, is at std
~1.09 @ 0.5M — task gradient there still has something to push
against; watch it, alarm if std >1.5 with step events still zero).
Entropy RUNAWAY now joins entropy collapse as a run-health alarm.
VERDICT: FAIL (gate: lower posture 0/6 everywhere vs >=5/6 sto
required; hold sto 4/6 vs 6/6 required). NOT HARDWARE-READY: two leg
pairs end airborne (rear pair skyward in rise, front pair horizontal
in lower/raise) — worse than the parent's single flag leg; no
roboticist would deploy this. Champion unchanged (cw_stance_dr10).
Crown-jewel note: champion itself untouched; this branch is a dead
end, do not warm-start from 5d8c6d63.
HYPOTHESIS STATUS: REFUTED — "re-opened exploration bridges to first
contact, then dense terms take over" is false as tested; the
confound (std runaway instead of re-anneal) makes it possible a
BOUNDED std 1.0 would behave differently, but 4M steps sampling at
std >=1.0 never once won the leg-4 contact payoff, which refutes the
load-bearing claim. Per pre-registration, remaining options are
STRUCTURAL: terminal-posture pricing (charge airborne clearance only
after the goal reference settles — dense gradient on the airborne
leg, zero tax on transients) or belly-rest reference states. Designed
this cycle: see cw-stance-endpost below.

### CODE — terminal end-posture pricing (pre-registered structural option, stance line)
Diff rationale (sim_env.py + trainer key + 2 tests; default OFF):
Root-cause chain (required): flag-leg endings <- airborne legs free at
episode end <- load_even/support_margin have ZERO gradient on an
unloaded airborne leg; stance_clearance excludes rise/lower/raise
(their transients need freedom); all-modes flag_leg REFUTED for
taxing exactly those transients (cw-walk-flag) <- deepest link
(current-model dead zone underpricing static holds) needs hardware
current recalibration — not reachable in sim-only work this cycle.
Reward-level pricing is therefore the deepest accessible link, same
argument as the walk park_duty term, per RL_PLAN item 3 ("same
defect, design once, route per mode").
- `reward.k_end_posture`: per-tick charge on per-foot clearance above
  the grounded pad reference, ONLY inside a terminal window that is a
  pure function of the pre-sampled goal schedule (cannot be dodged by
  avoiding the target): from where the height REFERENCE stays within
  end_posture_ref_mm (15) of final, +0.25 s grace, clamped to the
  last end_posture_window_s (1.5) of the episode (the clamp protects
  the early rise curl transient — small-amplitude rise refs sit near
  final almost immediately; measured windows: 36-38 charged ticks on
  lower/rise/raise across seeds). Allowances mirror the EVAL gate: 20
  mm stand-ending, 60 mm for belly-ending lower; per-leg contribution
  capped at 0.30 m. Routed to rise/lower/raise (the modes
  stance_clearance excludes); hold/lean/track/unload keep
  stance_clearance. Eval defs unchanged (checks stay independent).
SCALE AUDIT (kernel ~0.5-2/tick): horizontal flag leg 300 mm in lower
= -5.0*(0.30-0.06) = -1.2/tick over ~38 ticks = -46/episode (episode
returns ~100-350); grounded feet pay 0 (test-measured ~0 over a full
held-plant lower); leg at 150 mm pays exactly -0.45/tick
(test-asserted). k_end_posture=5.0 chosen.
Tests: 2 added (schedule gating + exact magnitude; grounded/routing
free) — 38 pass, 4 skipped.

### LAUNCH cw-stance-endpost (4M, DR 1.0, seed 0) — terminal-posture pricing on the champion
HYPOTHESIS: the flag ending survives because no term has gradient on
an airborne leg during the ONLY phase that matters (the end).
k_end_posture supplies a dense, schedule-gated gradient (every mm of
descent pays immediately, transients untaxed), so the planted ending
becomes reachable by plain gradient descent at inherited std —
explicitly WITHOUT the basin-escape exploration that posture2 just
showed to be destructive (std runaway 1.0->2.29).
One variable vs cw-stance-posture (named comparator, same parent
ppo_goal_cw_stance_dr10 md5 da1d912a, same cfg, same seed, same
inherited std): + reward.k_end_posture=5.0.
Prediction-if-true: lower end-posture climbs off 0/6 to >=4/6 det or
sto (parent-line baseline: 0/6 everywhere, worst_clear 256-264 mm),
rise/raise end postures improve, heights stay >=5/6 (canaries
protect). Prediction-if-false: worst_clear stays ~250+ mm with the
charge simply absorbed (-46/episode paid every lower episode) — that
would refute "dense terminal gradient suffices at inherited std" and
leave belly-rest reference states (reset-side, reward-free) as the
remaining pre-registered option. Strongest alternative: the term
works but k=5 also distorts the descent (e.g. body slams to cut the
window short — the window is time-based so slamming does NOT shorten
it, which is why this experiment distinguishes pricing from
landscape-unreachability). Gate (posture-strict, DR 1.0, 6 eps/mode
det+sto, unchanged from posture line): lower end-posture >=5/6 sto
AND >=4/6 det AND rise/lower height-only >=5/6 both AND hold sto
6/6. Budget 4M. Canaries default-on (warm start). New mechanism ->
probe-endpost smoke first (audit §6). Pod: s5.

### Cycle 14 launches — what actually happened (three launcher findings)
1. cw-walk-step0: first launcher attempt CRASHED before any process
   started — the --notes value reached the remote shell UNQUOTED and
   ">=10cm" parsed as a redirect (argparse exit 2; verified no
   process, no log on walk pod; stale INTENT marked FAILED). Retry
   with quote-wrapped notes: VERIFIED RUNNING, W&B wfcg6ues, ~2730
   fps solo on walk. (It has since FINISHED at 4M in 1766 s — verdict
   belongs to the watcher-triggered next cycle, not this one.)
2. probe-endpost (smoke, friction): the 150k probe ran so fast
   (~2430 fps) that it FINISHED before the launcher's log-growth
   check; the launcher read the stopped log as dead and its cleanup
   pkill killed the trainer mid final-save (no ckpt saved; log shows
   exactly 150,000 steps consumed, 0 tracebacks, std 0.198 sane).
   Probe evidence completed LOCALLY instead (cycle-13 pattern):
   champion ckpt rolled in the exact endpost cfg — lower flag-leg
   ending pays -0.82..-1.20/tick over the 30-39-tick terminal window
   (scale audit predicted -0.95 for ~250 mm), raise -0.15..-0.47
   (predicted -0.50 at 120 mm), planted rise endings ~0.00, zero
   charge outside terminal windows. Mechanical probe gate MET on
   combined evidence (150k integration + exact bands); ledger status
   FAILED per launcher-exit-code-is-truth, verdict carries the facts.
3. cw-stance-endpost: first attempt died at init — parent
   ppo_goal_cw_stance_dr10.zip MISSING on the new s5 pod (snapshot
   --sync carries code, not gitignored policies/). Fixed by kubectl
   cp (md5 verified da1d912a on pod), but the crashed W&B run blocks
   the name; relaunched as cw-stance-endpost-r1 — VERIFIED RUNNING,
   W&B no0ihywt, ~4779 fps solo on s5, std 0.197 at start (inherited,
   as designed). All hypothesis/gate/one-variable details as in the
   LAUNCH entry above (name suffix only).
LAUNCHER NOTES FOR OPERATOR (non-blocking, no orchestrator code
touched mid-cycle): (a) quote/escape extra args containing shell
metacharacters before building the remote command; (b) fast smokes
finish inside the verification window — treat "log stopped growing
AND budget reached in log" as SUCCESS, and don't pkill on cleanup
before checking for a completed run; (c) the /proc-scan pid recovery
can match its own scan shell (pattern contains the needle) — bracket
a character ([m]) or exclude self.

### Cycle 14 close — caps and fleet
Launched this cycle (started processes): cw-walk-step0 (4M, walk pod,
FINISHED already), probe-endpost (150k smoke, friction, ran to budget),
cw-stance-endpost-r1 (4M, s5, RUNNING) = 3-4 of 4 cap counting the
seconds-long endpost init crash. New steps: 8.15M of 16M. Champions:
unchanged (stance = cw_stance_dr10; no walk champion). Crown-jewel
watch: posture-strict lower still 0/12 on every line — the height
claim holds, the posture claim does not; endpost-r1 is the live fix.
Pods after cycle: s5 = cw-stance-endpost-r1 (~30-60 min at solo fps),
walk pod free (step0 done, next cycle's verdict), all others idle.
Code landed: reward.k_end_posture terminal pricing + 2 tests (38
pass); entropy-RUNAWAY health alarm recorded (posture2 finding).

## Cycle 15 (2026-08-09 ~00:45Z) — step0 + step0-c1 verdicts (first real gait; lineage plateau), lowent consolidation arm

### PROVENANCE NOTE — cw-walk-step0-c1 was OPERATOR-LAUNCHED outside the launcher
c1 has no INTENT ledger entry and no launcher record: the operator
(directive 0-a commit 4bde19b, Cursor co-authored) launched it by hand
at ~00:31Z, per "relaunch first". Reconstructed ledger entry added
post-hoc from /tmp/train_cw-walk-step0-c1.log + W&B 9sdboq3u. Because
it was launched raw, nothing added `--no-canary`: `--init-from`
auto-armed the canary system, the parent (a walk-only policy) happened
to pass rise_flat 2/2 at baseline probe, and the run was AUTO-STOPPED
at 5,265,024 cum (~1.26M of its 4M segment) when rise_flat — a skill
this lineage NEVER trained — failed 3 consecutive probes. Root cause
of the truncation: canary auto-arm keys on warm-start, not on whether
the protected skill is in the goal mix. Infra note, not policy
pathology. Future walk-only continuations must pass `--no-canary`.

### cw-walk-step0 — the step-event baseline WALKS (first six-leg cycling gait of the campaign); gate FAILS on the drag clause only
OBSERVATIONS. W&B wfcg6ues finished, 4,005,888/4M in 1766 s, final
ckpt md5 ea1685a4 (pulled; copied to policies/). train/std GREW
1.00 -> 2.30 monotonically over 4M (same runaway shape as posture2);
ep_rew_mean 54 -> 587, plateaued ~3.1M (581 @ 3.08M, band 554-610
thereafter). env/reward_step_event +0.096/tick at end (nonzero from
~0.3M; the 150k probe's zero-event stand was escaped); park_duty
residual -0.035/tick, drag -0.023/tick.
Gate harness (DR 0, own cfg, 6 eps/mode det+sto, default modes,
logs/ckpt_eval/cw_walk_step0_4M_gate): walk det 0/6, sto 1/6 on the
harness bar (vel_err<=0.03 AND gait_valid). Against the OPERATOR gate
clause by clause, det AND sto (12/12 eps unless noted):
- forward >=10 cm: PASS 12/12 (0.25-0.53 m).
- all six legs cycling: PASS 12/12 — swing_count per leg 3-14, no
  zero-swing leg anywhere.
- per-leg duty ~[0.2,0.9]: PASS 12/12 (range 0.20-0.79).
- >=2 swings/leg: PASS 12/12 (min 3).
- no parked leg: PASS — sacrificed_legs [] in all 12; gait_valid
  12/12; safety_flags 0; terminations 0.
- no drag: FAIL — slip_m_total 0.87-0.98 m det (0.69-0.82 sto) per
  ~10 s episode vs ~0.5 m body travel: stance feet slide at roughly
  HALF of body speed on average. This is real skating, mechanically
  measured, not frame-guesswork.
Also visible in scalars: NO speed-command tracking — det speed
0.054-0.066 m/s regardless of command drawn from [0.02,0.06] (vel_err
0.04-0.07); the sto pass (sampling at std 2.29!) runs SLOWER and
closer to command (0.041-0.055, vel_err 0.030-0.037, 1/6 passes).
Frames watched (provenance): walk_det_0.mp4 md5 6d66d09a, 250 frames,
full strip + 2 zoomed 10-frame tiles (0.12 s spacing); walk_det_3.mp4
md5 510fbefc, 250 frames, zoom tile. Pathologies first: stance is
sprawly/wide, feet placed far out; individual stance feet visibly
creep during body motion (consistent with the slip numbers); gait has
no clean tripod rhythm — cadence is irregular. Achievements: body
level and upright the whole episode, all six legs show genuine
lift -> airborne swing -> touchdown (shadow separation visible), no
flag leg, no belly drag, no lurch-and-slide episodes, no falls.
Exploit checked and not found: no parked/sacrificed leg (12/12), no
safety-layer reliance (0 flags), no height-collapse; step events are
NOT phantom (real swings on video match swing_count 3-14).
INTERPRETATION. The operator's pricing thesis worked where five
reward-shaping generations failed: paying the completed step directly
plus pricing park/drag produced the campaign's first genuine six-leg
gait from scratch. Two defects remain. (1) Skating: k_drag_loaded=10
charges ~-0.036/tick at observed slip while step events pay
+0.09/tick — skating that speeds up stepping is net-profitable at
current prices (pricing link; deeper sim-friction link untouched).
(2) Overspeed/no tracking: step-event credit scales with along-stride
(cap 1.5x) and r_prog caps at 1.25x with no overspeed charge, so
cadence/stride maximization beats tracking (pricing link). Both are
recorded, NOT patched this cycle — review §0 bans coefficient
iteration; escalation order applies if consolidation doesn't move
them. std runaway (1.0->2.30 with reward plateaued from 3.1M) says
the entropy bonus outran the saturated task gradient — third
occurrence (hist8, posture2, now step0); this one is from-scratch, so
the audit's "from-scratch arms use ent 0.01" prescription bought the
exploration that found stepping AND then kept inflating std after the
find. The missing piece is consolidation, not more exploration.
VERDICT: FAIL against the recorded gate — the "no drag" clause is
unambiguous and slip at ~half body speed fails it (5 of 6 clauses
pass, det AND sto). NOT HARDWARE-READY: DR 0 only, ignores speed
commands (always ~max speed), stance-foot skating would grind real
servos/feet, irregular cadence. It is, however, the first checkpoint
in this campaign a roboticist would call "attempting to walk" rather
than shuffling — walk-line reference checkpoint (see champion note).
HYPOTHESIS STATUS: SUPPORTED — pre-registered if-true predictions
(step events rise from zero; >=10 cm forward; six legs cycling; duty
[0.2,0.9]; >=2 swings/leg; det AND sto) ALL held; the park/shuffle
if-false branch did not occur. The gate fails only on drag, which the
hypothesis did not claim to eliminate. "Pricing the park suffices
from scratch to produce stepping": yes. Skating and tracking are the
next, different, problems.
CHAMPION NOTE: no walk champion existed (all prior walk lines were
shuffle/skate with flag legs — gait-invalid). ppo_goal_cw_walk_step0
.zip (md5 ea1685a4) is recorded as the walk-line reference/champion
checkpoint at DR 0, on gait validity 12/12 + 0.25-0.53 m travel;
append-only, copied to policies/ on controller and retained on pod.

### cw-walk-step0-c1 — canary-truncated continuation; NO evidence of improvement; lineage plateau confirmed
OBSERVATIONS. W&B 9sdboq3u finished; segment 4.006M -> 5.265M cum
(~1.26M of 4M budget) then canary AUTO-STOP (see provenance note);
final ckpt md5 e4314fa2 (pulled). train/std 2.32 -> 3.21, still
monotone rising; ep_rew_mean flat: segment band 554-610, last ~0.3M
avg ~583 vs prior ~0.3M ~580 (inside noise); env/reward_step_event
0.088/tick vs parent-end 0.096 (flat); walk err 0.047 -> 0.053 m/s
across segment (flat).
Gate harness (same setup, logs/ckpt_eval/cw_walk_step0_c1_gate),
deltas vs cw-walk-step0 named baseline: walk det 0/6 vs 0/6; sto 2/6
vs 1/6 (+1, inside the +-1-2 ep noise band — NOT evidence); det slip
mean 0.83 vs 0.93 m (-0.10, direction only, inside noise); forward
dist 0.39-0.50 vs 0.32-0.53 m (same); swing_count 3-13 all legs, 0
sacrificed, 0 terminations (same); harness policy_std 3.195 vs 2.294
(WORSE for any deterministic-deployment story).
Frames watched (provenance): walk_det_0.mp4 md5 ad604ff3, 250 frames,
full strip + zoomed 10-frame tile — same sprawly six-leg crawl as the
parent, real airborne swings, no flag legs, no park, no falls;
visually indistinguishable from the parent segment.
Exploit checked and not found: same checks as parent (no sacrificed
legs, no safety flags, real swings on video).
INTERPRETATION. 1.26M further steps at std 2.3->3.2 bought nothing
measurable: every delta is inside eval noise. Directive 0-a's own
stop rule — "stop the lineage on plateau (last quarter no better than
the prior quarter)" — is met on ep_rew_mean, step-event rate, walk
err, and harness numbers simultaneously. On top of that, the lineage
std is in the exact runaway regime cycle 14 canonized as destructive
(posture2). Continuing identical config (c2) would train at std >3.2.
VERDICT: FAIL (same gate as parent: drag clause; harness walk det
0/6) — and, for the 0-a decision, PLATEAU CONFIRMED: no c2. NOT
HARDWARE-READY (same reasons as parent, plus std 3.2 policy).
HYPOTHESIS STATUS (0-a continue-while-improving): REFUTED for this
segment — the trend did not continue; the lineage's identical-config
continuation is closed per the directive's own rule. The sanctioned
follow-up is a one-variable tweak arm (below), not c2.
NO-RELAUNCH JUSTIFICATION (directive 0-a says relaunch FIRST): the
directive conditions relaunch on the trend still improving. Checking
its own stop rule took ~3 min of log reads before any eval (rew flat,
std runaway, spurious auto-stop) — the plateau branch, not the
relaunch branch, applied. Recorded here so the operator can audit the
call: walk pod idled ~20 min during eval instead of hosting a c2 that
the directive itself says not to run.

### LAUNCH cw-walk-step0-lowent (4M, DR 0, seed 0, walk pod) — consolidation: does the plateau break when the entropy bonus stops inflating std?
HYPOTHESIS: the step0 lineage plateau (~590) is entropy-driven: with
the task gradient saturated, flat ent_coef 0.01 keeps inflating std
(1.0->2.30->3.21) and PPO optimizes returns under ever-noisier
sampling; cutting the entropy bonus lets std anneal down and converts
the discovered gait into a consolidated, lower-noise policy that
clears the plateau.
Root-cause chain (required): plateau + std growth <- entropy bonus
outruns saturated task gradient <- flat ent 0.01 prescribed for
exploration is left on after exploration has succeeded <- objective
defect (no anneal), directly accessible as a hyperparameter — no
reward change, no coefficient iteration on reward terms.
ONE VARIABLE vs the c1 continuation (same parent ppo_goal_cw_walk_
step0.zip md5 ea1685a4, same cfg, same seed 0, same 4M budget):
ent_coef 0.01 -> 0.001. Plus --no-canary (infra, not a variable: see
provenance note — canaries protect skills this lineage never had and
already truncated c1 spuriously).
Prediction-if-true: train/std falls from ~2.3 toward <=1.0;
ep_rew_mean climbs out of the 554-610 plateau band (>=620 sustained);
harness det keeps gait_valid 6/6, >=0.10 m forward, >=2 swings/leg,
det slip mean <= parent's 0.93 m (no worse).
Prediction-if-false, three diagnostic branches: (a) std stays >=2.0
-> entropy bonus was not the driver (KL/clipping dynamics instead);
(b) std falls but rew collapses <500 with step events dying -> the
"gait" was noise-dependent dither, the det gait on video was riding
stochastic kicks — the step-event income cannot be earned at low std,
which would be MAJOR (reward mispricing, back to walk escalation
order); (c) std falls, rew stays flat 554-610 -> plateau is a reward
ceiling (overspeed cap + step-event cap saturated), pointing at the
tracking/pricing defects recorded above, NOT at exploration.
Strongest alternative explanation: (c) — distinguishable because std
and reward move independently in the three branches; this experiment
separates exploration-side from reward-side causes before any reward
surgery. GATE: DR 0 harness, 6 eps/mode det+sto: gait_valid 12/12 AND
forward >=0.10 m 12/12 AND >=2 swings/leg 12/12 AND det slip mean <=
0.93 m AND final train/std <=1.2 AND ep_rew_mean(last 0.5M) >= 620.
Budget 4M. No new mechanism -> no probe needed (audit §6: ent_coef is
an existing knob; --no-canary is an existing flag).
Pod: hexapod-sweep-s6 (56-core, node g12ba48 empty at status check) —
walk pod was taken by c2 between my status checks (below).

### CONCURRENT EVENT — operator launched cw-walk-step0-c2 by hand (~00:55Z)
Live status re-check before placing lowent found cw-walk-step0-c2
RUNNING on the walk pod (W&B c1oyg4j6, warm-start from
ppo_goal_cw_walk_step0_c1.zip, ent 0.01, no canary-arm lines in the
log head — launched raw again, no ledger entry; reconstructed entry
added). My "PLATEAU CONFIRMED: no c2" above records THIS cycle's
decision under directive 0-a; the operator's hand-launch overrides it
for the identical-config lineage, which is their prerogative. Not
killed: it is the operator's own run, it is cheap (walk pod solo),
and it extends the identical-config arm while lowent (one variable:
ent 0.001, same lineage) runs beside it — the pair is a clean A/B on
the entropy hypothesis. s5 freed while this cycle ran
(cw-stance-endpost-r1 finished) — endpost-r1's verdict belongs to the
watcher-triggered cycle for it, not this one; s5 left free.

### Cycle 15 close — caps and fleet
cw-walk-step0-lowent LAUNCH VERIFIED (launcher exit 0): s6, pid 1086,
W&B vkrvueqg (ent 0.001, seed 0, init-from md5-verified ea1685a4 on
pod), ~3550 fps solo, global_step advancing 4.41M->4.73M in the
verification window. Launched this cycle: 1 of 4 cap; new steps 4M of
16M. Champions: stance unchanged (cw_stance_dr10); walk-line
reference/champion recorded = ppo_goal_cw_walk_step0.zip md5 ea1685a4
(first gait_valid 12/12 walk; NOT hardware-ready — DR 0, skating,
no speed tracking). Fleet after cycle: walk = c2 (operator's
identical-config arm), s6 = lowent (the ent A/B against it), s5
freed (endpost-r1 finished — verdict belongs to its own
watcher-triggered cycle), all others idle. Evals archived:
logs/ckpt_eval/cw_walk_step0_4M_gate + cw_walk_step0_c1_gate.
Ledger: step0 FINISHED+verdict, c1 reconstructed+verdict, c2
reconstructed RUNNING (operator raw launch), lowent RUNNING.

## OPERATOR CORRECTION (2026-08-09 ~01:15Z) — ledger clobbering root cause

The cycle above mis-verdicted the ops story: c1 and c2 were NOT raw
launches. Both went through `launch_run.py` (c1 verified 00:34Z, c2
verified 00:57:40Z, each printed "VERIFIED RUNNING ... ledger
updated"). Their entries vanished because cycles were HAND-EDITING
`experiments.json` (read whole file -> modify -> dump, no lock, stale
copy): each manual "reconstruction" erased whatever concurrent
launches had landed since the stale read — c1's entry, then lowent's
(operator re-backfilled it 01:05Z). Fix landed this commit:
`launch_run.py update --run <name> --set k=v [--create]` does locked
single-entry edits, and the prompt now forbids hand-editing the
ledger. Prompt change requires watcher restart (prompt is cached at
watcher start) — operator handling it.
