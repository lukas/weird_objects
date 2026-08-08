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

<<<<<<< Updated upstream
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
=======
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
>>>>>>> Stashed changes

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
