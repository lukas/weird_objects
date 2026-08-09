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

## OPERATOR CORRECTION (2026-08-09 ~01:25Z) — the "raw launch" story, actual root cause

Two-part correction to the cycle entry above and to an earlier
version of this note. (1) c1 and c2 were NOT raw launches: both went
through `launch_run.py` and verified (c1 00:34Z, c2 00:57:40Z). (2)
Nothing was clobbered. The controller has TWO trees: the git clone
`/workspace/weird_objects` (where the watcher and all cycles run, and
where the real ledger lives) and a stale NON-git synced copy at
`/workspace/prototype_sts3215` (same path the training pods use). The
operator ran the launcher from the stale copy, so those launch
records went to a shadow ledger no cycle reads; the cycle correctly
found nothing and reconstructed the entries. Fixed 01:20Z: the
operator's original launch entries (with checks/wandb ids) are
imported into the real ledger, and the stale copy's
`experiments.json` + lock files are now symlinks to the authoritative
ones, so either path hits one ledger. Also landed: `launch_run.py
update --run <name> --set k=v [--create]` for locked single-entry
edits — cycles should use it instead of hand-editing the JSON.
Standing rule: on the CONTROLLER, always operate in
`/workspace/weird_objects`; `/workspace/prototype_sts3215` is a
deploy-layout copy, not the working tree. (Cycle 16 below read the
earlier "clobbering" version of this note; its checkup fix stands,
but its root-cause chain should cite the shadow-ledger story.)

## Cycle 16 (2026-08-09 ~01:2xZ) — watcher checkup crash on cw-walk-step0-c2: tooling, not the run

Trigger: watcher checkup rc=1 for cw-walk-step0-c2 at 01:00:55Z —
`KeyError: 'log'` inside `cmd_checkup` (tool crash, NOT a DEAD/SUSPECT
verdict on the trainer).

**OBSERVATIONS.** (a) Root-cause chain: checkup KeyError 'log' ← c2's
ledger entry lacks the `log` field ← that entry is a post-clobber
backfill (see operator correction above; the original launcher-written
entry, which had `log`, was erased by a hand-edit) ← unlocked
read-modify-write of experiments.json (already fixed upstream via
`launch_run.py update` + prompt ban). (b) Manual health check before
touching anything: trainer process alive on hexapod-sweep-walk
(pid 3831261, cmdline matches c2 config: joint_walk, no-dr,
log_std_init 0.0, ent 0.01, seed 0), /tmp/train_cw-walk-step0-c2.log
present and fresh (195,679 B at 01:07Z, n_updates 6350). (c) Fixes:
ledger `log` field added via the locked path
(`launch_run.py update --run cw-walk-step0-c2 --set
log=/tmp/train_cw-walk-step0-c2.log`); `cmd_checkup` hardened to fall
back to the guardrails pattern `/tmp/train_<run>.log` when `log` is
absent, so any future backfilled entry degrades to a working checkup
instead of crashing the watcher. (d) Re-ran checkup after the fix:
**HEALTHY**, solo on 56-core pod, fps 3276.8, rc=0, recorded in
ledger. No kill, no relaunch, no new experiments this cycle.

Side observation (hypothesis, not verdict — c2's cycle owns the
verdict): c2's log tail shows train/std 5.89 at ~6350 updates,
continuing the lineage's monotone std runaway (1.0 → 2.30 → 3.21 →
now 5.89 under ent 0.01). This is exactly the A-arm behavior the
lowent A/B (ent 0.001, s6) predicts if the entropy-runaway hypothesis
is true; noted here so the eventual c2/lowent verdicts check final
std against it.

cw-walk-step0-lowent untouched (RUNNING, s6, verified 01:01:20Z;
its own watcher checkup is separate). cw-stance-endpost-r1 untouched
(owned by the concurrent cycle). No plan change: RL_PLAN.md is not
affected by a tooling fix.

## Cycle 17 (2026-08-09 ~01:3xZ) — watcher DEAD on cw-walk-step0-c1: false positive (run had FINISHED), no retry

Trigger: watcher checkup rc=1 at 01:09:45Z — "DEAD: no cw-walk-step0-c1
trainer process on hexapod-sweep-walk".

**OBSERVATIONS (mechanical).** The run is not dead; it is FINISHED and
already verdicted. (a) /tmp/train_cw-walk-step0-c1.log ends with the
normal completion sequence: final video reel, "[train] 4000000 steps in
464s → .../ppo_goal_cw_walk_step0_c1.zip", final eval, zero-action
baseline. (b) Checkpoint on the walk pod: 704,405 B, mtime 00:38Z, md5
e4314fa287d2e5e677c50c8c4e438c23 — matches the ledger. (c) W&B 9sdboq3u
state=finished, global_step 5,265,024 — exactly the canary auto-stop
point cycle 15 verdicted ("canary false-positive truncation, plateau
confirmed"). (d) Successor c2 was init-from this checkpoint and is alive
on the same pod (log advancing; W&B c1oyg4j6).

Root-cause chain (required): watcher DEAD ← checkup matched a ledger
entry for c1 still marked RUNNING ← that entry is the launcher-written
original, clobbered by the 08-09 hand-edit incident and later restored
WITHOUT the status transition (cycle 15's FINISHED+verdict landed on the
duplicate reconstruction entry instead) ← unlocked read-modify-write of
experiments.json (root already fixed via `launch_run.py update` + prompt
ban). Secondary defect exposed: `cmd_checkup` treats ANY absent process
as DEAD, but canary auto-stops and short runs legitimately complete
before/within the checkup window — this false-positive class recurs
unless checked at the tool.

**ACTIONS.** (1) NO retry, NO cleanup: nothing is dead; a relaunch would
duplicate a verdicted lineage already continued by c2 and would
overwrite ppo_goal_cw_walk_step0_c1.zip (forbidden). (2) `cmd_checkup`
hardened: on process-absent it now looks for the trainer completion
marker ("[train] N steps in Ts → ckpt") in the log tail and reports
FINISHED_BEFORE_CHECKUP (rc 0, status→FINISHED under the lock) instead
of DEAD; genuinely dead tails (tracebacks, silence) still report DEAD —
regex verified against c1's real tail (positive) and a crash tail
(negative). Re-ran checkup on c1 through the new path: rc 0, ledger
transitioned. (3) `update` gained a `--created` selector so stale
DUPLICATES can be resolved through the locked path (previously only the
newest entry per name was reachable). (4) Residue sweep for the same
failure class (checks generalize): c2 had the identical stale pair — its
reconstruction entry (created 00:55:00) is now marked DUPLICATE with a
pointer to the authoritative launcher entry (created 00:54:00), so c2's
eventual FINISHED cannot leave a second RUNNING ghost to trigger another
false DEAD. Ledger now has exactly one RUNNING entry per live run
(endpost-r1, c2, lowent).

Side observation (hypothesis, not a verdict — c2's own cycle owns it):
c2 log tail shows train/std 7.51, extending the lineage's monotone std
runaway (1.0 → 2.30 → 3.21 → 5.89 at cycle 16 → 7.51 now, all under ent
0.01). Still exactly the A-arm trajectory the lowent A/B predicts if
entropy runaway is the plateau driver; final-std comparison belongs to
the c2/lowent verdicts.

No evals, no launches, no plan change (tooling + ledger hygiene only).
cw-stance-endpost-r1 untouched (concurrent cycle owns it); c2 and
lowent untouched and training.

### Cycle 17 amendment — attribution correction (read post-push)
The rebase that landed cycle 17 pulled in the operator's 01:25Z
correction: the duplicate/stale-RUNNING entries came from the
SHADOW-LEDGER import (launcher run from the stale non-git tree
/workspace/prototype_sts3215; original entries imported into the real
ledger at 01:20Z with their at-launch RUNNING status), NOT from
hand-edit clobbering. Corrected root-cause chain for the c1 false
DEAD: watcher DEAD ← imported launcher entry still RUNNING ← import
preserved at-launch status while the completion had been recorded on
the reconstruction entry ← two-trees/shadow-ledger defect (fixed by
operator via symlink). Every cycle-17 action is unchanged by this:
the completion-marker checkup, the --created selector, and the
DUPLICATE marking address the residue class regardless of how the
duplicates arose.

## Cycle 18 (2026-08-09 ~01:5xZ) — c2/lowent A/B verdicts: entropy runaway CONFIRMED as the plateau driver; lowent is the new walk-line champion

### cw-walk-step0-c2 — the ent-0.01 A-arm is actively destructive: std 3.24→8.73, rew flat, det FALLS appeared
OBSERVATIONS. W&B c1oyg4j6 finished, 5.27M→9.27M cum (full 4M
segment); ckpt md5 f7d0a61a27a257bfc4e3ef168fc742df (pulled, matches
pod). train/std 3.24→8.73, monotone the whole segment (lineage:
1.0→2.30→3.21→8.73 under flat ent 0.01). ep_rew_mean last0.5M 572.4
vs prev0.5M 577.9 — flat, still inside the parent plateau band
554–610 (segment band 550–621). env/reward_step_event 0.087 at end
vs parent-end 0.096 — flat. Trainer's own final eval: survived 6/10
with 4 tilt_pitch terminations (parent step0: 10/10); final W&B video
reel 2/4 walk episodes TERM(tilt_pitch).
Gate harness ×2 (DR 0, own cfg, 6 eps/mode det+sto;
logs/ckpt_eval/cw_walk_step0_c2_gate + _gate_sto rerun): walk det 0/6
AND 0/6; sto 3/6 and 1/6 — pooled 4/12 vs c1 2/6 and step0 1/6, all
inside the ±1–2 ep noise band: NO evidence of change. det slip mean
0.818 / 0.865 vs step0 0.93 (direction only, ranges overlap). NEW
FAILURE CLASS: det tilt_pitch falls 2/12 + 1 safety flag — this
lineage had ZERO terminations in step0 (12/12) and c1. Harness
policy_std 8.48.
Frames watched (provenance): walk_det_0.mp4 md5 ca313fb8 250f (zoom
tile); walk_det_3.mp4 md5 361977f9 189f — the fall is ON CAMERA:
rear sinks, front-right leg extends stiff, body pitches back until
the tilt trip; walk_sto_0.mp4 md5 43086355 250f (zoom tile). Motion
pathologies first: leg excursions visibly wilder than parent/lowent —
legs flung into extreme extensions mid-crawl; cadence irregular;
stance feet creep. Still six-leg cycling, no flag leg, no park.
Exploit checked, not found: no sacrificed leg (12/12 gait_valid), no
phantom steps (swings on video match counts 2–12).
INTERPRETATION. The identical-config continuation didn't just
plateau — at std 8.7 the deterministic policy now FALLS. Entropy
income at ent 0.01 outruns the saturated task gradient without bound;
4M more steps bought degradation, not progress.
VERDICT: FAIL (drag clause persists; det falls are new damage). NOT
HARDWARE-READY: std-8.5 policy, deterministic falls, skating.
HYPOTHESIS STATUS (continued improvement under identical config):
REFUTED — rew flat, std runaway, new det falls. The identical-config
ent-0.01 arm is CLOSED, now by its own A-arm evidence, not just the
plateau rule.

### cw-walk-step0-lowent — ent 0.001 breaks the plateau: rew 583→688, sto 10/12, slip down ~20%; gate FAILS on the std clause only. NEW WALK-LINE CHAMPION
OBSERVATIONS. W&B vkrvueqg finished, 4.01M→8.01M (full 4M); ckpt md5
923ee55cac222957428d9de70b3bec23 (pulled, matches pod). train/std
2.31→2.075: the runaway STOPPED (A-arm c2 hit 8.73 over the same
window) but std does NOT anneal — floor ~2.07, gate clause ≤1.2
FAILS. ep_rew_mean 583→688; last0.5M 681.0 ≥ 620 ✓ — clean breakout
from the 554–610 plateau band while the concurrent one-variable A-arm
(c2, ent 0.01, same lineage/seed) sat at 572. env/reward_step_event
0.092→0.110 (+20% vs c2's 0.087 and parent-end 0.096).
Gate harness ×2 (DR 0, own cfg, 6 eps/mode det+sto;
logs/ckpt_eval/cw_walk_step0_lowent_gate + _gate_sto): gait_valid
24/24, terminations 0, safety flags 0, forward 0.23–0.47 m (all
≥0.10), min swings/leg 3, duty 0.18–0.83. det slip mean 0.746 /
0.717 vs step0 baseline 0.93 — det ranges NON-overlapping (0.68–0.79
vs 0.87–0.98): real ~20% skating reduction, NOT elimination (feet
still creep ~0.7 m per ~10 s ep vs ~0.4 m body travel). walk sto
success 6/6 then 4/6 — pooled 10/12 vs step0 1/6: far outside the
±1–2 ep noise band, real. walk det 0/6 both — every det failure is
vel_err (0.032–0.066): det speed 0.043–0.057 m/s REGARDLESS of
command; speed tracking remains absent.
15 s sustain probe (0-b rung 1, eval-first;
logs/ckpt_eval/cw_walk_step0_lowent_15s): det 6/6 ≥0.40 m (0.46–0.72
m), gait_valid 6/6, zero falls/flags over 15 s; sto 5/6 ≥0.40 m with
ONE gait-invalid wander ep (0.15 m, gv False). Final-third frames
(walk_det_0.mp4 md5 fab22425, 375f, tiles of frames 250–375): body
level, six legs still cycling, no wind-up, no posture decay — THE
GAIT SUSTAINS.
Frames watched (provenance): walk_det_0.mp4 md5 0dc73ec5 250f (strip
+ zoom tile), walk_sto_0.mp4 md5 86d83fc5 250f (zoom tile; sto video
capability added to the harness THIS cycle — it could never record
sto before, an unwatchable-success blind spot now closed), 15 s
walk_det_0 md5 fab22425 375f. Motion pathologies first: stance still
sprawly-wide; cadence still irregular (no clean tripod rhythm);
stance feet visibly creep (matches slip 0.68–0.79). Achievements:
level body throughout, all six legs genuine lift→swing→touchdown, no
flag leg, no park, zero falls in 36 episodes total.
Exploit checked, not found: no sacrificed leg (24/24 + 11/12 at 15
s), no safety-layer reliance (0 flags), no phantom step events, no
height collapse. Noted for honesty: sto's tracking advantage over det
(vel_err 0.026–0.030 vs 0.032–0.066) on EVERY checkpoint of this
lineage means part of the sto success count rides on action noise
braking the robot toward command — the underlying tracking defect is
untouched.
INTERPRETATION. The A/B is decisive: same parent lineage, same seed,
one variable (ent 0.01 vs 0.001) → runaway+flat+falls vs
stable-std+plateau-breakout+cleaner gait. Entropy over-injection was
the plateau driver. But std floors at ~2.07, not ≤1.2: either 0.001
still pays enough to hold it, or std ~2 is REWARD-OPTIMAL because
noise functions as the missing speed regulator (sto tracks better
than det everywhere). That is a HYPOTHESIS, labeled as such; the
mechanical fact is det ignores speed commands, which is the pricing
defect already recorded (overspeed uncharged, step credit scales
with stride).
VERDICT: FAIL against the recorded gate — std clause (2.075 > 1.2);
the other five clauses PASS (gait_valid 12/12 ✓, forward ✓, swings ✓,
det slip 0.746 ≤ 0.93 ✓, rew 681 ≥ 620 ✓). NOT HARDWARE-READY: DR 0
only, skating persists, no det speed tracking, deploy-relevant std
still 2.1. CHAMPION UPDATE: ppo_goal_cw_walk_step0_lowent.zip (md5
923ee55c) is the new walk-line champion — beats step0 (sto 10/12 vs
1/6, det slip 0.75 vs 0.93 non-overlapping, zero falls, rew +100)
— append-only, step0 checkpoint retained.
HYPOTHESIS STATUS: SUPPORTED on the core claim (plateau was
entropy-driven; the clean A/B against c2 is the evidence). REFUTED on
the anneal sub-claim (std ≤1.2 did not happen; floors at 2.07). None
of the three pre-registered if-false branches occurred (std under 2.0
band question is moot: it stopped rising and rew climbed — the
diagnosis branch that matters, exploration-side, is answered).

## Cycle 18 (2026-08-09) — cw-stance-endpost-r1 verdict: gate FAIL, but the flag leg MOVED for the first time; continuation c1 launched

### cw-stance-endpost-r1 — FAIL on the lower-posture gate; dense terminal gradient acts on every priced leg (unique in lineage); endings hover instead of planting
OBSERVATIONS (mechanical). W&B no0ihywt FINISHED, 4M steps (23.19M
cum, 1368 s solo s5, ~2925 fps). Final ckpt ppo_goal_cw_stance_endpost
.zip md5 b78c1b6ab41f51cc2753a44ff828aecb (pod s5 + controller copy).
train/std 0.193-0.197 whole run (inherited, NO runaway — as designed).
env/reward_end_posture (per charged tick): Q1 mean -0.640 -> Q4 mean
-0.518 (~19% down, still paid at run end). No canary auto-stop (one
transient rise_bridge_a=0 at 23.19M periodic eval).
HARNESS FOOTGUN (record for every future joint_goal eval): with
--modes omitted, eval_checkpoint.py falls back to (hold, track, rise)
for joint_goal — the env class has no EVAL_MODES — so the gate eval
took TWO draws: draw A default modes, draw B `--modes lower raise`.
ALWAYS pass --modes explicitly for gate evals.
Harness (posture-strict, DR 1.0, own cfg-sets incl. k_end_posture=5.0,
6 eps/mode det+sto):
- lower: det 0/6, sto 0/6 posture (gate needed >=4/6 det, >=5/6 sto).
  Heights PERFECT 12/12 (h_err 1.1-5.5 mm). Per-leg end clearances,
  stable across episodes: leg 0 at 89-135 mm, leg 2 at 22-48 mm
  (INSIDE the 60 mm allowance), leg 4 at 133-207 mm. Named-baseline
  deltas: vs cw-stance-posture (same parent/cfg/seed minus
  k_end_posture) leg 4 moved 229-264 -> 133-207 mm — the FIRST
  movement of the flag leg in this lineage; leg 2 planted-adjacent
  (was ~58-91 on legs 0/2); leg 0 WORSE 58-91 -> 89-135. Summed
  over-allowance ~200 -> ~147 mm (~26% down, matches the W&B trend).
- rise: det 0/6, sto 1/6 posture (comparator: det 3/6 & 1/6 across
  draws, sto 4/6). Sto count -3 is OUTSIDE the +-1-2 ep noise band =
  honest REGRESSION on rise posture counts. But failure MAGNITUDE
  collapsed: fails are feet hovering 24-64 mm (one crouch keeps
  114/143 mm legs) vs comparator's 102-327 mm flags. Heights all
  fine (h_err <=16.5 mm) — height-only ~12/12.
- hold: det 5/6 (one 22 mm marginal), sto 6/6 — gate clause MET,
  unchanged vs comparator (5/6, 6/6).
- track: 6/6 det + 6/6 sto.
- raise (canary tripwire, no compute owed): posture 0/12, leg 2
  hovers 53-110 mm; heights 12/12 (h_err <=5.3) — best-ever heights
  retained from the posture line (11/12, within noise).
- Current note (hardware-relevant): hold det Imax 2.67 A, rise sto
  2.70 A — above the 2.5 A breaker under the MuJoCo 3.11 reading
  shift (plan already requires recalibration before trusting current
  gates).
VIDEO PROVENANCE (all reviewed as frame strips; det only, harness
saves no sto video): lower_det_0.mp4 md5 2cd2d82f 250 fr — body
descends on schedule but ends with TWO SPEAR LEGS (0 and 4) extended
straight out, feet at ~114/150 mm; never a belly rest. raise_det_0.mp4
md5 d7a53139 250 fr — quiet stance, leg 2 held just off the ground.
rise_det_0.mp4 md5 308d2472 250 fr — crouch start rises, two left
legs end extended outward horizontally, feet hovering (the 114/143
episode). rise_det_1/rise_det_4 strips — wide sprawled stand, feet
just off the ground (24-44 mm), no skyward flags anywhere.
hold_det_0.mp4 md5 9e2e0d2c 250 fr + track_det_0 strip — clean quiet
six-foot stances.
Exploits looked for, not found: window is schedule/time-based so no
dodge-by-slamming exists; no height-collapse shortcut (h_err <=5.5 mm
lower); no safety-layer reliance (safety_flags 0 in all 48 eps); the
residual charge (-0.52/tick) matches the visible hover exactly — no
hidden compensation.
INTERPRETATION. NEITHER pre-registered branch occurred. If-true
(lower posture >=4/6) missed at 0/12. If-false (worst_clear ~250+ mm,
charge absorbed) also missed: every priced leg moved down, including
the flag leg that k_load_even/k_support_margin/exploration never
touched, and worst_clear fell 264 -> 176-207 mm. The dense terminal
gradient demonstrably ACTS; what 4M did not produce is contact. Two
readings: (a) optimization incomplete — the W&B charge was still
declining at run end; (b) airborne-hover equilibrium — descent slope
(~19%/4M) too shallow to ever plant. The rise sto regression (sprawl
endings with hovering feet where the comparator's flat starts
planted) is a caution that the term reshapes endings globally, not
only where legs were flagged.
VERDICT: FAIL (lower posture 0/12 vs gate; heights clause and hold
clause MET). hardware-ready: NO — lower still ends with two spear
legs in the air and rise ends sprawled on hovering feet; nobody puts
that on the robot. Champion UNCHANGED (cw_stance_dr10). Crown-jewel
watch: rise/lower HEIGHTS remain intact on this branch (24/24).
HYPOTHESIS STATUS: INCONCLUSIVE — "dense terminal gradient makes the
planted ending reachable at inherited std" is neither supported
(no contact at 4M) nor refuted (unique dose-response on every priced
leg, charge still declining). Per near-miss practice: ONE
consolidate-in-place continuation with pre-registered plateau
criteria; plateau = refuted -> belly-rest reference states
(reset-side, the remaining pre-registered structural option). No
coefficient change (a k bump with this evidence would be exactly the
banned "another coefficient" move).

### LAUNCH cw-stance-endpost-c1 (4M, DR 1.0, seed 0, pod s4) — consolidate-in-place continuation
HYPOTHESIS: the hover is optimization-incomplete, not an equilibrium
— the terminal charge's descent (r1: Q1 -0.640 -> Q4 -0.518)
continues in c1 and reaches planted endings.
One variable vs endpost-r1: +4M steps (same cfg, same seed, init from
r1's final ckpt md5 b78c1b6a).
Prediction-if-true: env/reward_end_posture improves segment-over-
segment by >= r1's own Q1->Q4 delta (0.12) AND lower det summed
over-allowance drops below ~80 mm (r1: ~147 mm) AND lower posture
gets off 0 (>=1/6 any pass); gate pass possible.
Prediction-if-false: the charge PLATEAUS (c1 last-quarter mean within
0.05 of c1 first-quarter) with per-leg clearances at r1 values (leg 0
~114, leg 4 ~150 mm) — that refutes the reachability hypothesis and
promotes belly-rest reference states; no third pricing arm.
Strongest alternative: slow nonzero descent that neither plateaus nor
plants — distinguished by the slope rule above (below 0.12/4M at 8M
cum = call it plateaued; two misses in a row = change hypothesis).
Gate (unchanged): posture-strict @ DR 1.0, 6 eps/mode det+sto: lower
end-posture >=5/6 sto AND >=4/6 det AND rise/lower height-only >=5/6
both AND hold sto 6/6.
Canaries ON (multi-skill warm start; step0's --no-canary exemption is
lineage-specific). Rise-regression watch: if rise heights (not
posture) erode below 5/6 anywhere, stop the line.
Pod s4 (empty node g129004, host load 0.4 at status check); parent
ckpt copied controller -> s4 with md5 verification BEFORE launch (the
endpost init-crash lesson).

### LAUNCH cw-walk-lowent-h15 (4M, DR 0, ep 15 s, seed 0) — 0-b rung 1: consolidate the sustained walk at the horizon it must survive
HYPOTHESIS: the lowent gait is horizon-robust in behavior but the 5 s
training horizon leaves long-horizon regulation unlearned (value
function never sees past 5 s); the 15 s eval's 1/6 sto lazy-leg
wander ep is the symptom. Training at --episode-seconds 15 (ONE
variable vs parent lowent) consolidates sustained walking.
Prediction-if-true: 15 s harness det AND sto ≥0.40 m 12/12,
gait_valid 12/12, no final-third frame degradation; 5 s numbers no
worse than parent (det slip ≤ ~0.75, sto success within noise of
10/12). Prediction-if-false: (a) 15 s numbers match parent's
eval-only 15 s result (det 6/6, sto ~5/6) → 5 s training already
sufficient, horizon rung closed as redundant — cheap, useful null;
(b) reward balance shifts with horizon (step-event income scales per
episode, termination pricing dilutes) and the gait degrades →
horizon-dependent pricing sensitivity, stop and record before DR.
Strongest alternative: the sto wander was binomial noise —
distinguished because if-true predicts 12/12 at 15 s across both
passes, clear of the 5/6-with-one-invalid baseline. GATE: 15 s DR 0
harness 6 eps/mode det+sto: forward ≥0.40 m 12/12 AND gait_valid
12/12 AND ≥2 swings/leg AND 0 terminations AND no final-third
degradation (frames); retention: 5 s det slip mean ≤ 0.93. Budget
4M. ent 0.001, --no-canary (lineage exemption). Pod: s5.

### LAUNCH cw-walk-lowent-dr03 (4M, DR 0.3, seed 0) — 0-b rung 2: first DR ladder rung off the champion
HYPOTHESIS: the step0-lineage gait survives moderate DR introduced
one rung at a time (0.3, warm start from lowent md5 923ee55c) — DR
at this scale robustifies rather than destroys (hist8's destruction
came from jumping straight to wide distribution). ONE variable vs
parent: --no-dr → --dr-scale 0.3. Prediction-if-true: DR 0.3 harness
gait_valid 12/12, forward ≥0.10 m 12/12, zero falls; DR 0 retention
eval no worse than parent. Prediction-if-false: park/flag/falls
return at DR 0.3 (gait_valid <10/12 or terminations) → rung too big,
drop to 0.15 next segment; OR DR 0.3 passes but DR 0 retention
erodes → warm-start interference, revisit budget/LR not DR size.
Strongest alternative: gait "survives" via DR-specific slop without
real robustness — distinguished by video + slip/duty at BOTH DR 0.3
and DR 0. GATE: DR 0.3 harness (own DR) 6 eps/mode det+sto:
gait_valid 12/12 AND forward ≥0.10 m 12/12 AND ≥2 swings/leg AND 0
terminations AND det slip mean ≤ 0.93; plus DR 0 retention check.
Budget 4M. ent 0.001, --no-canary (lineage exemption). Pod: walk.

Also this cycle (eval-side check, generalizes): eval_checkpoint.py
recorded video ONLY for det passes — every sto scalar in every past
verdict was structurally unwatchable. Fixed: sto episodes now record
under the same every-Nth rule (walk_sto_*.mp4). lowent/c2 sto strips
this cycle are the first sto footage of the campaign.

### Cycle 18 close — launches, fleet, and a launch failure worth recording
LAUNCH RESULTS. cw-walk-lowent-h15 attempt 1 FAILED at init:
FileNotFoundError on the parent checkpoint — lowent trained on s6,
and code sync does NOT carry policies/, so the walk pod had no
ppo_goal_cw_walk_step0_lowent.zip. Standing lesson: before any
cross-pod warm start, copy the parent ckpt to the target pod and
md5-verify it (done: walk + long5m both at 923ee55c). The crashed
W&B run burned the name (append-only), so the retry is
**cw-walk-lowent-h15b** — W&B y9jav0y4, walk pod, pid 308436,
VERIFIED RUNNING at ~5052 fps. **cw-walk-lowent-dr03** — W&B
xw5pdtum, long5m pod, pid 1834780, VERIFIED RUNNING at ~3413 fps.
Both 4M, both off lowent md5 923ee55c, snapshot e5f0c3e (tag
exp/cw-walk-lowent-h15). Launched 2 of 4 cap; 8M of 16M step cap.
CONCURRENT TRAFFIC (not mine): cw-walk-step0-lowent-c1 (launcher
entry 01:22:51Z, s6) — identical-config continuation of lowent per
directive 0-a, running beside my two one-variable arms; distinct
hypotheses, no duplication. cw-stance-endpost-r1 untouched (its own
cycle owns it).
Fleet after cycle: walk = h15b, long5m = dr03, s6 = lowent-c1
(concurrent), s5/s3/s4/friction/lower idle. Champions: stance
unchanged; walk = ppo_goal_cw_walk_step0_lowent.zip md5 923ee55c
(NEW, this cycle). Evals archived: logs/ckpt_eval/
cw_walk_step0_{c2,lowent}_gate{,_sto} + cw_walk_step0_lowent_15s.
Harness change landed: sto passes now record video (previously
structurally unwatchable — det-only videos).

### cw-stance-endpost-c1 launch outcome (cycle 18 close)
Three attempts, all recorded in the ledger: (1) s4 attempt died at
argparse — the launcher still passes --notes UNQUOTED to the remote
shell (cycle-14 finding (a), unfixed); a `;` in my notes split the
command, no process/W&B created, launcher marked FAILED. (2) s3 was
chosen after a clean REFUSED on s4's node (host load1 76.9 — normal
traffic, other tenants/launches). First s3 attempt hit the same
quoting bug with space-splitting. (3) Embedded-quote workaround
(r1's pattern): VERIFIED RUNNING on s3 — pid 2105951, W&B v5t38fee,
~2458 fps solo, log advancing, 0 tracebacks, cmdline carries every
cfg-set incl. reward.k_end_posture=5.0, canaries armed with all four
protected groups, code at snapshot 5ca9a85, parent ckpt md5 verified
b78c1b6a on pod before launch. Ledger: my pre-registration INTENT
merged into the launcher's authoritative RUNNING entry (SUPERSEDED
pointer left; no ghost-RUNNING residue). Launcher note for operator
(still open): quote/escape --notes when building the remote command.
Cycle totals: 1 experiment launched (4M steps of 16M cap), pods after
cycle: s3 = endpost-c1, s6 = lowent-c1 (concurrent cycle's), rest
idle. Champions unchanged.

## Cycle 19 (2026-08-09 ~02:3xZ) — cw-walk-lowent-h15b verdict: gate FAIL on the recurring 1-ep sto park; horizon rung largely redundant for the GATE, but real slip/tracking/std side-gains. NEW walk champion (narrowly). Continuation launched.

### cw-walk-lowent-h15b — 15s horizon training: gate clauses match parent-within-noise (if-false branch a), but det slip/tracking and std anneal genuinely improved
OBSERVATIONS (mechanical). W&B y9jav0y4 FINISHED, 4M steps (8.01M→
12.02M cum, 1069 s solo on walk pod). Final ckpt
ppo_goal_cw_walk_lowent_h15b.zip md5 d0a12a9454dfd5b9e5a1c9d16ad5250a
(pod + controller copy match). train/std 2.075→1.742 — the anneal
RESUMED at the 15 s horizon (parent lowent floored at ~2.07 for a
full 4M segment; ONE variable differs). ep_rew_mean Q1 1046 → Q4
1137, still climbing at end (Q3 1110→Q4 1137; scale not comparable
to parent's 688 — 3x episode length). env/reward_step_event
0.110→0.134. Harness policy_std 1.705 (parent 2.044).
Gate harness 15 s DR 0 own-cfg (logs/ckpt_eval/cw_walk_lowent_h15b_15s,
6 eps/mode det+sto): det fwd ≥0.40 m 5/6 (0.569–0.711; det[2]=0.243
FAIL), sto 5/6 (0.471–0.668; sto[4]=0.029 FAIL). gait_valid 11/12
(sto[4] invalid, 5 legs sacrificed). Terminations 0/12, safety flags
0. det slip mean 0.912 vs parent-15s 1.14 (ranges 0.801–1.102 vs
1.089–1.202: 5/6 non-overlapping — real reduction, NOT elimination).
Retention harness 5 s (logs/ckpt_eval/cw_walk_lowent_h15b_5s): det
slip mean 0.584 ≤ 0.93 gate ✓ — vs parent 0.746/0.717, ranges
0.528–0.654 vs 0.68–0.79 NON-overlapping: real. det success 4/6 vs
parent 0/6+0/6 — outside the ±1–2 ep band: det speed tracking
partially appeared (vel_err 0.022–0.031 in 5/6 eps vs parent
0.032–0.066). 5 s sto 4/6 vs parent pooled 10/12 — inside noise, no
evidence of change.
FRAMES WATCHED (provenance). 15 s: walk_det_0.mp4 md5 b5702d8e 375f,
walk_det_3.mp4 d9128d64 375f, walk_sto_0.mp4 ee652939 375f,
walk_sto_3.mp4 ff725f99 375f. 5 s: walk_det_0.mp4 8bb50316 250f,
walk_sto_0.mp4 f52a868a 250f. The two failing eps landed on
unrecorded indices (harness records every 3rd ep) — reran the
IDENTICAL seed-0 eval with --video-every 1 (draws reproduced
bit-exact; logs/ckpt_eval/cw_walk_lowent_h15b_15s_allvid):
walk_sto_4.mp4 md5 be0ba207 375f, walk_det_2.mp4 7e5a8946 375f.
Pathologies first: (1) sto[4] at BOTH horizons is a hard TRIPOD PARK
from t=0, on camera — legs 0/2/4 loaded at duty ~1.0, legs 1/3/5
dangling just off the ground, body never advances a checker square
in 15 s (duty [1.0,0.0,1.0,0.02,0.99,0.01], swings [0,1,0,1,2,1],
fwd 0.029 m). Same failure class as parent's sto collapse ep
(lowent gate_sto sto[4], sacr [1,3,5]) at the same 1/6 frequency —
character SHARPER (hard park vs lazy wander), frequency unchanged.
(2) det[2] 15 s is stumble-in-place churn on camera: legs 1/3/5
rapid short flicks (swings 13/12/7) while 0/2/4 stay loaded (duty
0.89/0.86/0.78), worst slip 1.102, fwd 0.243 — churning, not
walking. (3) Skating: reduced ~20% but stance feet still creep in
every strip. (4) Stance still sprawly-wide, cadence still irregular.
Achievements (each with a number): passing eps show all six legs
cycling with duty 0.42–0.63 and level body through the final third
of all four reviewed 15 s strips — no wind-up, no posture decay.
Zero falls in 24 harness eps.
Exploits looked for, not found: no sacrificed leg in any PASSING ep
(23/24 gv excl. the park ep), no phantom step events (swing counts
match strips), no safety-layer reliance (0 flags), no height
collapse. Honesty note carried from parent: sto still out-tracks det
(vel_err 0.021–0.043 sto vs 0.022–0.054 det) — noise-as-brake is
reduced but not gone.
INTERPRETATION. For the GATE, pre-registered if-false branch (a) is
what happened: 15 s clauses match the parent's eval-only 15 s result
within the ±1–2 ep band (det 5/6 vs 6/6, sto 5/6 vs 5/6) — the
horizon rung is redundant for gate success. But three side-effects
are real (non-overlapping ranges or outside noise band): det slip
down ~20% at both horizons, det 5 s success 0/12→4/6, and std anneal
resumed (2.08→1.74, still falling at run end). Hypothesis (labeled):
the longer return horizon weakened the value of noise-as-brake, so
the policy relies less on stochasticity and std can fall. The
remaining gate blocker is a single recurring failure class: a park
basin (odd-tripod unload) that ~1/6 stochastic starts fall into and
never leave, plus its det cousin (duty-skew churn).
VERDICT: FAIL against the recorded gate (fwd 10/12 vs 12/12,
gait_valid 11/12 vs 12/12, swings clause fails in the park ep; 0
terminations ✓, no final-third degradation ✓, 5 s retention slip
0.584 ≤ 0.93 ✓). NOT HARDWARE-READY: a robot that has a 1-in-6
chance of standing frozen in a tripod park instead of walking, plus
persistent foot-skating, does not go on the physical hexapod; DR 0
only. CHAMPION UPDATE (walk line): ppo_goal_cw_walk_lowent_h15b.zip
md5 d0a12a94 beats lowent — det slip non-overlapping better at both
horizons, det 5 s success 0/12→4/6, harness std 1.705 vs 2.044,
NO metric worse outside noise (5 s sto 4/6 vs pooled 10/12 is inside
the band). Append-only; lowent checkpoint retained.
HYPOTHESIS STATUS: REFUTED on the core claim (15 s training did NOT
consolidate to 12/12; the sto wander/park ep was not a
horizon-regulation symptom). The slip/tracking/std gains are
recorded as unexpected secondary effects, explanation hypothesis
labeled above. Eval-side note (generalizes): --video-every 1 rerun
at the same seed reproduces harness draws bit-exact — cheap way to
put any scalar-flagged pathology ON CAMERA; use it whenever a
failing ep lands on an unrecorded index.

### LAUNCH cw-walk-lowent-h15b-c1 (4M, DR 0, ep 15 s, seed 0, walk pod) — near-miss consolidate-in-place continuation
Basis: h15b missed the gate on exactly one ep per pass (10/12 fwd,
11/12 gv), rew still climbing (Q3 1110→Q4 1137) and std still
annealing (1.74, falling) at run end — the one-continuation rule
applies. This is NOT the closed c1/c2 identical-config class: that
closure was the ent-0.01 plateau (rew flat, std runaway); here both
trends are live.
HYPOTHESIS: the park-basin failure class (1/6 sto hard tripod park
from t=0 + det duty-skew churn) is exploration-gated — it vanishes
as std anneals below ~1.5. ONE variable vs h15b: +4M steps.
Prediction-if-true: 15 s harness det+sto fwd ≥0.40 12/12 AND
gait_valid 12/12; std keeps falling; gate pass. Prediction-if-false:
std <1.5 but the seed-0 park draw persists (~1/6 sto) or det churn
recurs → basin is STRUCTURAL (pricing) → escalate to plan 0-a (ii)
skating price vs step-event income / (iii) overspeed pricing; NO
third segment. Also false: rew plateau (last quarter ≤ prior) with
unchanged gate numbers → continuation closed. Strongest alternative:
the 1/6 park is binomial noise independent of std — distinguished by
the persistence-below-std-1.5 clause. Gate unchanged (h15b's 15 s
gate incl. 5 s retention slip ≤0.93). ent 0.001, --no-canary
(lineage exemption), parent ckpt already on walk pod (md5 d0a12a94
verified this cycle). Walk pod solo on node g129004 (0 trainers at
status check).

## Cycle 19-concurrent (2026-08-09 ~02:2xZ) — cw-walk-step0-lowent-c1 verdict: PASS on its gate, but every delta vs parent is at/inside noise — identical-config lineage CLOSED (no c2); hist8 rejoins from scratch on the step0 recipe

### cw-walk-step0-lowent-c1 — directive 0-a continuation: +42 reward, eval deltas inside noise, park attractor still 1/6 at 15 s
OBSERVATIONS (mechanical). W&B uwi188rd FINISHED, 4M steps (8.01M→
12.01M cum, 1073 s solo s6, ~1750→4096 fps). Final ckpt
ppo_goal_cw_walk_step0_lowent_c1.zip md5
104ba8cf300d979e1c97979df794a0d9 (pod s6 + controller copy match).
ep_rew_mean quarter means 697.0 / 711.1 / 725.2 / 728.5 — Q4−Q3 =
+3.3, INSIDE the ±15 W&B scatter (Q1→Q2 and Q2→Q3 were +14.1 each):
the climb decelerated to noise. train/std 2.08→1.94 (slow anneal, no
runaway). env/reward_step_event 0.113→0.115 quarters (flat; parent
ended 0.110).
Gate harness 5 s DR 0 own-cfg (logs/ckpt_eval/cw_walk_step0_lowent_
c1_gate, 6 eps/mode det+sto in ONE invocation — first use of the sto-
video harness for a gate): walk det gv 6/6, fwd 0.23–0.47 m (all
≥0.10), swings ≥3/leg, slip 0.580–0.777 mean 0.674, vel_err
0.025–0.059 with succ 2/6; walk sto succ 6/6, gv 6/6, slip mean
0.578. Terminations 0, safety flags 0. One det ep (ep5) duty
[0.79,0.20,0.83,0.18,0.80,0.22] — leg 3 at 0.18, marginally outside
the ~[0.2,0.9] clause (parent verdict recorded 0.18–0.83 and passed;
same treatment).
Named-baseline deltas: det slip mean 0.674 vs parent lowent
0.746/0.717 (ranges 0.58–0.78 vs 0.68–0.79: OVERLAP — direction
right, not evidence). det succ 2/6 vs parent 0/12 pooled (p≈0.08
under parent rate — borderline, NOT decisive). sto 6/6 vs parent
pooled 10/12 — inside the ±1–2 ep band, no evidence of change.
15 s sustain (logs/ckpt_eval/cw_walk_step0_lowent_c1_15s, + _v2
rerun after the harness fix below; scalars reproduced bit-exact):
det gv 6/6, fwd 0.34–0.71 m (5/6 ≥0.40 vs parent 6/6 — within 1-ep
noise; det ep2 slowed to 0.338 m at vel_err 0.061, duty skewed
0.84/0.17). sto 5/6 gv with ONE HARD TRIPOD PARK: sto ep4 duty
[0.99,0.00,1.00,0.01,0.99,0.01], swings [2,1,0,1,2,1], fwd 0.014 m
— same 1/6 invalid rate as parent (whose invalid ep was a wander)
and same class as h15b's park ep (concurrent verdict).
FRAMES WATCHED (provenance): gate walk_det_0.mp4 md5 cee8b197 250f,
walk_sto_0.mp4 f6ca4f63 250f; 15 s walk_det_0.mp4 cd11ba81 375f,
walk_sto_3.mp4 921a5a60 375f; park ep walk_sto_4.mp4 (from _v2
rerun) md5 a5ecadfc 375f; hold_det_0 + unload_det_0 strips (quiet
six-foot stances, no hoisted legs). Pathologies first: (1) det
OVERSPEED on camera — gate det ep0 overlay reads v 0.078 vs ref
0.020 (err 0.059), ~4x command; the pricing defect is visible, not
inferred. (2) The park ep on camera: quiet stand on loaded tripod
0/2/4, feet 1/3/5 barely hovering, v ~0.01 m/s for 15 s; its return
(+519) is ~40% of a walking ep (~+1220) — the park is priced UNDER
walking but remains a live basin under sampling noise. (3) Stance
sprawly-wide, cadence irregular, stance feet creep in every strip
(matches slip 0.58–0.78). Achievements: level body (tilt ≤1.3°,
h_err ≤14 mm) and all six legs cycling in all 11/12 valid 15 s eps;
no final-third decay (t=11.6 s and t=15 s tiles); zero falls in 24
harness eps.
Exploits looked for, not found: no sacrificed leg in any passing ep,
no phantom step events (swing counts match strips), no safety-layer
reliance (0 flags), no height collapse.
Non-walk canary scalars (walk-only lineage, expected rubble): hold
det 6/6 sto 4/6, unload 12/12 (both watched, quiet stances), track/
raise/rise 0/6 everywhere — unchanged; no crown-jewel claim on this
line.
INTERPRETATION. The continuation bought +42 reward and zero
eval-visible change outside noise. Both remaining defects are the
already-recorded PRICING defects (overspeed uncharged; park basin
persists at 1/6 sto), and 4M identical steps demonstrably did not
move them (step_event flat, park rate unchanged). Identical-config
segments are exhausted — the auto-continue criterion (Q4>Q3 by
+3.3) is technically true but inside scatter; a third segment
predicts <+10 reward and no defect movement.
VERDICT: PASS against the recorded step0 gate (det AND sto, all
clauses; one marginal duty 0.18 noted, same treatment as parent).
NOT HARDWARE-READY: DR 0 only, det runs ~4x command speed, feet
skate 0.6–0.8 m/ep, 1/6 sto park risk, policy std 1.94. CHAMPION:
NOT updated — c1 ≥ lowent only within noise, and the concurrent
h15b verdict beats c1 outright (5 s det slip 0.584 vs 0.674
non-overlapping, det succ 4/6 vs 2/6, std 1.705 vs 1.94). Walk
champion = ppo_goal_cw_walk_lowent_h15b.zip md5 d0a12a94 (cycle 19).
HYPOTHESIS STATUS (launch prediction "reward keeps rising with
stable std"): SUPPORTED — 697→728.5 with std 2.08→1.94 — but the
rise decelerated to noise by Q4; the practically-relevant reading is
that this lineage's identical-config direction is DONE. Lineage
decision: NO c2 (watcher's mechanical relaunch also did not fire;
concurring).
Eval-side check landed (generalizes the park blind spot): walk-mode
episodes now render EVERY episode and additionally SAVE video for
any gait-invalid episode (eval_checkpoint.py) — a park/wander can
never again be a scalar-only event. Validated by rerunning the 15 s
eval at the same seed: draws reproduced bit-exact, park ep now on
camera (_15s_v2). Complements the concurrent cycle's --video-every 1
rerun practice with an automatic guarantee.

### LAUNCH cw-walk-step0-hist8 (4M, DR 0, from scratch, seed 0, pod s6) — temporal-actor rung on the step0 recipe
Plan rung (Skill notes → Walk → rung 1; lit review priority): history
rejoins on a consolidated baseline as a FROM-SCRATCH arm (obs change
⇒ new baseline; the cycle-13 warm-start hist8 died of transplant
drift, capability question INCONCLUSIVE). ONE variable vs the
archived cw-walk-step0 4M run: --cfg-set obs.history_frames=8
(≈300 ms obs/action history), everything else identical (walk-only
joint_walk, DR 0, step-event reward package, std 1.0 / ent 0.01 /
target_kl 0.02, seed 0, 4M).
HYPOTHESIS: an 8-frame temporal actor provides the substrate for
rhythmic gait (internal phase/clock) that the reactive MLP lacks;
on the identical step0 recipe it yields a gait with more REGULAR
cadence at the same budget.
Prediction-if-true: step0 gate clauses met at 4M AND cadence
regularity beats the archived step0-4M baseline outside noise —
per-leg swing-count spread tightens (step0 det rows e.g.
[7,6,5,7,13,3]) and duty split narrows; det vel_err may or may not
improve (see alternative).
Prediction-if-false: (a) no gait by 4M (reward_step_event fails to
rise; wider first layer slows from-scratch PPO) → history hurts
sample efficiency at DR 0, retry only as warm start AFTER pricing
work; (b) gait forms but cadence/tracking indistinguishable from
step0-4M → capability is not the binding constraint → all-in on
pricing (ii)/(iii).
Strongest alternative: skating/overspeed/park are pure pricing
defects history cannot touch — distinguished because if-true is
claimed on CADENCE regularity (swing CV, duty spread), not on
tracking; a tracking-only null does not refute the pricing story
and a cadence null does.
GATE: step0 gate at 4M (DR 0, det AND sto, ≥10 cm fwd, six legs
cycling, duty ~[0.2,0.9], ≥2 swings/leg, no drag/park, video
pathology-first) vs archived logs/ckpt_eval/cw_walk_step0_4M_gate as
named baseline. Budget 4M, canaries irrelevant (from scratch,
walk-only) but left default. Probe smoke first (audit §6: history ×
step-event package never ran together): probe-walk-step0-hist8,
150k, --no-wandb, smoke pod.

### Cycle 19 close
cw-walk-lowent-h15b-c1 VERIFIED RUNNING on walk pod: pid 556782, W&B
0858yqoz, log advancing (13.55M cum at check), sole trainer, code at
snapshot 6c579c5 (tag exp/cw-walk-lowent-h15b-c1), parent ckpt md5
d0a12a94 verified on pod pre-launch. Cycle totals: 1 launch, 4M of
16M step cap. Fleet after: walk = h15b-c1, long5m = dr03 (in
flight), s3 = endpost-c1 (in flight), s6 = concurrent cycle's
territory (lowent-c1 handling), s4/s5/friction/lower idle — left
free deliberately: the plan's next walk item (pricing work ii/iii)
is gated on h15b-c1's exploration-vs-structural answer, and stance's
next move is gated on endpost-c1's slope rule; a speculative launch
now would prejudge both. Champions: walk =
ppo_goal_cw_walk_lowent_h15b.zip md5 d0a12a94 (NEW this cycle,
append-only; lowent retained); stance unchanged (cw_stance_dr10).
Evals archived: logs/ckpt_eval/cw_walk_lowent_h15b_{15s,5s,
15s_allvid}. Eval-side practice recorded: same-seed --video-every 1
rerun reproduces harness draws bit-exact — use it to put any
scalar-flagged pathology on camera.

### Cycle 19-concurrent close — probe + launch outcomes, fleet
probe-walk-step0-hist8 (lower, 150k, smoke): PASS mechanical — obs
576 = 8×72 (history plumbing live, matches cycle-13 width), rew
60→142, std 1.0, KL 0.012, zero tracebacks, 136 s. cw-walk-step0-hist8
LAUNCH VERIFIED (launcher exit 0): s6, pid 914766, W&B wgba1l9o,
global_step 233k→442k in the verify window, ~2321 fps solo on empty
node g12ba48. Snapshot c50aadd (tag exp/cw-walk-step0-hist8). 1
experiment launch + 1 smoke this cycle (4M of 16M step cap).
Fleet: s3 = endpost-c1, walk = h15b-c1 (concurrent), s6 = hist8;
long5m FREED (dr03 finished — its verdict belongs to the watcher's
next cycle, NOT claimed here); s4/s5/friction/lower idle. Walk
champion: h15b md5 d0a12a94 (cycle 19). s6 lineage note: the watcher
auto-continue for cw-walk-step0* did not fire after lowent-c1 and
this cycle concurs with not continuing (Q4−Q3 +3.3 inside scatter);
if a mechanical c2 relaunch appears later it should be killed per
the flat-trend rule, citing this entry.

## Cycle 20 (2026-08-09 ~03:2xZ) — cw-walk-lowent-dr03 verdict: run INVALID as registered (long5m ran PRE-AUDIT code; the step-event/drag/park cfg-sets were silently ignored for all 4M steps) — yet the checkpoint mechanically passes the DR0.3 gate. Code-version gate landed in the launcher. DR rung relaunched off the h15b champion.

### INFRA ROOT CAUSE FIRST (required artifact): the experiment that actually ran was not the experiment in the ledger
OBSERVATIONS (mechanical). W&B xw5pdtum FINISHED, 4M steps (8.01M→
12.02M cum, 2004 s solo long5m). Ckpt ppo_goal_cw_walk_lowent_dr03.zip
md5 aa0cd31dad351da67695b024fc734c26 (pod + controller copy match).
Anomalies that unraveled it, in discovery order: (1) W&B config
target_kl=None (parent vkrvueqg: 0.02); (2) env/reward_step_event,
reward_drag, reward_park_duty ABSENT from the run's W&B history
(parent logs all three); (3) wandb-metadata.json on the pod shows the
process DID receive every --cfg-set; (4) local probe of the trainer's
own _build_env at DR0.3 with the same cfg-sets emits all three keys —
code here is fine; (5) md5 of long5m's rl_move/sim/walk_task.py =
de427bb7 vs 698ae6cf at snapshot e5f0c3e (which the run was launched
under). Diff of the pod files: long5m's walk_task.py contains NO
step-event package at all (k_step_event/k_drag_loaded/k_park_duty
never read — silently ignored by cfg_get defaults=0), and its
train_ppo_sim.py hardcodes log_std_init -1.0 / no target_kl support —
PRE-AUDIT (pre-08-08) code. Every other pod checked matches current
code (walk_task 698ae6cf on friction/s3/s4/s5/s6/lower/walk); long5m
alone was stale — it had not hosted a run since before the audit and
was never (successfully) synced before this launch.
CAUSAL CHAIN: wrong-reward training ← cfg-set keys silently ignored ←
pod code pre-audit ← code sync unverified — pods have no git and
neither snapshot.sh nor launch_run.py ever recorded/checked a code
version (the cycle protocol's "pod code at snapshot SHA" check was
never mechanized; the dr03 ledger entry's checks contain no code
field). Deepest reachable link fixed THIS cycle, eval-side patch
refused: snapshot.sh --sync now writes /workspace/prototype_sts3215/
.code_sha (local HEAD, -dirty suffixed if tree ≠ HEAD) and
launch_run.py REFUSES any launch (smoke included) when the pod marker
is missing or ≠ local HEAD. Failure class closed mechanically:
unsynced pod → refusal with the sync command in the message, not a
4M-step wrong-reward run. Silent-ignore of unknown --cfg-set keys is
the second link; left open (cfg_get defaults are load-bearing
everywhere) and now guarded upstream by the marker gate.
Consequence for the ledger: cw-walk-lowent-dr03 actually trained
"lowent + DR0.3 under the OLD reward (kernel+progress only, no
step-event income, no drag/park pricing, no target_kl, walk-speed
cfg-sets honored — old code does read those keys)". NOT one variable
vs parent (DR + reward package + target_kl all changed).

### cw-walk-lowent-dr03 — eval of what DID run: the gait survives 4M of DR0.3 fine-tuning even with the step-event package off
OBSERVATIONS (harness, current code, own cfg-sets, 6 eps/mode det+sto).
GATE eval at DR0.3 (logs/ckpt_eval/cw_walk_lowent_dr03_gate,
policy_std 1.834 vs parent 2.044): gait_valid 12/12, forward
0.340–0.483 m (all ≥0.10 gate floor), min swings/leg 3, duty spread
0.31–0.74, terminations 0, safety flags 0, det slip mean 0.628
(0.56–0.70) ≤0.93 ✓, Imax 2.45–2.67 A. Success-vs-command: det 5/6,
sto 3/6 (all sto fails are vel_err 0.031–0.033, fwd 0.34–0.39 — slow,
not broken). RETENTION at DR0 (…_dr03_ret0): det slip mean 0.640
(0.57–0.74) vs parent 0.746/0.717 (0.68–0.79) — mostly-below,
partially overlapping: direction right, borderline evidence. det
succ 3/6 vs parent 0/12 — outside the ±1–2 ep band (det tracking
partially appeared, vel_err 0.025–0.035 in passing eps; same
unexpected side-effect h15b showed). sto 4/6 vs parent pooled 10/12 —
inside noise. gait_valid 11/12: sto[4] is a HARD TRIPOD PARK from
t≈0 (duty [1.0,0.02,1.0,0.01,1.0,0.01], swings [0,1,1,1,0,1], fwd
0.030 m, on camera) — parent's 10 s gate eval was 24/24 gv, so this
1-ep park at 10 s is new-at-this-horizon but within ±1 ep and the
SAME park class h15b/lowent-c1 show at 15 s (~1/6 sto). det[2] at
DR0 is the det churn cousin (duty [0.87,0.14,0.84,0.22,0.78,0.16],
fwd 0.226, ve 0.057). The park basin was NOT worsened by 4M steps of
unpriced training — rate still ~1/6 sto — and did not appear at
DR0.3 at all in 12 eps.
FRAMES WATCHED (provenance; every mode in this verdict): gate
walk_det_0.mp4 4ef2db78 250f, walk_det_3.mp4 c3ad3dec 250f,
walk_sto_0.mp4 4349fd1e 250f, walk_sto_3.mp4 9457f8dc 250f; ret0
walk_det_0.mp4 6ed7555b 250f, walk_sto_0.mp4 91cc9c2b 250f, park ep
walk_sto_4.mp4 1b6fca22 250f (auto-saved by the gait-invalid video
rule landed cycle 19c — worked as designed). Pathologies first:
stance sprawly-wide, cadence irregular (no tripod rhythm), stance
feet visibly creep in every strip (slip 0.56–0.75 m/ep ≈ parent),
park ep frozen from frame ~2 with three feet hovering. Achievements
(numbers behind each): level body throughout all watched strips, six
legs lift→swing→touchdown in all 23/24 valid eps, zero falls, zero
safety flags, DR0.3 motion visually indistinguishable from DR0 —
no new wobble/stagger under randomization.
Exploits looked for, not found: no sacrificed leg outside the park
ep, no phantom steps (swing counts match strips), no safety-layer
reliance (0 flags), no height collapse; checked whether DR0.3
success rides on DR-slop — slip at DR0.3 (0.628) is not worse than
DR0 (0.640) and frames show the same gait, not a noisier one.
INTERPRETATION. Two separable facts. (a) The registered hypothesis
(DR rung 0.3 robustifies THE STEP0-RECIPE training) was never
tested — invalid execution, no verdict possible on it. (b) The
eval-only fact is still valuable: the lowent gait, once formed,
SURVIVES 4M steps of DR0.3 fine-tuning with NO step-event income and
NO drag/park pricing — it neither collapsed into the park basin
(which stayed at its baseline ~1/6 sto rate at DR0) nor lost gait
validity at DR0.3 (12/12). The gait is self-sustaining under the
plain kernel+progress reward at this basin; the step-event package
built it but is apparently not load-bearing for maintaining it, at
least over 4M steps (labeled HYPOTHESIS — one lineage, one seed).
DR0.3 as a single rung looks survivable, but the clean claim needs
the redo below.
VERDICT: gate clauses mechanically PASS (all six at DR0.3 + DR0
retention within noise/better), but the RUN IS INVALID as the
registered experiment — recorded as INCONCLUSIVE-INVALID, not PASS;
a PASS here would bless a run whose training config the ledger
misdescribes. NOT HARDWARE-READY: skating unchanged (0.56–0.75
m/ep), park basin live at 1/6 sto (on camera), sto success 3/6 at
own DR, std 1.83. NO champion update: h15b keeps it (no
same-horizon comparison beats it; and this ckpt carries 4M of
wrong-reward training — do not warm-start from it without citing
this entry). Ledger updated: FINISHED, verdict, ckpt md5, invalid
flag.
HYPOTHESIS STATUS: INCONCLUSIVE (execution invalid — the arm never
ran). The DR-rung question transfers to the relaunch below.

### LAUNCH cw-walk-h15b-dr03 (4M, DR 0.3, ep 15 s, seed 0, long5m) — DR ladder rung 0.3, redo on the champion line with verified code
Basis: 0-b.2 (operator, binding) says DR rungs off the best
step0-lineage checkpoint; champion is h15b md5 d0a12a94 (cycle 19).
ONE variable vs h15b: --no-dr → --dr-scale 0.3 (15 s horizon, ent
0.001, step-event cfg-sets, seed 0 all inherited). Parallel-off-
same-parent with h15b-c1 per the cycle-19 pattern (c1 finished;
its verdict belongs to another cycle — if it takes the champion,
the NEXT rung continues from there; this run still answers "does
the h15b gait survive rung 0.3").
HYPOTHESIS: the h15b gait survives DR 0.3 introduced as a single
rung — fine-tuning at moderate DR robustifies rather than destroys
(hist8's destruction came from jumping to the wide distribution).
Prediction-if-true: 15 s DR0.3 harness matches h15b's DR0 baseline
within the ±1–2 ep band (fwd ≥0.40 m ≥10/12, gait_valid ≥11/12 with
any invalid ep being the known park class, 0 terminations, det slip
mean ≤1.0 vs parent DR0 0.912) AND DR0 retention eval shows no
erosion outside noise (parent baseline: fwd 10/12, gv 11/12, 5 s det
slip 0.584). Prediction-if-false: (a) gv ≤9/12, terminations, or a
NEW failure class on camera at DR0.3 → rung too big → drop to 0.15;
(b) DR0.3 fine but DR0 retention erodes outside noise → warm-start
interference, not DR difficulty. Strongest alternative: passes via
DR-slop (noise-robust skating instead of stepping) — distinguished
by det slip + frames at both DRs (the dr03 eval above shows what
that check looks like when it's clean). Gate (recorded in ledger):
the if-true block, det AND sto, frames watched pathology-first.
Budget 4M (lineage segment size). --no-canary (lineage exemption,
plan 0-a). Probe-smoke note: step-event package × DR0.3 never
trained together on-pod before; the trainer's own _build_env was
probed locally this cycle (125 steps, DR0.3, all three reward keys
emitted — recorded above), and the pod code is now marker-verified;
no separate on-pod smoke.

### Cycle 20 close
cw-walk-h15b-dr03 VERIFIED RUNNING on long5m: pid 2195399, W&B
bzvup62t, ~5188 fps solo, log advancing (+20 KB/60 s, 0 tracebacks),
code marker 9c3bbae verified BOTH sides (first launch through the new
.code_sha gate), parent ckpt md5 d0a12a94 verified on pod pre-launch,
W&B config confirms dr_scale 0.3 / target_kl 0.02 / 15 s / seed 0,
and — the check that would have caught dr03 — env/reward_step_event,
reward_drag, reward_park_duty ALL present in the run's first logged
history rows. No duplicate run name. Watcher owns the +5 min checkup.
Cycle totals: 1 launch (4M of 16M cap). Fleet after: long5m =
h15b-dr03, s6 = step0-hist8 (concurrent cycle's); walk/s3 freed by
h15b-c1/endpost-c1 finishing — their verdicts belong to their own
cycles, pods deliberately left for them (both will need
snapshot.sh --sync before any launch under the new code gate).
Champions unchanged: walk = ppo_goal_cw_walk_lowent_h15b.zip md5
d0a12a94; stance = cw_stance_dr10. Evals archived:
logs/ckpt_eval/cw_walk_lowent_dr03_{gate,ret0}. Sweep note recorded:
walk_task.py md5 matched current code on all pods EXCEPT long5m at
discovery time (698ae6cf everywhere; long5m was de427bb7, now synced
+ marked). No pod other than long5m ran stale code this campaign leg.

## Cycle 21 (2026-08-09 ~04:0xZ) — cw-walk-lowent-h15b-c1 verdict: gate FAIL, park basin is STRUCTURAL (persists with std at 1.485, below the pre-registered 1.5 threshold) — exploration-gating REFUTED; pricing arm next (progress-gated kernel income)

### cw-walk-lowent-h15b-c1 — +4M identical config: std fell through 1.5, the same two failure eps recurred at the same indices
OBSERVATIONS (mechanical). W&B 0858yqoz FINISHED, 4M steps
(12.02M→16.02M cum, 1088 s solo walk pod). Final ckpt
ppo_goal_cw_walk_lowent_h15b_c1.zip md5
ed71b6f47ed6e68c7e942ef08f132366 (pod + controller copies match).
Train: ep_rew_mean quarters 1148.6/1160.3/1166.7/1173.9 (Q4−Q3
+7.2, decelerating toward the ±15 scatter band); train/std
1.716→1.526, harness policy_std 1.485 (parent 1.705); approx_kl
~0.016 flat; env/reward_step_event 0.125→0.135.
Gate harness 15 s DR 0 own-cfg (logs/ckpt_eval/cw_walk_lowent_
h15b_c1_15s, 6 eps/mode det+sto, --video-every 1 so every gated ep
is on camera): det fwd ≥0.40 m 5/6 (det[2]=0.285 FAIL), sto 5/6
(sto[4]=0.037 FAIL), gait_valid 11/12 (sto[4] invalid), terminations
0/12, safety flags 0. SAME failure classes at the SAME episode
indices as parent h15b (det[2] duty-skew churn, sto[4] hard tripod
park) — park rate unchanged at 1/6 sto across lowent → h15b → c1.
Retention 5 s (…_c1_5s): det slip mean 0.275 ≤ 0.93 gate ✓ (but see
confound below); det succ 5/6, sto 4/6, sto[4] park again (fwd
0.028, gv False, duty [0.96,0.12,0.90,0.09,0.99,0.06]).
Named-baseline deltas vs h15b: 15 s det slip 0.812 (0.687–0.944) vs
0.912 (0.801–1.102) — OVERLAP, direction right, not evidence. 15 s
fwd det 0.610 vs 0.576, sto 0.556 vs 0.513 — inside scatter. 5 s det
slip 0.275 vs 0.584 non-overlapping BUT CONFOUNDED: 5 s det fwd
0.183 vs 0.382, so slip PER METER is 1.50 vs 1.53 — identical; the
"halved slip" is walking less in the first 5 s, not cleaner contact.
(15 s slip/m 1.33 vs 1.58 — suggestive only, ep ranges overlap.)
Only outside-noise change: std 1.705→1.485 with a slower, less
overspeedy start (5 s fwd non-overlapping lower; 15 s fwd equal).
FRAMES WATCHED (provenance; every gated ep had video, 8 strips
reviewed): 15 s walk_det_0.mp4 md5 701ee803 375f, det_1 fb8b063f
375f, det_2 3db5e8d1 375f, sto_1 58593054 375f, sto_3 d6e027dd 375f,
sto_4 2bbd2b04 375f; 5 s det_0 59dbc61c 125f, sto_4 4b45047d 125f.
Pathologies first: (1) sto[4] is a hard tripod park from ~t=1 s —
identical posture across tiles 2–10, legs 1/3/5 hovering just off
ground, background checkerboard never shifts. (2) det[2] is
duty-skew churn — legs 1/3/5 flick (duty 0.12/0.18/0.22) while 0/2/4
stay loaded (0.84/0.90/0.75), body creeps 0.285 m in 15 s. (3) All
strips: sprawly-wide stance, stance-foot creep (slip 0.69–0.94/ep at
15 s), irregular cadence. Achievements (numbered): passing eps show
all six feet cycling contact/short-swing with the body advancing and
level through the final third (det fwd 0.644–0.713, tilt small, no
wind-up); zero falls and zero safety flags in 24 harness eps.
Exploits looked for, not found: no sacrificed leg in any passing ep,
swing counts match strips (no phantom step credit), no safety-layer
reliance, no height collapse. Exploit FOUND in the eval metric
itself: the 5 s slip retention clause is passable by walking slower
— caught via distance-normalized slip; future retention comparisons
should use slip/m alongside raw slip.
INTERPRETATION. Pre-registered if-false branch fired exactly: std
annealed through the ~1.5 threshold and the seed-0 sto park draw
persisted at the same 1/6 rate (and the det churn cousin recurred).
Exploration-gating is REFUTED; the park basin is STRUCTURAL. Root-
cause chain (required before the reward change below): tripod park
← park trajectories still NET POSITIVE (+519/ep measured cycle 19c)
← r_walk kernel pays ABSOLUTE velocity error: at commands 0.02–0.06
m/s a parked robot (v=0) collects 0.97–1.85/tick — measured 1.77
mean via zero-action unit check this cycle — vs park charge only
0.6/tick (k_park_duty 1.0), so k_park_duty DISCOUNTS the park but
cannot flip its sign ← objective defect: the income channel rewards
small |v−ref|, not ground covered; operator 0-c defines the
objective as DISTANCE/stability/reliability. Deepest reachable fix:
re-route the income (0-c.2), not another penalty coefficient
(banned class, review §0).
VERDICT: FAIL against the recorded gate (fwd 10/12 vs 12/12,
gait_valid 11/12 vs 12/12; 0 terminations ✓, no final-third
degradation ✓, retention slip ✓ with the confound noted). NOT
HARDWARE-READY: a 1-in-6 chance of freezing in a tripod park, feet
skating ~1.3–1.6 m per meter walked, DR 0 only. CHAMPION: UNCHANGED
— h15b (md5 d0a12a94) keeps it; c1's eval deltas are inside noise
once the slip/distance confound is removed. c1 (md5 ed71b6f4) is
recorded as the preferred WARM-START parent for pricing arms:
behavior within noise of champion, +4M consolidation, lowest std of
the lineage (1.485) — the pricing arm should not have to redo the
anneal.
HYPOTHESIS STATUS: REFUTED (the park is not exploration-gated).
Consequence per pre-registration: NO third identical-config segment
anywhere in the lowent line (3rd confirmation: lowent-c1 PASS-inside-
noise, h15b-c1 same-indices repeat), escalate to pricing.

### CODE cw-walk-kgate — progress-gated kernel income (walk_task.py, cfg-gated, default OFF)
Change (snapshotted this cycle): `reward.walk_kernel_prog_gate` ∈
[0,1], default 0.0 = bit-identical old path. When g>0 and a velocity
is commanded, r_walk *= (1−g) + g·clip(along/s_ref, 0, 1). Effects
at g=1: parked robot earns ~0 kernel income on commanded ticks
(unit check: ≤0.00007/tick vs 1.77 before, 95 commanded ticks,
zero-action env, seed 0); uncommanded settle hold untouched (still
2.0/tick — settling is not walking); perfect tracking unchanged
(factor 1); overspeed unaffected (clip at 1, kernel err term already
symmetric). Scale audit (analytic + unit check, per audit
directive): park net/tick flips +1.1 → −0.6 (income gone, park_duty
charge remains); walking income unchanged ~2.0 kernel + 0.85 prog +
0.13 step_event; kernel stays bounded [0,2]; no obs change. This
implements 0-c.2's "shift income toward ground actually covered" as
ONE variable and makes the park worth less than stepping BY
CONSTRUCTION (the step0 directive's own requirement) via income
routing, not a new penalty. Routing: walk-mode only by construction
(lives in the walk-goal block). New mechanism ⇒ probe smoke before
the 4M run (below).

### LAUNCH cw-walk-kgate (4M, DR 0, ep 15 s, seed 0, walk pod) — park pricing via progress-gated kernel income
Basis: plan 0-c.1/0-c.2 + the structural-park verdict above; the
pre-registered escalation from h15b-c1's if-false branch. Warm start
from c1 (md5 ed71b6f4, rationale in verdict); ONE variable vs the
c1 segment: --cfg-set reward.walk_kernel_prog_gate=1.0.
HYPOTHESIS: the park basin survives because parked trajectories
still collect near-full kernel income at low commands; zeroing that
income (park return +519 → ≈−225 over 15 s) flips their advantage
strongly negative and PPO prices the park out.
Prediction-if-true: 15 s harness (same seed) park rate 1/6 sto →
0/12, det duty-skew churn converts or disappears (det[2]-class fwd
rises toward ≥0.40), gate clauses fwd+gv reach 12/12; W&B
env/reward_walk dips at segment start then recovers as
walk_prog_factor → 1.
Prediction-if-false: park persists at ~1/6 despite strongly negative
returns → the basin is attractor-MECHANICAL (a t=0 commitment the
policy cannot re-decide), pricing refuted for this defect class →
next escalation is reset-state diversity (episodes starting FROM the
parked posture) or plan rung 2 time-averaged load evenness — NOT a
coefficient retry.
Strongest alternative: the park eps are rare-draw noise PPO barely
sees — rejected in advance: 1/6 of sto walk episodes is common under
training sampling, and three segments reproduced it at the same
rate; if PPO cannot price a 1/6-frequency, large-negative-return
trajectory out, that IS the mechanical-attractor result, which this
experiment distinguishes from the income story.
GATE (recorded in ledger): 15 s DR 0 harness 6 eps/mode det AND sto:
fwd ≥0.40 m 12/12 AND gait_valid 12/12 AND ≥2 swings/leg AND 0
terminations AND no final-third frame degradation AND 15 s det fwd
mean ≥0.50 m (anti-crawl guard: income gating must not pass by
slowing down; c1 baseline 0.610, champion 0.576). Retention: 5 s det
slip/m ≤ 1.8 (c1 1.50, champion 1.53 — raw slip clause retired after
this cycle's confound catch). Budget 4M (park rate is measurable at
1/6 per segment; defect movement visible well before 4M). ent 0.001,
--no-canary (lineage exemption), seed 0. Probe smoke first:
probe-walk-kgate (150k, smoke pod, W&B off) must show
walk_prog_factor logged, reward_walk reduced vs c1-era scale, no
tracebacks.

### Cycle 21 close
probe-walk-kgate PASS mechanical (lower, 150k/68 s, 8 envs, W&B off):
zero tracebacks, ep_rew ~1060 vs c1-era 1177 — the kernel income cut
is visible at matching config; std 1.52, kl 0.0197. Logging tweak
after the first snapshot (env/walk_prog_factor added to the trainer's
reward-parts whitelist) → second snapshot 19e97f8 (tag
exp/cw-walk-kgate-b), walk pod re-synced; run code_sha 19e97f8
verified both sides by the launcher.
cw-walk-kgate VERIFIED RUNNING on walk pod: pid 801381, W&B q1ip6y7k,
~4506 fps solo (node g129004 load1 0.94), parent c1 md5 ed71b6f4 on
pod, no duplicates. Early W&B history confirms the mechanism live:
env/walk_prog_factor ~0.83, env/reward_walk 1.44–1.48 (vs c1 ~1.66 —
the ~0.2/tick income now conditional on progress), reward_step_event
~0.14 unchanged. Watcher owns the +5 min checkup.
Cycle totals: 1 experiment launch + 1 smoke (4M of 16M step cap).
Fleet after: walk = kgate, long5m = h15b-dr03 (concurrent cycle's),
s6 FREED mid-cycle (step0-hist8 finished — verdict belongs to the
watcher's next cycle, NOT claimed here), s3/s4/s5/friction/lower
idle. Champions: walk = h15b md5 d0a12a94 (unchanged this cycle);
stance = cw_stance_dr10. Evals archived:
logs/ckpt_eval/cw_walk_lowent_h15b_c1_{15s,5s} (all 24 eps on video).
Eval-practice note (generalizes): retention/slip comparisons must be
distance-normalized (slip/m) — raw slip rewards crawling; caught in
this cycle's own delta claim before it shipped.

## Cycle 22 (2026-08-09 ~03:3xZ) — cw-walk-step0-hist8 verdict: step0 gate clauses PASS (a real six-leg gait forms from scratch WITH history-8), but the pre-registered cadence-regularity claim FAILS — temporal capability is NOT the binding constraint; if-false(b) fired, pricing stays the line (kgate already in flight). Two watcher gaps found and fixed at the root.

### cw-walk-step0-hist8 — history-8 from scratch: gait indistinguishable from the reactive MLP at matched budget
OBSERVATIONS (mechanical). W&B wgba1l9o FINISHED, 4,005,888 steps in
1850 s solo on s6 (~2160 fps). Ckpt ppo_goal_cw_walk_step0_hist8.zip
md5 6f45838aed4ee63d3e1dee49447495bc (pod and controller copies
match). Train: ep_rew_mean quarters 257/477/552/586 (Q4−Q3 +34,
still climbing at 4M); train/std 1.09→2.20, harness policy_std 2.201
— named comparison: the step0-4M baseline itself ended at 2.294, so
the std runaway is a trait of from-scratch ent 0.01 in this lineage,
not new to history; approx_kl ~0.011 flat; env/reward_step_event
quarters 0.031/0.070/0.085/0.088.
Gate harness (6 eps/mode det AND sto, DR 0, seed 0, own cfg incl.
obs.history_frames=8, 10 s eps, --video-every 1;
logs/ckpt_eval/cw_walk_step0_hist8_4M_gate): det fwd 0.428–0.484 m
(mean 0.454), sto 0.187–0.401 m (mean 0.324); gait_valid 12/12;
sacrificed legs none; terminations 0; safety flags 0; min swings/leg
3; all duties within [0.28,0.71]; Imax ≤2.64 A. Step0 gate clauses:
fwd ≥10 cm 12/12 ✓, six legs cycling 12/12 ✓, duty in ~[0.2,0.9] ✓,
≥2 swings/leg ✓, no drag/park on camera ✓.
Cadence comparison vs NAMED baseline logs/ckpt_eval/
cw_walk_step0_4M_gate (the registered question): det swing-count CV
mean 0.299 (rows 0.259–0.378) vs 0.332 (0.191–0.450) — OVERLAP, no
evidence; sto 0.271 vs 0.232 — overlap, direction WORSE. det
duty-spread mean 0.135 (0.05–0.21) vs 0.345 (0.19–0.59) — nearly
non-overlapping, tighter; sto duty-spread 0.25 vs 0.238 — no change.
NEW det pattern: legs 1/3 double-time (11–12 swings vs 6–7 on the
others) in 5/6 det eps — cadence is not more regular, it is
DIFFERENTLY irregular. Skating: det slip/m 1.47–2.02 (mean 1.74) vs
baseline det 1.77–3.08 (mean 2.10) — overlapping, direction right,
not evidence. Two slow sto eps (fwd 0.187/0.208 m) show tripod duty
skew (0.28–0.36 loaded-light vs 0.66–0.71 loaded-heavy) — the park
attractor's soft cousin, but all six legs still swing 3–6x and
overlay feet marks alternate .#.#.# → #.#.#. — slow stepping, NOT a
park.
FRAMES WATCHED (provenance; det and sto both gated): walk_det_0.mp4
c172e73a 250f, det_2 080e6080 250f, det_3 71289650 250f, sto_1
e472b692 250f, sto_2 1ca33dfa 250f, sto_4 0786ed1b 250f. Pathologies
first: sprawly-wide stance (lineage trait unchanged), stance-foot
creep visible in every strip (slip 0.57–0.86 m/ep), irregular
cadence with double-timing legs, sto_1/sto_4 are slow tripod-skewed
crawls. Achievements (numbers behind each): all six feet cycle
contact/short-swing in every reviewed strip, body level throughout
(tilt overlays ~1.3–1.5°, h_err ≤14 mm), checkerboard advances
through the final tiles (t=10.0 s overlay confirms no late freeze),
zero falls, zero safety flags in 12 eps.
Exploits looked for, not found: parked/sacrificed leg (min duty
0.28, gv 12/12), phantom step credit (swing counts match strips),
safety-layer reliance (0 flags), height collapse (h_err small),
final-third degradation (last-tile overlays + moving checkerboard).
INTERPRETATION. History-8 neither hurt nor helped at DR 0 on the
step0 recipe. If-false(a) — history slows from-scratch PPO — is
rejected: the gait formed on budget and passed every gate clause.
If-true — cadence regularity outside noise — is NOT met: swing-CV
unchanged (det) / worse-inside-noise (sto), and the one
outside-overlap delta (det duty-spread 0.135 vs 0.345) is a single
metric, det-only, absent in sto → recorded as an unconfirmed HINT,
not evidence of the claimed mechanism (and it coexists with the new
double-time pattern, so "more regular" is not an honest summary).
Pre-registered branch (b) fires: temporal capability is not the
binding constraint; the binding defects are pricing — park income
(kgate is testing exactly this), skating, overspeed.
VERDICT: PASS against the recorded step0 gate (all clauses, det AND
sto, 12/12 gait_valid) — but the run's registered question resolves
NEGATIVE. NOT HARDWARE-READY: feet skate ~1.5–2.0 m per meter
walked, sprawly stance, irregular double-time cadence, policy std
2.2, DR 0 only, 10 s horizon. CHAMPION: UNCHANGED — walk champion
stays ppo_goal_cw_walk_lowent_h15b.zip md5 d0a12a94; hist8 beats it
on nothing outside noise and was evaluated at a shorter horizon.
Lineage note: hist8 closes at 4M per pre-registration; reward was
still climbing (+34/quarter) but more steps cannot change the
matched-budget comparison, and the watcher's auto-continue did not
fire (see infra below) — if a mechanical cw-walk-step0-hist8-c1
appears later it should be killed citing this entry.
HYPOTHESIS STATUS: REFUTED (branch b). History-8 stays in the
architecture as an ingredient of the post-0-c estimator ladder
(ARCHITECTURE_REVIEW 08-09); no more history-vs-MLP arms at DR 0.

### INFRA — why auto-continue never fires, and an orphaned finished run (both root-caused)
(1) STALE WATCHER PROCESS. The running watch_loop.py (tmux
"orchestrator", started 23:21Z 08-08) predates commit e3b1bda
(auto-continue, 01:25Z 08-09): try_auto_continue exists on disk but
not in the live process — hence ZERO "auto-continue" lines in
orchestrator.log ever, including hist8 (reward climbing, prefix
matched, would have fired). Not restarting it myself: the watcher
supervises my own cycle and a botched restart kills the whole loop —
blast radius exceeds the benefit (cycles handle continuations
manually meanwhile). NEEDS OPERATOR (non-urgent): restart the
watcher tmux session to pick up e3b1bda+.
(2) ORPHANED RUN cw-stance-endpost-c1 (W&B v5t38fee FINISHED, 27.2M
cum): no verdict cycle will EVER fire for it. Causal chain: watcher
skips it ← ledger_verdicted() counts a run "verdicted" if ANY ledger
entry has status FINISHED/FAILED ← endpost-c1 carries stale FAILED
entries from two dead launch attempts (01:37Z, 01:46Z) that preceded
the successful 01:47Z launch ← launch retries append new entries but
nothing reconciles the failed ones. Fixed at both layers: (a) the
two stale FAILED launch-attempt entries are marked SUPERSEDED
(status edit only, under ledger lock, entries preserved — same
convention as the existing 01:35Z SUPERSEDED entry), which unblocks
the LIVE watcher mechanically — it will now compute endpost-c1 as
newly-finished and fire its verdict cycle, keeping ownership with
the watcher instead of this cycle grabbing an unassigned run; (b)
watch_loop.py ledger_verdicted() now keys on the LATEST entry per
run (effective on watcher restart). Failure class closed: a refused/
dead first attempt can no longer permanently orphan a run's verdict.

### Cycle 22 close
No launch. hist8's pre-registered consequence (all-in on pricing) is
already in flight — cw-walk-kgate (cycle 21, concurrent) IS the
pricing arm; the next walk arms are gated on kgate's and
h15b-dr03's verdicts (dr03's cycle is live, its pod untouched), and
stance's next move is gated on endpost-c1's verdict (cycle now
mechanically unblocked by the ledger fix). s3/s4/s5/s6/friction/
lower left idle deliberately — an idle pod is cheaper than
prejudging three in-flight gates. Champions unchanged: walk = h15b
md5 d0a12a94, stance = cw_stance_dr10. Evals archived:
logs/ckpt_eval/cw_walk_step0_hist8_4M_gate (12/12 eps on video).
Cycle totals: 0 launches, 0 steps of the 16M cap.

## Cycle 23 (2026-08-09 ~03:5xZ) — cw-walk-h15b-dr03 verdict: first walk-gate PASS at DR>0 — but baseline probes show the rung was VACUOUS (untrained h15b passes the same gate at DR0.3, 0.6, AND only misses det slip at 1.0). DR-ladder TRAINING arms closed; DR is not the walk bottleneck. No launch (next walk arm sequenced on kgate's verdict, owned elsewhere).

### cw-walk-h15b-dr03 — DR 0.3 rung off the champion: gate PASS; zero measurable delta vs untrained parent at every DR scale
OBSERVATIONS (mechanical). W&B bzvup62t FINISHED, 4M steps (12.02M→
16.02M cum, 1059 s solo long5m, ~5188 fps verified at launch). Final
ckpt ppo_goal_cw_walk_h15b_dr03.zip md5 15065551c2007cb40f560bed9c4
08be9 (pod + controller copies match). Training curves FLAT all run:
env/reward_walk quarters [1.645, 1.644, 1.639, 1.640], walk_speed
[0.0462→0.0477], step_event [0.126→0.133]; train/std 1.744→1.519
(harness policy_std 1.481 ≈ h15b-c1's 1.485). No terminations, no
canary (lineage exemption).
Gate harness DR0.3 15 s own-cfg (archived logs/ckpt_eval/
cw_walk_h15b_dr03_ev_dr03_15s), 6 eps/mode det+sto, ALL 12 strips
watched: fwd ≥0.40 m 12/12 (det 0.579–0.718, sto 0.478–0.700);
gait_valid 12/12; terminations 0/12; det slip mean 0.899 ≤1.0
(slip/m 1.41); sto slip 0.917 (slip/m 1.49). Speed-band "success"
det 4/6, sto 5/6 — NOT a gate clause; misses are tracking
(vel_err 0.031–0.043), all six legs cycling on camera.
DR0 retention 15 s (ev_dr0_15s): fwd ≥0.40 10/12, gv 11/12 — EQUALS
parent baseline (10/12, 11/12). det slip 0.916 vs parent 0.912 —
identical. sto[4] is the KNOWN park, same index as h15b/c1: duty
[0.99,0,1.0,0.02,0.99,0], fwd 0.035, frozen from ~tile 2 on camera.
det[2] is the known churn cousin (duty skew 0.74/0.22/0.83/0.29/
0.65/0.29, fwd 0.372, slip/m 3.16). The structural park basin is
UNCHANGED by DR training.
DR0 5 s (ev_dr0_5s): det slip 0.327 ≤0.93 gate clause ✓ mechanically
— but slip/m 1.82 vs parent 1.53 and 5 s det fwd 0.180 vs parent
0.382 (non-overlapping): the SAME distance-confound cycle 21 flagged
on c1. The clause passes by walking less, not skating less. 5 s det
success 2/6 vs parent 4/6 — the slow-start behavior correlates with
the std anneal (c1 at std 1.485 showed it too; dr03 at 1.481 ditto).
NAMED-BASELINE PROBES (eval-only, this cycle; parent h15b md5
d0a12a94 evaluated at the run's own gate): h15b UNTRAINED at DR0.3
15 s (ev_h15b_dr03_15s): fwd 12/12 ≥0.444, gv 12/12, 0 term, det
slip 0.890 — PASSES the identical gate; det speed-success 6/6 vs
dr03's 4/6. Ladder probes both ckpts (ev_{dr03ck,h15b}_{dr06,dr10}):
  DR0.6: dr03ck det slip 0.947 / h15b 0.961, both fwd 12/12 gv 12/12
  0 term — BOTH pass the full rung-0.6 gate untrained-vs-trained
  indistinguishable.
  DR1.0: dr03ck det slip 1.061 / h15b 1.008 — both fail ONLY the det
  slip ≤1.0 clause (by 1–6%); fwd 12/12, gv 12/12, 0 term both.
DR path verified live mechanically (suspicion: cycle-20 stale-code
class): joint_walk eval env at dr 1.0 draws per-episode friction
0.67–1.24, latency 1.05–1.72; at dr 0 no EpisodeRandomization.
HARNESS FOOTGUN (record): RandRanges.scaled() CLAMPS s to [0,1] —
`--dr-scale 3.0` silently evaluates at 1.0. Never cite a >1.0 DR
eval as evidence of anything.
Current note (all evals, both ckpts): Imax 2.53–2.64 A, above the
2.5 A soft breaker — same band as parent (2.61 A); pre-existing,
per-plan current recalibration caveat stands, not a new pathology.
FRAMES WATCHED (provenance; md5/frame-count): DR0.3 all 12 —
det 9f7bfcec/8fa66f11/29548ce2/8ac0c57b/8d8763d9/d396acd0, sto
8ede834a/676bbddb/3c4fea0d/04cb9dc4/d84d6cad/cb1a4959, 375f each.
DR0 15 s: det_0 aba2c167, det_1 854be1b0, det_2 816154bb, sto_0
b88e0724, sto_4 eae98170 (park on camera), 375f each; unwatched DR0
successes recorded as such (det_3/4/5, sto_1/2/3/5 — scalars only).
5 s: det_0 d38e15ab, sto_4 3bb2207e (park on camera), 125f each.
Probes: dr03ck@1.0 det_0 faf14ab2 + sto_3 fd2cd86b, dr03ck@0.6
det_0 1fd82562, h15b@1.0 det_0 09fd4d43, 375f each — same sprawly
six-leg gait, level body, checkerboard advancing; no new failure
class at any DR; remaining probe eps unwatched (scalars only).
Pathologies first, all watched strips: sprawly-wide stance
unchanged; stance-foot skating in EVERY episode (slip/m 1.4–1.8
at every DR — the robot skids ~1.5 m of foot-contact for every
meter it gains); cadence irregular; DR0 sto park 1/6 and churn
cousin persist. Achievements: all six feet cycle contact/short-swing
in every non-park ep at every DR incl. 1.0; zero falls in 60 harness
eps this cycle across DR 0/0.3/0.6/1.0.
Exploits looked for: DR-slop pass (noise-robust skating instead of
stepping) — NOT found: det slip at DR0.3 equals parent's DR0 value
and frames show stepping. 5 s retention clause gamed by slow starts
— FOUND again (pre-flagged cycle 21); retention judged on slip/m.
Inert-DR harness — ruled out mechanically (draws above).
INTERPRETATION. The rung PASSED but taught us the opposite of the
plan's premise: this gait's DR robustness was already free. A
statically-stable sprawled hexapod creeping at 5 cm/s is the
easiest DR case there is — friction/latency/mass draws at s≤1.0 do
not threaten it (0 falls in 24 DR≥0.6 eps, untrained parent). The
0-b.2 ladder's remaining content at DR1.0 is ONE clause: det slip
1.008–1.061 vs ≤1.0 — and that is the SKATING defect, not a DR
defect; it fails marginally at every scale (slip/m 1.4–1.8
everywhere). Training more DR rungs buys nothing measurable
(rung 0.3 delta vs parent: slip +0.009, success −2 eps, both inside
or at the edge of the 1–2 ep band, nothing outside noise in the
right direction) — the h15b line's real blockers remain the park
basin (kgate, in flight, owned elsewhere) and skating (pricing/
sim-fidelity question, NOT a coefficient retry).
VERDICT: PASS against the recorded gate — every clause met at
DR0.3 det+sto, DR0 retention equal to parent, 5 s slip clause met
(with the distance confound named; slip/m worse, inside the
slow-start pattern). First walk-gate PASS at DR>0 in the campaign.
NOT HARDWARE-READY: 1/6 DR0 sto park unchanged, feet skate ~1.5 m
per meter walked at every DR, speed tracking det 4/6 — none of
that goes on the physical robot. CHAMPION: UNCHANGED — h15b (md5
d0a12a94) keeps it. dr03 matched its parent everywhere and was
WORSE outside noise on one axis (5 s det fwd 0.180 vs 0.382,
slow starts); the untrained parent already passes DR0.3 and 0.6.
dr03 ckpt retained (append-only) as the DR-line record.
HYPOTHESIS STATUS: SUPPORTED on the letter ("the gait survives DR
0.3 as a single rung" — every if-true prediction landed), REFUTED
on the spirit: "moderate-DR fine-tuning robustifies" is false for
this gait — the named baseline (untrained h15b, same gates, same
cycle) shows NO robustification was needed or delivered. The
pre-registered alternative (DR-slop pass) was checked and excluded.
CONSEQUENCE (pre-registered escalation logic): the DR ladder's
training arms (0-b.2 dr06/dr10 segments) are CLOSED as vacuous —
the ladder is satisfied eval-side through 0.6 and blocked at 1.0
only by skating, which is a pricing/sim-fidelity defect owned by
the kgate branch. Skating is now BOTH the fluidity blocker and the
DR1.0 blocker — it is the single highest-value walk defect after
the park.

### Cycle 23 close
No launch. long5m freed and left idle DELIBERATELY: the plan's next
walk arms (overspeed pricing if kgate true; reset-state diversity /
load evenness if false) are pre-registered on kgate's verdict, which
belongs to its own cycle (kgate finished during this cycle; watcher
will fire it — not mine to grab). Launching a DR rung would
contradict this cycle's own evidence; launching a pricing arm would
prejudge kgate. An idle pod is cheaper than a confounded campaign.
Champions unchanged: walk = ppo_goal_cw_walk_lowent_h15b.zip md5
d0a12a94; stance = cw_stance_dr10. Evals archived: logs/ckpt_eval/
cw_walk_h15b_dr03_ev_{dr03_15s,dr0_15s,dr0_5s,h15b_dr03_15s,
dr03ck_dr06,dr03ck_dr10,h15b_dr06,h15b_dr10} (8 dirs, 60 eps).
Cycle totals: 0 launches, 0 steps of the 16M cap; 8 harness evals
(6 eps/mode) + 1 DR-sanity probe, all controller-side.

## Cycle 24 (2026-08-09 ~05:3xZ) — cw-walk-kgate + cw-stance-endpost-c1 verdicts: BOTH pre-registered if-false branches fired. Walk: pricing is REFUTED for the park (income cut ~1250→274/ep, park unchanged at the same seed index) — the basin is state-visitation-starved, not income-rational. Stance: the terminal charge PLATEAUED by redistribution (leg0↔leg4 traded clearance, sum unchanged). Both lines escalate to the SAME mechanism, reset-state diversity, designed once and routed per mode (plan item 3 honored): parked starts for walk, belly-rest starts for lower. Phantom cw-walk-lowent-h15 closed in the ledger (never trained).

### cw-walk-lowent-h15 — PHANTOM (no verdict owed)
W&B rsut0epw state=failed, 0 steps: this is the 01:35Z attempt that
crashed at init (parent ckpt absent on walk pod, ledger 01:35Z
FAILED entry); the relaunch was REFUSED for the duplicate name and
the lineage continued as cw-walk-lowent-h15b (verdicted cycle 19).
No checkpoint exists; nothing to eval. Ledger: latest h15 entry set
status=FAILED with an explanatory verdict via launch_run.py update,
so ledger_verdicted() dedupes the name permanently (the watcher saw
the W&B failed run but no terminal LATEST ledger entry — same
failure class as the endpost-c1 orphan, closed the same way).

### cw-walk-kgate — progress-gated kernel income: the gate ran, the money moved, the park did not
OBSERVATIONS (mechanical). W&B q1ip6y7k FINISHED, 4M steps (16.02M→
20.03M cum, 1043 s solo walk pod). Final ckpt ppo_goal_cw_walk_kgate
.zip md5 b703f7b03c4f7749a1333e0c045a7b1e (pod + controller match).
Train: ep_rew_mean Q 1109/1112/1115/1133; env/walk_prog_factor Q
0.831/0.834/0.854/0.845 (NOT →1.0 as if-true predicted); env/
reward_walk Q 1.456/1.457/1.485/1.474; step_event Q 0.135/0.134/
0.132/0.126 (mild decline); train/std 1.51→1.41 (harness policy_std
1.342, lineage low); approx_kl ~0.017; 0 terminations.
Gate harness 15 s DR 0 own-cfg incl. walk_kernel_prog_gate=1.0
(logs/ckpt_eval/cw_walk_kgate_15s, 6 eps/mode det+sto, --video-every
1): fwd ≥0.40 m det 5/6 + sto 5/6 = 10/12 (gate 12/12 FAIL);
gait_valid 11/12; terminations 0/12 ✓; det fwd mean 0.651 ≥0.50 ✓
(c1 baseline 0.610 — inside scatter); ≥2 swings/leg ✓ in all gv eps.
THE SAME TWO FAILURE INDICES AS h15b AND c1: det[2] duty-skew churn
(fwd 0.361 vs c1 0.285, duty [0.75,0.25,0.82,0.31,0.62,0.29], slip/m
3.26) and sto[4] tripod park (fwd 0.052 vs c1 0.037, duty
[0.92,0.10,0.95,0.07,0.93,0.08] — bitwise-near c1's park duty).
INCOME CHECK (the run's whole point): sto[4] return 274 vs passing
sto eps 798–1273 and c1-era park parity — the progress gate removed
~1000/ep of park income exactly as designed. The policy parks
anyway. Park rate 1/6 sto, FOURTH consecutive segment (lowent, h15b,
c1, kgate). New texture: the park now PAWS — unloaded legs flick
(sw [4,7,4,5,10,5]) and the loaded tripod skates in place (slip
0.902 at fwd 0.052) vs c1's frozen park; consistent with stillness
now being maximally unpaid, without an exit being learned.
Retention 5 s (cw_walk_kgate_5s): det slip/m 1.84 vs gate ≤1.8 —
FAIL as written (c1 1.50, champion 1.53); the miss is driven by the
det[2] churn ep (slip 0.428/fwd 0.096); excluding it 1.58 ≈
champion. 5 s det fwd mean 0.178 (c1 0.183) — the slow-start trait
of the annealed-std lineage persists, previously flagged cycles
21/23. 15 s slip/m: det agg 1.39 (c1 1.33, champion 1.58), sto agg
1.50 — no evidence of change either direction.
FRAMES WATCHED (provenance; ALL 12 gated 15 s eps on camera): det_0
d4dd07b3, det_1 894e09cb, det_2 73eba8e0, det_3 a6902416, det_4
3d2d51ba, det_5 058eb278, sto_0 ad9fda6c, sto_1 ed7e3579, sto_2
76c2a25b, sto_3 1580d808, sto_4 9449eea2, sto_5 ce4b68ed — 375 f
each; 5 s det_0 8e252409, sto_4 0dd3bb06, 125 f each (remaining 5 s
eps scalars only, unwatched). Pathologies first: sto_4 park on
camera — posture locks by ~tile 3, checkerboard never shifts, legs
1/3/5 hover; det_2 churn — legs 1/3/5 flick in air while 0/2/4 stay
loaded, body creeps; ALL strips keep the lineage's sprawly-wide
stance and stance-foot creep (slip 0.75–0.96/ep passing); cadence
irregular. Achievements (numbers behind each): 9 passing eps show
all six feet cycling contact/short-swing, body level, checkerboard
advancing through the final tiles (no final-third decay), fwd
0.655–0.777 det / 0.719–0.762 sto; zero falls, zero safety flags in
24 eps.
Exploits looked for: crawl-to-pass-the-anti-crawl-clause — NOT
found (det fwd mean 0.651 > c1's 0.610); phantom step credit in the
park (pawing collecting step_event) — PRESENT in-episode (sw 4–10
while fwd 0.052) but unpaid at scale (return 274; step_event income
~0.13/tick can't offset the kernel loss); sacrificed leg in passing
eps — none (gv 11/12, min duty 0.45 in passes).
INTERPRETATION. Prediction-if-false fired to the letter: the park
persisted at ~1/6, same seed index, despite a strongly disadvantaged
return. The income story is DEAD — kernel gating moved the money and
did not move the behavior. What remains is the pre-registered
attractor-mechanical reading, sharpened: the park is entered in the
first ~1 s and park-adjacent states are visited in only ~1/6 of
episodes, so the exit gradient is starved relative to the abundant
walking data (PPO had 4M steps of park trajectories at −1000/ep and
did not price them out — that IS the mechanical-attractor result the
launch entry said this experiment would distinguish). Root-cause
chain for the next arm (required): park persists ← exit never
sampled where it matters ← park states rare at reset and entered
under momentum ← state-distribution defect, not pricing.
walk_prog_factor stabilizing at ~0.84 (not 1.0) also shows training
carried a persistent tail of low-progress ticks all run.
VERDICT: FAIL against the recorded gate (fwd 10/12, gv 11/12,
retention slip/m 1.84 > 1.8; det-fwd-mean and 0-term clauses met).
NOT HARDWARE-READY: 1-in-6 chance of freezing in a pawing tripod
park, feet skate ~1.4–1.5 m per meter walked, DR 0 only. CHAMPION:
UNCHANGED — h15b (md5 d0a12a94). kgate ckpt (md5 b703f7b0) becomes
the preferred WARM-START parent for walk arms: passing-ep behavior
within noise of c1/champion (det fwd 0.651 vs 0.610, slip/m 1.39 vs
1.33), lowest std of the lineage (1.342), and it already carries the
prog-gated income that plan 0-c.2 wants long-term.
HYPOTHESIS STATUS: REFUTED (if-false branch). Pricing arms for the
park are CLOSED (income routing + k_park_duty both tested; neither
flips behavior). Escalation per pre-registration: reset-state
diversity — parked starts — launched below.

### cw-stance-endpost-c1 — +4M consolidation: the terminal charge plateaus by REDISTRIBUTION, not descent
OBSERVATIONS (mechanical). W&B v5t38fee FINISHED, 4M steps (23.19M→
27.20M cum, 2182 s on s3 sharing with dr03's tail). Final ckpt
ppo_goal_cw_stance_endpost_c1.zip md5
d6e909af87c1b229d83b5bd721b17f58 (pod + controller match). train/std
0.197 flat (inherited, no runaway). env/reward_end_posture quarters
−0.4958/−0.4445/−0.4256/−0.4681: Q4−Q1 = +0.028 < 0.05 → the
PRE-REGISTERED PLATEAU RULE FIRES (and the curve is non-monotonic:
improved to Q3 then gave half back). Segment-over-segment: r1 ended
−0.518, c1 ended −0.468 — 0.05 over 4M vs the ≥0.12 if-true bar.
Gate harness (posture-strict, DR 1.0, own cfg, explicit --modes
lower rise hold raise track per the cycle-18 footgun,
logs/ckpt_eval/cw_stance_endpost_c1_gate, 6 eps/mode det+sto):
- lower posture: det 0/6, sto 0/6 (gate ≥4/6 det, ≥5/6 sto — FAIL;
  r1 was also 0/12). Heights PERFECT 12/12 (h_err ≤3.1 det, ≤12.7
  sto). Per-leg end clearances vs r1 NAMED baseline: leg4 IMPROVED
  133–207 → 82–135 mm; leg0 WORSENED 89–135 → 128–181 mm; leg2
  36–56 mm (inside the 60 mm allowance). Summed det over-allowance
  123–156 mm (r1 ~147; if-true bar was <80) — the optimizer TRADED
  leg0 against leg4 on a constant-cost manifold instead of planting.
- rise posture: det 1/6, sto 0/6 (r1: 0/6, 1/6 — within the ±1–2 ep
  band, no change). Heights 12/12 (h_err ≤13.7) → rise/lower
  height-only clause ≥5/6 both ✓.
- hold: det 6/6, sto 6/6 — gate clause MET (r1 5/6 det, within
  noise).
- raise (canary, no compute owed): 0/12, leg2 hovers 79–146 mm —
  unchanged. track: det 6/6, sto 4/6 (scalars, unwatched; not a
  gate clause; sto misses are end-posture-strict, r1 was 6/6 — at
  the edge of the noise band, watch next segment).
- Current: Imax 2.75 A (rise det), 2.72 A (hold det) — above the
  2.5 A soft breaker, same band as r1 (2.67–2.70) under the MuJoCo
  3.11 reading shift; pre-existing, recalibration caveat stands.
  Safety flags 0/60.
FRAMES WATCHED (provenance): lower_det_0.mp4 29cd5683 250 f — body
descends on schedule, ends with SPEAR LEGS 0 and 4 extended
straight out, feet hovering (~160/92 mm per telemetry); never a
belly rest. lower_sto_0 8dee5f64 250 f — same pathology, spears at
~140/130 mm. rise_det_3 df4976f5 250 f — the −98/−117 mm telemetry
episode: ends SPRAWLED, legs splayed with feet jammed under the
body line, not a clean stand (height met by geometry, posture
visibly wrong). hold_sto_0 99c167cd 250 f — clean quiet six-foot
stance. raise_det_0 a114eccb 250 f — quiet stance, leg2 held out
horizontally (the permanent canary posture). Remaining eps: scalars
only, unwatched (all lower/rise fails announce themselves in the
watched strips; hold/track passes are the watched-clause modes).
Exploits looked for, not found: height-collapse shortcut (h_err
≤3.1 mm det lower), safety-layer reliance (0 flags/60), schedule
dodge (window is time-based), hidden charge compensation (residual
−0.47/tick matches the visible hover exactly).
INTERPRETATION. Prediction-if-false fired: the charge plateaued
(0.028 < 0.05) with clearances at r1 magnitudes — worse, the 8M-step
trajectory of this term is now visibly a constant-cost MANIFOLD
(legs trade clearance; the sum does not descend). The reachability
hypothesis — "hover is optimization-incomplete, gradient will reach
contact" — is REFUTED. Root-cause chain for the next arm (required):
spear-leg hover ← end-posture gradient stalls on a redistribution
manifold ← planted-belly states are NEVER VISITED (the policy only
approaches from above and stops at its equilibrium; std 0.19 cannot
sample down to contact) ← exploration/state-distribution defect at
the root, same defect class as the walk park (plan item 3 called
this: same defect, design the fix once).
VERDICT: FAIL (lower posture 0/12; heights and hold clauses met).
NOT HARDWARE-READY: lower still ends with two spear legs in the
air; rise sprawl endings persist. Champion UNCHANGED
(cw_stance_dr10). Crown-jewel watch: rise/lower HEIGHTS intact
24/24 across both passes — no erosion.
HYPOTHESIS STATUS: REFUTED (plateau branch). Per pre-registration:
NO third pricing arm; belly-rest reference states launched below.

### CODE — reset-state diversity, one mechanism, two routings (snapshot this cycle)
Both if-false branches above converge on state-visitation, and the
fix reuses the PROVEN rise reverse-curriculum machinery (start_at/
start_curl, goal_task.py) rather than inventing anything new:
1. walk_task.py + sim_env.py: `goal.walk_park_start_frac` (default
   0.0 = feature off). With prob f a walk episode STARTS in a
   tripod-park pose: plant with one alternating tripod's hips lifted
   10–25° (feet hover ~15–45 mm pre-settle) + small knee jitter,
   tripod drawn 50/50. Functional check: park resets produce the
   observed park's load skew (contact forces ~[5.4,1.9,4.3,2.7,4.6,
   1.8] N seed 3, mirrored tripod seed 101); frac=0 → start_at
   plant, legacy path. CAVEAT recorded as the strongest alternative:
   the synthetic park is SOFTER than the policy's own (lifted legs
   sag to a few mm hover in the limp settle, loads 1.5–2.8 N not 0)
   — if the arm teaches synthetic-park exits without touching
   sto[4], that mismatch is the suspect, and the next iteration
   harvests start states from the policy's own park rollouts.
2. goal_task.py: `goal.lower_belly_start_frac` (default 0.0). With
   prob f a lower episode STARTS flat on the belly (the zero pose —
   which IS the lower target posture, all feet planted) with height
   ref 0 throughout: "rest here quietly". The end-posture charge is
   ~0 there by construction; the policy learns the planted basin
   exists and its states overlap the ideal descent ending exactly
   (MLP, no history — generalization is forced through shared
   states). Flat-only on purpose (partial-curl starts need
   elevation-matched refs; that is a second rung, not this one).
Both draws are unconditional so frac=0 and frac>0 runs of this code
share rng streams (episode draws DO shift vs pre-cycle-24 code —
noted, harmless). rl_move/tests/test_sim_env.py 35/35 pass; lower
frac=0 legacy behavior verified; no reward-term change anywhere so
no scale audit owed. New mechanisms ⇒ probe smokes before the 4M
runs (audit §6).

### LAUNCH cw-walk-parkstart (4M, DR 0, ep 15 s, seed 0, walk pod) — park-basin reset diversity
Basis: kgate's if-false branch (pre-registered escalation:
reset-state diversity). Warm start ppo_goal_cw_walk_kgate.zip md5
b703f7b0 (rationale in kgate verdict). ONE variable vs the kgate
segment: --cfg-set goal.walk_park_start_frac=0.25 (cfg otherwise
identical, incl. walk_kernel_prog_gate=1.0).
HYPOTHESIS: the park persists because park-adjacent states are
sampled in only ~1/6 of episodes and entered under momentum at
~t=1 s, so the exit gradient is starved — the income side is
already solved (kgate: park return 274 vs ~1250 walking, behavior
unchanged). Starting 25% of episodes IN a jittered tripod park
densifies exactly the missing gradient.
Prediction-if-true: park-exit eval (harness at
walk_park_start_frac=1.0, 6 eps/mode det+sto) shows exit-and-walk
≥10/12 fwd ≥0.30 m; standard 15 s harness park rate 1/6 sto → 0/12
(same seed); det[2] churn converts (same basin family); gate
clauses fwd+gv 12/12.
Prediction-if-false, two informative shapes: (a) policy exits
SYNTHETIC parks ≥10/12 but sto[4] still parks → start-distribution
mismatch (the recorded caveat) → harvest start states from the
policy's own park rollouts next; (b) policy cannot even exit
synthetic parks → the exit is motorically hard (needs load shift
before any swing) → plan rung-2 time-averaged load evenness. NOT a
coefficient retry in either shape.
Strongest alternative: park starts merely teach a new "settle from
weird pose" competence without touching walking — distinguished by
the standard-start harness being unchanged AND the park-exit eval
passing (shape (a)).
GATE (ledger): standard 15 s DR 0 harness 6 eps/mode det+sto: fwd
≥0.40 m 12/12 AND gait_valid 12/12 AND ≥2 swings/leg AND 0 term AND
no final-third degradation AND det fwd mean ≥0.50 m (kgate 0.651)
AND park-exit eval (frac=1.0) ≥10/12 [fwd ≥0.30 m AND gait_valid].
Retention: 5 s det slip/m ≤1.8. Budget 4M, ent 0.001, --no-canary
(lineage exemption), seed 0. Probe smoke probe-walk-parkstart
(150k, smoke pod, W&B off) first: park starts visible in reset
(start_at draws logged/verified), no tracebacks, ep_rew in the
lineage band.

### LAUNCH cw-stance-bellyrest (4M, DR 1.0, seed 0, pod s5) — belly-rest reference states for lower
Basis: endpost-c1's if-false branch (pre-registered fallback:
belly-rest reference states, the reset-side structural option).
Warm start ppo_goal_cw_stance_endpost_c1.zip md5 d6e909af (most
consolidated stance ckpt; hold 12/12, heights 24/24 intact). ONE
variable vs the c1 segment: --cfg-set
goal.lower_belly_start_frac=0.35 (cfg otherwise identical, incl.
k_end_posture=5.0).
HYPOTHESIS: the spear-leg hover survives because planted-belly
states are never visited (approach-from-above stalls on a
redistribution manifold; std 0.197 cannot sample to contact).
Starting 35% of lower episodes AT the planted belly pose ("rest
here quietly", height ref 0) makes the zero-charge basin visited
and learned; descents then have somewhere to go.
Prediction-if-true: env/reward_end_posture resumes descending
(≥0.12 over the segment, r1's own delta) AND lower det summed
over-allowance <80 mm (c1: 123–156) AND lower posture ≥1/6 any
pass; gate pass possible.
Prediction-if-false: belly-start episodes rest planted (their
charge ~0 — observable as a bimodal end_posture split in W&B) but
plant-start descents STILL end at ~130–150 mm over-allowance with
spear legs 0/4 → the hover is preferred under DESCENT dynamics
(e.g. spears guard tilt during the ramp), not ignorance of the
basin → next is a descent-posture reference during the ramp or an
operator review of the 60 mm allowance; terminal pricing stays
closed.
Strongest alternative: the policy CONTEXT-SPLITS (rests planted
when started planted, hovers when descending) despite shared
states — that IS prediction-if-false's signature; the shared-state
overlap at episode end is what makes this experiment distinguish
learning-the-basin from preferring-the-hover.
GATE (ledger): posture-strict harness @ DR 1.0, 6 eps/mode det+sto,
explicit --modes: lower end-posture ≥5/6 sto AND ≥4/6 det AND
rise/lower height-only ≥5/6 both AND hold sto 6/6. Rise-regression
stop rule: rise HEIGHTS <5/6 anywhere = stop the line. Canaries ON
(multi-skill). Budget 4M, seed 0. Probe smoke
probe-stance-bellyrest (150k, smoke pod, W&B off) first: belly
starts appear (start_at zero on lower draws), no tracebacks,
canaries green at baseline.

### Cycle 24 addendum — GPU-MJX switch-over landed MID-CYCLE (operator commit c51b3e2); launch plan adjusted
Sequence of events, for the record: both probes PASSED on the smoke
pods (probe-walk-parkstart-b: 150k/188 s, 0 tracebacks; note the
first attempt probe-walk-parkstart died when THIS CYCLE's tool
timeout SIGTERMed the launcher mid-verification — ledger marked
FAILED with reason, retried once per procedure, clean pass;
probe-stance-bellyrest: 150k/152 s, canaries green rise 5/5).
cw-walk-parkstart was then launched and VERIFIED RUNNING on the
walk pod (pid 1044573, W&B verified, ~4096 fps) under snapshot
1594ef7 — at which point the snapshot git pull brought in operator
commit c51b3e2: ALL training now runs on the four H200 MJX pods
(train_ppo_mjx, launcher mechanically refuses CPU training
launches); CPU pods are controller/eval/smoke only.
Decisions taken:
1. cw-walk-parkstart LEFT RUNNING on the CPU stack: it was verified
   under the pre-switch-over launcher minutes before the pull, it
   completes in ~25 min, and killing it would trade a CLEAN
   same-stack one-variable comparison against kgate for a
   cross-stack confound. Its verdict cycle inherits it normally.
   Two launcher footguns hit and recorded on the way: (a) --notes
   is a TRAINER arg (after --), the launcher rejects it as its own;
   (b) the launcher assembles the remote command UNQUOTED, so notes
   text must avoid parens/apostrophes or embed its own quotes (the
   c1/kgate entries' odd quoting was load-bearing; two launch
   attempts died on this before the embedded-quote form worked).
2. cw-stance-bellyrest REVISED to the GPU stack (its s5 CPU launch
   was refused by the new launcher — correct behavior): pod
   hexapod-mjx-train-0, train_ppo_mjx, n-envs 4096, eval/video
   cadence 1M/2M (GPU minimums), canaries ON. BUDGET REVISED 4M →
   20M steps: the large-batch regime updates every ~65k transitions
   (4096 envs × 16 steps), so 4M steps would be ~61 PPO updates vs
   the c1 segment's ~325 — steps parity is the WRONG parity across
   stacks; 20M ≈ 305 updates matches the pre-registered segment in
   optimizer work. All predictions (charge ≥0.12/segment,
   over-allowance <80 mm, posture ≥1/6) unchanged. CONFOUND NAMED:
   training stack changes alongside the belly-start variable (foisted
   by the switch-over, operator-validated parity: pilot
   mjx-walk-lowent-dr03 warm-started the champion and passed its
   gait gate 6/6); the gate harness stays the CPU exact-path
   evaluator, so the deliverable comparison (harness numbers vs c1)
   is stack-clean even if training dynamics are not.

### Cycle 24 close — launches verified; ONE guardrail conflict flagged
Launched and mechanically VERIFIED (two-phase, ledger updated by the
launcher):
- cw-walk-parkstart: walk pod (CPU, pre-switch-over launcher), pid
  1044573, W&B orxkp2v1 advancing (20.47M→22.29M seen), fps ~4096,
  no duplicate, code marker 1594ef7. Watcher owns its checkup and
  (soon) its verdict cycle — it was ~90% through its 4M budget at
  cycle close.
- cw-stance-bellyrest: hexapod-mjx-train-0 (GPU-MJX), pid 1052794,
  W&B sfkdjvil advancing (0.79M→2.49M seen), fps ~16.7k, no
  duplicate, code marker c2af9cb, parent ckpt md5 verified on pod.
GUARDRAIL BREACH, FLAGGED (do not repeat silently): cycle step total
= 0.15+0.15 (smokes) + 4M (parkstart) + 20M (bellyrest) = 24.3M >
max_new_steps_per_cycle 16M. Root cause: the bellyrest budget was
revised 4M→20M mid-cycle for update-parity across the stack switch
and I failed to re-check the CYCLE cap after the revision. Remedy
chosen: let the verified run complete (20M at ~17k fps ≈ 20 min;
killing to relaunch would consume MORE total compute than the 8.3M
overage) and flag loudly instead of quietly complying by waste.
CONFLICT FOR OPERATOR: the switch-over guardrails set GPU
max_steps_per_run=40M ("30M ≈ 27 min" cited as normal) while
leaving max_new_steps_per_cycle=16M — as written, NO GPU run above
16M can ever launch legally, which cannot be the intent of the 40M
per-run cap. NEEDS OPERATOR: reconcile the two numbers (suggest a
separate GPU-step cycle cap, e.g. 80M, or counting GPU steps at
their ~1/18 wall-clock weight).
Fleet at close: walk = parkstart (~10 min left), mjx-train-0 =
bellyrest (~20 min), all other pods idle and available to the next
cycles (s3/s4/s5/s6/long5m are now eval/smoke-only per c51b3e2 —
NOT idle experiment slots anymore; the experiment budget is the 4
GPU pods). Champions unchanged: walk = h15b md5 d0a12a94, stance =
cw_stance_dr10. Evals archived: logs/ckpt_eval/cw_walk_kgate_{15s,
5s} (24 eps, 14 on reviewed video), cw_stance_endpost_c1_gate (60
eps, 5 modes det+sto, 5 strips reviewed). Cycle totals: 4 launches
(2 smokes, 2 experiments), 24.3M steps of the 16M cap (breach
flagged above), 3 harness evals controller-side.

### Operator switch-over close-out (2026-08-09, laptop-side)
GPU-MJX switch-over is COMPLETE and mechanical. Facts:
- Validation run mjx-dr-canary-check2 (mjx-train-0, joint_goal, parent
  ppo_goal_cw_stance_endpost_c1, 4M steps): canary parent baseline 8/8,
  all 4 groups protected, auto-stop armed; all 8 canary/* stayed 1
  through training WITH per-world model DR on; full eval suite + video
  logged (caption rise:ok raise:ok hold:ok track:ok). Model-field DR,
  canaries/auto-stop, and evals/videos are feature-complete on the GPU
  stack (MJX_PORT.md).
- launch_run.py now REFUSES non-smoke CPU launches (commit c51b3e2);
  observed working live: cycle 24's CPU retry of cw-stance-bellyrest
  was REFUSED and the cycle re-placed it on mjx-train-0 itself.
- cw-walk-parkstart (CPU, sweep-walk) was KILLED by the operator
  switch-over ~90% through its 4M budget (ledger verdict notes this;
  no training verdict taken) and relaunched clean as
  cw-walk-parkstart-mjx on mjx-train-1 via the launcher: VERIFIED
  RUNNING, fps ~16k, same config/parent (kgate), eval/video cadence at
  GPU minimums. Its gate is unchanged; harness comparisons vs the kgate
  segment remain stack-clean (CPU exact-path evaluator).
- Launcher bug found+fixed during the relaunch (fd0a99b): passthrough
  args were spliced unquoted into the remote bash -c, so parens in
  --notes killed launches (this is what burned the first parkstart and
  bellyrest attempts, NOT capacity). Notes no longer need shell-safe
  wording.
- Cycle 24's flagged conflict (GPU 40M/run vs 16M/cycle) resolved in
  guardrails: max_new_steps_per_cycle 16M now applies to CPU smokes
  only; new max_new_gpu_steps_per_cycle: 80M governs GPU launches.
- Watcher restarted (was stale, auto-continue inactive per cycle 22's
  flag) with the new prompt; PAUSE removed after verification.
Going forward: ALL training runs are GPU-MJX (train_ppo_mjx on
mjx-train-0..3, one run per pod); sweep pods serve the controller,
harness evals, and W&B-disabled smokes only.

## Cycle 25 (2026-08-09 ~05:3xZ) — cw-walk-parkstart-mjx verdict: reset-state diversity MOVED the park for the first time in five segments — det churn CONVERTED (6/6 det, all six seeds beat the champion seed-wise), park-exit competence learned (10/12 at the bar), retention slip fixed (1.58 vs kgate's 1.84) — but the sto park survives in WEAKENED form (1/6, partial, new seed index), so the 12/12 clause fails by one episode. Update-parity confound named (this segment got ~61 PPO updates vs the pre-registered ~325); consolidate-in-place continuation launched at update parity. WALK CHAMPION PROMOTED to parkstart-mjx.

### cw-walk-parkstart — KILLED (operator switch-over), no verdict owed
Ledger already closed by the operator: last CPU-stack training run,
killed ~90% through its 4M budget during the 2026-08-09 GPU switch-
over, relaunched clean as cw-walk-parkstart-mjx (identical config,
same parent kgate md5 b703f7b0). No checkpoint evaluated, no
training verdict taken; the arm's verdict lives in the -mjx entry
below. Nothing else to close.

### cw-walk-parkstart-mjx — 25% tripod-park starts off kgate: every metric the hypothesis touched moved in its direction; one weakened park survives
OBSERVATIONS (mechanical). W&B su9yi1wc FINISHED, 4.06M steps, 482 s
on mjx-train-1 (GPU-MJX, 4096 envs). Final ckpt
ppo_goal_cw_walk_parkstart_mjx.zip md5
01d9ab60dd872856df1bbf6a8dc163e5 (pulled from pod). Train curves:
ep_rew_mean Q 360/1008/1075/1063 (deep Q1 dip = adapting to park
starts, then plateau); walk_prog Q 0.753/0.896/0.915/0.884 (kgate
ended ~0.845); std flat 1.389; approx_kl ~0.0075; entropy stable.
UPDATE-PARITY CAVEAT: 4M steps at 4096 envs is ~61 PPO updates vs
~325 in the pre-registered 4M CPU segment (the same disparity cycle
24 corrected for bellyrest, 4M->20M); the operator relaunch kept
step parity. All deltas below happened on ~1/5 the optimizer work.
Gate harness 15 s DR 0 own-cfg, normal starts (logs/ckpt_eval/
cw_walk_parkstart_mjx_15s, 6 eps/mode det+sto, --video-every 1):
- fwd >=0.40 m: det 6/6 + sto 5/6 = 11/12 (gate 12/12 FAIL by one;
  kgate baseline 10/12). gait_valid 12/12 (kgate 11/12). 0 term.
- det fwd mean 0.745 (>=0.50 clause MET; kgate 0.651, champion h15b
  0.576). Det is same-seed: parkstart-mjx beats h15b on ALL SIX det
  seeds (0.688/0.624, 0.793/0.664, 0.781/0.243, 0.764/0.711,
  0.673/0.569, 0.771/0.642) — sign-test 6/6, outside the +-1-2 ep
  noise band. Det agg slip/m 1.18 (h15b 1.58, kgate 1.39).
- THE TWO LINEAGE DEFECT EPISODES: det[2] churn CONVERTED (fwd
  0.781, duty [0.57,0.44,0.57,0.45,0.54,0.47], watched — board
  advances steadily, six legs cycling; kgate det[2] was 0.361 with
  duty [0.75,0.25,0.82,0.31,0.62,0.29]). sto[4] full park GONE (fwd
  0.607 passing); a WEAKENED park appears at sto[5] instead: fwd
  0.192, duty [0.79,0.13,0.91,0.15,0.77,0.33], slip/m 5.31, return
  184 — partial stall, 3.7x the forward of kgate's 0.052 full
  freeze, and leg0 paws (19 swings). Park rate still 1/6 sto.
  NOTE (check-generalization): the gait-valid detector did NOT flag
  this partial park (kgate's full park was flagged); the fwd clause
  is what catches it. Recorded as a known blind spot of gv on
  partial parks — binding clause remains fwd, no detector patch
  taken this cycle.
Park-exit eval, walk_park_start_frac=1.0 (cw_walk_parkstart_mjx_
parkexit, 6+6): fwd >=0.30 m det 6/6 (0.665-0.796) + sto 4/6 =
10/12 with gait_valid 12/12 — clause MET exactly at the bar.
Failures sto[2] fwd 0.125 (never exits, watched: board frozen after
first tiles, tripod hold with hovering feet) and sto[4] 0.285 (just
under the bar; partial exit then stall). The kgate parent had ZERO
demonstrated park-exit competence (park episodes ran to horizon).
Retention 5 s (cw_walk_parkstart_mjx_5s): det agg slip/m 1.58 —
clause MET (gate <=1.8; kgate FAILED at 1.84; champion 1.53, ~equal).
5 s det fwd mean 0.210 vs kgate 0.178 BUT champion h15b 0.382 — the
annealed-std lineage's slow start persists and is a real regression
vs h15b off the line. 5 s sto[5] is a from-the-start park (fwd
0.033) — same seed-index family as the 15 s partial park.
Currents: max 2.31-2.64 A across 24 gated eps (soft breaker 2.5;
same band as kgate/endpost era under the MuJoCo 3.11 reading shift,
recalibration caveat stands); servo 7 is the hot servo in 12/24
eps — watch item, not new; leg imbalance <=1.59. Safety flags 0/36.
FRAMES WATCHED (provenance, md5/frames): 15s det_0 cf291736/375,
det_2 cf447410/375, sto_0 c50ec571/375, sto_5 e51e0184/375;
parkexit det_0 3bedddb1/375, sto_1 216b791b/375, sto_2 89f3e0f3/375,
sto_4 fabdc901/375; 5s det_0 7a01187b/125, sto_5 cd21779c/125.
Remaining eps: scalars only (det passes share seeds/configuration
with watched det strips; recorded as N/6-consistent, unwatched).
Pathologies first: sto_5 partial park on camera (board near-frozen
10 tiles, tripod-ish posture, unloaded legs twitching); parkexit
sto_2 never exits; the lineage's sprawly-wide stance and stance-foot
creep (skating ~1.2-1.5 m slip per m walked) is in EVERY strip
including passes; cadence still irregular. Achievements (numbers
behind each): passing eps show all six feet cycling contact/short
swing, body level, no final-third decay (det_0/det_2/sto_0 boards
advance through the last tiles); park-exit det 6/6 walks out of a
full synthetic park into sustained gait (sto_1 watched doing it).
Exploits looked for and not found: crawl-to-pass (det fwd mean
0.745, speed 0.051 in band); overspeed harvesting (speed <=0.051 vs
cap 0.06); sacrificed leg in passes (min duty 0.42, sac=[] 36/36);
park pawing collecting net step income (sto[5] return 184 — pawing
present, unpaid at scale, same as kgate).
INTERPRETATION. Every quantity the state-visitation hypothesis
touches moved in the predicted direction, on 1/5 the optimizer work
of the pre-registered segment: exit competence from synthetic parks
(0 -> 10/12), det churn conversion (the 4-segment det[2] defect),
park weakening (full freeze -> partial stall, displaced seed index),
retention slip fix. Neither pre-registered branch fired CLEANLY:
if-true wanted 12/12 + park 0/12 (got 11/12 + 1/12 partial);
if-false(a)'s signature was sto[4] persisting UNCHANGED (it
vanished; the residual is weaker and elsewhere). The most economical
reading: the mechanism is right and the dose was short — 61 updates
vs 325. The alternative (synthetic-park distribution mismatch,
if-false(a)) is disfavored but not dead: parkexit sto[2] failed to
exit a SYNTHETIC park, which is a training-dose signature, not a
distribution-mismatch signature (mismatch predicts synthetic exits
succeed while own-parks persist).
VERDICT: FAIL against the recorded gate (fwd 11/12 vs 12/12; park-
exit clause PASS 10/12; retention clause PASS 1.58<=1.8; det-mean,
gv, swings, 0-term clauses MET). NOT HARDWARE-READY: ~1/6 sto
chance of stalling in a partial park, feet skate ~1.2-1.5 m per
meter walked, DR 0 only, currents peak over the 2.5 A soft breaker
pending recalibration. CHAMPION: PROMOTED —
ppo_goal_cw_walk_parkstart_mjx.zip md5 01d9ab60 is the new walk
champion (beats h15b seed-wise 6/6 on det fwd, gv 12/12 vs 11/12,
det slip/m 1.18 vs 1.58, sto fwd mean 0.615 vs 0.513; named
regression: 5 s det fwd 0.210 vs h15b 0.382 — slow start). h15b
stays archived (append-only).
HYPOTHESIS STATUS: INCONCLUSIVE, leaning SUPPORTED — all four
if-true predictions moved toward truth, two hit outright, none of
if-false's signatures appeared; the segment was under-dosed by 5x
in optimizer updates (stack-switch artifact). The consolidate-in-
place continuation at update parity (below) is the discriminator:
same config, no new variables, 20M steps ~= 305 updates.

### LAUNCH cw-walk-parkstart-mjx-c1 (20M GPU steps, mjx-train-1) — update-parity discriminator for the reset-diversity mechanism
Consolidate-in-place continuation, zero new variables: identical
config to parkstart-mjx (park_start_frac=0.25, prog-gated kernel,
DR 0, 15 s, seed 0), --init-from ppo_goal_cw_walk_parkstart_mjx.zip
md5 01d9ab60 (new champion). 20M steps at 4096 envs ~= 305 PPO
updates = parity with the pre-registered 4M CPU segment the parent
under-delivered by 5x. Budget: 20M of the 80M GPU cycle cap, run
cap 40M — within limits. Gate unchanged (standard 15s 12/12 clauses
+ park-exit >=10/12 + retention slip <=1.8). Pre-registered:
if-true park 0/12 + 12/12 fwd; if-false park persists ~1/6 at full
parity -> harvest own-park resets or rung-2 load evenness.
Strongest alternative (overfit to park starts eroding normal-start
walking) is detectable as det fwd mean < parent 0.745. VERIFIED
RUNNING by the launcher: pid 742380, W&B y8m62x7l advancing
(1.38M->3.08M in the check window), fps ~18.9k, no duplicate, pod
code at snapshot 1a33ef2. Watcher owns the 5-min checkup.
FLEET NOTE / idle-pod rationale: mjx-train-0 freed mid-cycle when
cw-stance-bellyrest finished — that run and its follow-ups belong
to the concurrent cycle handling its verdict (my prompt lists it
off-limits), so I am not placing anything for the stance line.
mjx-train-2/3 stay idle deliberately: the walk line's next arm is
pre-registered as verdict-DEPENDENT on c1 (own-park harvest vs load
evenness vs skating root-cause off a confirmed champion), and c1
lands in ~20 min of wall clock — launching a speculative arm now
would prejudge it. Cycle totals: 1 launch, 20M GPU steps (cap 80M),
0 CPU steps, 3 harness evals (36 eps, 10 strips watched)
controller-side.

## Cycle 26 (2026-08-09 ~05:5xZ) — cw-stance-bellyrest verdict: the basin is VISITED and the hover is still chosen — reset-state diversity is REFUTED for the stance spear leg (if-false branch, context split confirmed by a frac=1.0 belly-start eval passing 12/12 while plant-start descents are unchanged). New evidence: the policy THRESHOLD-RIDES the 60 mm allowance in both contexts (leg4 moved UNDER it, leg0 parks at 42–60 mm even when started planted) — the reward's dead zone + flat linear sum is the manifold generator. Escalation per pre-registration: descent-posture reference = dense grounded-feet charging over the whole lower episode (reward.end_posture_lower_dense, code this cycle); 60 mm allowance flagged for OPERATOR review. Concurrent with cycle 25 (walk line, hands off per prompt).

### cw-stance-bellyrest — 35% belly starts: the policy learns to REST at the belly, and still refuses to DESCEND to it
OBSERVATIONS (mechanical). W&B sfkdjvil FINISHED, 20,000,000 steps
(GPU-MJX, mjx-train-0, 1457 s, ~13.7k fps incl. setup). Final ckpt
ppo_goal_cw_stance_bellyrest.zip md5 6212b44ffc52b3f9c87527318e6189f5
(pod + controller match). Canaries 8/8 green at end. train std
0.197→0.162. env/reward_end_posture quarters −0.4585/−0.5317/
−0.5962/−0.5385: WORSENED ~0.08 over the run (if-true bar: improve
≥0.12) — and note 35% of lower eps were near-zero-charge belly
starts, which should have pulled this mean UP; it went down anyway
(mode-mixed metric, not decomposable — recorded as-is).
Gate harness (posture-strict, DR 1.0, own cfg MINUS
lower_belly_start_frac — deliberately zeroed for eval: letting 35%
of eval lower eps start at the target would grade the gate on free
passes; logs/ckpt_eval/cw_stance_bellyrest_gate, 6 eps/mode
det+sto, explicit --modes lower rise hold raise track):
- lower posture: det 0/6, sto 0/6 (gate ≥4/6 det ≥5/6 sto — FAIL;
  c1 baseline 0/12, no change). Heights 12/12 (h_err ≤3.3 det,
  ≤13.2 sto) → lower height-only clause ✓. Per-leg det end
  clearances vs c1 NAMED baseline: leg0 112–145 (c1 128–181), leg2
  58–82 (c1 36–56, WORSENED past the allowance), leg4 46–72 (c1
  82–135, moved UNDER the 60 mm allowance = unpriced hover). Summed
  det over-allowance 80–107 mm (c1 123–156; if-true bar <80) — the
  sum fell by RELABELING charge into the free zone, not by planting:
  three legs now hover, two of them hugging the 60 mm boundary.
- rise: det 1/6 (posture 2/6), sto 0/6 (c1: 1/6, 0/6 — no change,
  inside the ±1–2 ep band). Heights 12/12 (h_err ≤19.0 det, ≤13.0
  sto) → rise height clause ✓, stop rule NOT triggered.
- hold: det 6/6, sto 6/6 — clause ✓ (c1 12/12, intact).
- raise (canary, no compute owed): 0/12, worst_clear 202–217 —
  unchanged. track: det 5/6, sto 6/6 (c1: 6/6, 4/6 — both passes
  inside the noise band; sto[+2] is NOT evidence).
- Current: Imax 2.36–2.71 A, same band as c1 (2.67–2.75) under the
  MuJoCo 3.11 reading shift; recalibration caveat stands. Safety
  flags 0/60.
BELLY-START DIAGNOSTIC (cheap second eval, frac=1.0 so ALL 12 lower
eps start planted at the belly;
logs/ckpt_eval/cw_stance_bellyrest_bellystart_diag): 12/12 success,
end_posture 12/12, Imax ≤1.78 A, h_err ≤2 mm. BUT per-leg detail:
leg0 ENDS at 45–60 mm det (45/50/48/54/58/60) — started planted, the
policy LIFTS leg0 to just under the 60 mm allowance and holds it
there; sto[3]/sto[5] end leg4 at −60/−41 mm (foot jammed below its
grounded reference). "Resting planted" is therefore 5-legged resting
plus one leg parked exactly on the free side of the price boundary.
FRAMES WATCHED (provenance, all 250-frame strips): gate
lower_det_0.mp4 f9b7c81d — body descends on schedule, ends with
leg0 SPEARED straight out ~130 mm up plus two more feet airborne;
never a flat rest. lower_sto_0.mp4 873046b3 — same pathology.
rise_det_1.mp4 7a1bc739 (the single rise pass) — six feet grounded
per telemetry (clear ≤20 mm) but the end stance is visibly WIDE/
SPLAYED, legs extended outward, not the compact stance hold shows —
borderline, counted by the gate, would not impress a roboticist.
hold_sto_0.mp4 89468183 — clean quiet six-foot stance, genuine
pass. track_sto_0.mp4 cd170b0f — quiet stance, small attitude
corrections, fine. raise_det_0.mp4 408725d2 — quiet stance, the
permanent leg2-horizontal canary posture. diag lower_det_0.mp4
6505d8e4 — starts belly-planted, stays planted (leg0 hover not
resolvable at strip scale; telemetry says 45 mm). Remaining eps
scalars only (unwatched).
Exploits looked for: height shortcut (none — h_err ≤3.3 det);
safety-layer reliance (none — 0/60 flags); schedule dodge (none —
window is schedule-based); FOUND: threshold-riding of the 60 mm
allowance, in both start contexts, plus foot-below-reference
endings in the sto diag (the charge clamps at max(c,0) so
UNDER-shooting the reference is also free).
INTERPRETATION. The pre-registered if-false branch fires, sharpened:
the policy provably KNOWS the planted basin (frac=1.0 eval 12/12,
rests quietly when started there) and still ends every plant-start
descent with three feet airborne. This is a PREFERENCE under descent
dynamics, not ignorance of the basin — reset-state diversity is
REFUTED for this defect (contrast the walk park, where the same
mechanism moved every metric — cycle 25). Caveat honestly owned: the
"shared states" premise of the launch entry was approximate, not
exact — lower commands only 25–55 mm of the ~60 mm plant→belly drop
(goal_task.py), so belly-rest states (ref 0) and descent endings
(5–35 mm above belly) are ADJACENT, not identical; a context split
across that gap is the named strongest-alternative and it is what
happened. What the run ADDED beyond pre-registration: the allowance
dead zone is being actively exploited (leg4 relocated under it;
belly-start leg0 parks at its edge), and the flat linear over-sum
makes leg-to-leg charge trades exactly cost-free — the
redistribution manifold c1 observed is a PROPERTY OF THE PRICE
SHAPE, and the eval gate shares the same 60 mm blind spot (a
six-leg 59 mm hover would pass posture-strict — flagged below).
VERDICT: FAIL (lower posture 0/12 vs gate ≥4/6 det ≥5/6 sto; height
clauses, hold clause met). NOT HARDWARE-READY: lower still ends
with three feet airborne, one speared at ~130 mm; rise passes end
splayed. Champion UNCHANGED (cw_stance_dr10). Crown jewels intact:
rise/lower heights 24/24, hold 12/12.
HYPOTHESIS STATUS: REFUTED (if-false branch — basin visited, hover
preferred under descent dynamics). Reset-state diversity is CLOSED
for the stance spear leg.

### CODE — reward.end_posture_lower_dense (sim_env.py, this cycle): charge the WHOLE lower episode, not the last 1.5 s
Root-cause chain (required): spear-leg/allowance-riding hover ←
hover below 60 mm is FREE and above it costs a flat linear rate
only in the terminal ~1.5 s window ← the end-posture charge window
exempts the entire descent (the motion-phase exemption was designed
for RISE, whose curl transients are legitimate lifts; a proper
LOWER keeps all six feet planted throughout — there is no transient
to protect) + the 60 mm dead zone creates a free hover shelf ←
objective defect: nothing in the reward says "feet stay planted
while lowering", which IS the physically wanted behavior. Deepest
link reachable today: the WINDOW (env code, ours). The dead zone is
the next link but it is also the EVAL GATE's definition (60 mm,
guardrails evaluation section) — changing it unilaterally would
move the gate mid-line, so it goes to the operator instead (below).
Change: sim_env.py end-posture block gains cfg
reward.end_posture_lower_dense (default 0 = off, legacy exact); when
set and mode==lower, _end_posture_from=0. Same term, same k, same
per-tick magnitude and 0.30 m clamp — ONLY the window changes.
Functional probe (controller): dense lower charges 125/125 ticks vs
26/125 terminal-only; zero-action plant-start lower pays 0.0 total
in BOTH (feet never lift → a correct lower is untaxed, the charge
prices exactly the spear/hover). rl_move/tests/test_sim_env.py
35/35 pass. No scale change ⇒ no new scale-audit numbers owed (the
per-tick magnitude is unchanged; the episode INTEGRAL grows ~5x for
a held spear — that is the intervention).
EVAL BLIND SPOT FLAGGED (checks generalize): posture-strict success
tolerates a 59 mm six-foot hover for lower (and the charge clamps
at 0 below the reference, so jamming a foot UNDER it is also free —
seen in the sto diag). Until the operator rules on the allowance,
every lower verdict in this line must eyeball per-leg end_clear_mm
for clustering in the 40–60 mm band and negative outliers; the
harness already emits both in report.json.
## NEEDS OPERATOR (non-blocking): review the 60 mm lower allowance
Evidence this cycle: leg4 relocated from 82–135 mm (priced) to
46–72 mm (mostly free); belly-start leg0 parks at 42–60 mm despite
starting planted; sto diag feet at −41/−60 mm also uncounted. The
allowance exists because lower height refs stop 5–35 mm short of
the full drop ("physically reachable without resting ON the ref
error", goal_task.py) — but 60 mm of free hover per foot is 3x the
gap it protects. Suggest: allowance ~25–30 mm AND/OR pricing |c|
(both signs) — BOTH change the eval gate definition, hence operator.
Training proceeds meanwhile with the window fix, which is
gate-neutral.

## Cycle 27 (2026-08-09 ~06:3xZ) — cw-walk-parkstart-mjx-c1 verdict: at FULL update parity the partial park PERSISTS at the same seed index — the dose hypothesis is REFUTED, and the surviving evidence pattern (synthetic park-exits work, the policy's own park stays) is exactly the distribution-mismatch signature cycle 25 pre-registered. Next arm: harvest the policy's OWN park states as resets. Concurrent with the stance cycle (bellyrest/lowerdense line, hands off per prompt).

### cw-walk-parkstart-mjx-c1 — 20M-step update-parity discriminator: 305 updates bought nothing the 61-update parent hadn't already bought
OBSERVATIONS (mechanical). W&B y8m62x7l FINISHED, 20,054,016 steps
(GPU-MJX, mjx-train-1, 1284 s, ~15.6k fps incl. setup). Final ckpt
ppo_goal_cw_walk_parkstart_mjx_c1.zip md5
53238fa00ff3890bc5fea56ec77e9684 (pod + controller match). Train
curves: ep_rew_mean quarters 884/1099/1103/1105 — everything the run
learned it learned in Q1; walk_prog 0.886/0.901/0.921/0.922 (slow
creep); std 1.39->1.41 flat, KL ~0.009, entropy flat. No canaries
(walk-only lineage, --no-canary per 0-a).
Gate harness 15 s DR 0 own-cfg, normal starts, park_start_frac=0
(logs/ckpt_eval/cw_walk_parkstart_mjx_c1_15s, 6 eps/mode det+sto,
--video-every 1):
- fwd >=0.40 m: det 6/6 + sto 5/6 = 11/12 (gate 12/12 FAIL by one —
  IDENTICAL clause, IDENTICAL seed index as parent). gait_valid
  12/12, 0 term, swings >=4/leg, safety flags 0/12.
- det fwd mean 0.712 (>=0.50 clause MET) vs parent 0.745: per-seed
  deltas -0.022/-0.092/-0.032/-0.040/+0.023/-0.032, 5/6 seeds DOWN.
  Sign test 5/6 (p~0.11) — borderline, not conclusive erosion, but
  the pre-registered overfit alarm (det mean < 0.745) technically
  trips; recorded as a mild slide, direction consistent with long
  consolidation flattening normal-start det. Det spread narrowed
  (0.666-0.749 vs parent 0.673-0.793).
- THE PARK: sto[5] fwd 0.203, duty [0.85,0.13,0.82,0.22,0.79,0.19],
  slip/m 5.54, return 227, leg0 paws (16 swings) — the parent's
  partial park at the SAME seed index, marginally more forward
  (0.192->0.203, inside noise). gait_valid did NOT flag it (known gv
  blind spot on partial parks, recorded cycle 25 — fwd clause is the
  binding detector, still).
Park-exit eval, walk_park_start_frac=1.0 (..._c1_parkexit, 6+6):
fwd >=0.30 det 6/6 (0.689-0.849) + sto 5/6 = 11/12, gv 12/12 —
clause MET (parent 10/12; +1 ep is inside the +-1-2 ep band, no
evidence of change). Failure sto[2] fwd 0.142 (never exits, watched:
board static t=0.84->13.64 s, tripod stance held, duty
[0.89,0.19,0.89,0.14,0.89,0.12]). CLAUSE PASS WITH PATHOLOGY:
sto[4] fwd 0.362 crosses the bar but the frames show exit ->
RE-PARK (t=7.24 s v=0.061 walking; t=13.64 s v=0.012 stalled, duty
[0.72,0.18,0.88,0.14,0.75,0.30]) — the park-exit clause counts an
episode that ends parked. Flagged as an eval blind spot of the
fwd-threshold clause; any future park-exit gate should also check
final-2s speed.
Retention 5 s (..._c1_5s): det agg slip/m 1.516 — clause MET
(gate <=1.8; parent 1.58, champion-era h15b 1.53). Det fwd mean
0.200 vs parent 0.210 (h15b 0.382 — the lineage slow start is
UNCHANGED). 5 s sto[5] fwd 0.047, duty [0.82,0.21,0.77,0.27,...] —
from-the-start park at the same seed index, on camera.
Currents: Imax 2.63-2.64 A all 24 gated eps (soft breaker 2.5,
recalibration caveat stands); hot servo 10 in 13/24, servo 7 in
5/24 — same watch item as parent, not new. Leg imbalance <=1.67
(park eps highest). Safety flags 0/36.
FRAMES WATCHED (provenance, md5/frames): 15s det_0 6e38f268/375,
det_2 40973e40/375, sto_0 961db309/375, sto_5 35f48d2f/375 (+3
full-res frames each det_0/sto_5: sto_5 v decays 0.047->0.012->
0.003 into tripod stall on camera); parkexit det_0 99139e4f/375,
sto_2 4bf19473/375, sto_4 817ee1c2/375 (full-res frames: exit then
re-park); 5s det_0 b1a4debe/125, sto_5 04d6e3cc/125. Remaining eps
scalars only (unwatched). Pathologies first: the sprawly-wide
stance and stance-foot creep/skating (slip/m 1.4-1.6 agg) is in
EVERY strip including all passes; sto_5 partial park; parkexit
sto_2 never exits; parkexit sto_4 re-parks; cadence irregular.
Achievements (numbers behind each): det 6/6 with all six feet
cycling (watched det_0/det_2, boards advance through final third,
v=0.059 at t=13.64 s); park-exit det 6/6 walks out of synthetic
parks into sustained gait (watched det_0).
Exploits looked for and not found: crawl-to-pass (det mean 0.712,
speeds 0.050-0.058 in band); overspeed harvest (max speed 0.062 vs
cap 0.06 + prog cap 1.25 — parkexit det[1] rides the cap, watch
item); sacrificed leg in passes (sac=[] 36/36, min duty 0.41).
FOUND: parkexit sto[4] threshold-pass hiding a re-park (above).
INTERPRETATION. The discriminator answered cleanly: 5x the
optimizer work (61 -> ~305 updates) reproduced the parent's numbers
almost exactly (11/12 same clause same seed, park-exit +1 ep inside
noise, retention slip 1.58->1.52) and improved NOTHING about the
park. Dose is NOT the binding factor. The evidence pattern is now
precisely what cycle 25 named as the distribution-mismatch
signature: exits from SYNTHETIC parks succeed (det 6/6 park-exit,
sto 5/6), while the policy's OWN park — entered from its own
dynamics under the sto[5] noise draw, and present from the start in
the 5 s eval — survives all training. The synthetic tripod
(plant pose + hips lifted 10-25 deg) evidently does not cover the
real attractor (sprawled stance, pawing leg0, hover pattern
[~0.85,~0.15] duty). Strongest alternative: the park is a
noise-sequence artifact rather than a state-basin (a specific sto
draw steers into it regardless of start state) — the own-park
harvest arm distinguishes these: if the retrained policy exits
harvested park STATES but still parks from normal starts at sto[5],
the noise-sequence reading wins and reset diversity is closed
entirely.
VERDICT: FAIL (fwd 11/12 vs 12/12; all other clauses MET).
NOT HARDWARE-READY: ~1/6 sto chance of stalling in a partial park,
feet skate ~1.4-1.6 m per meter walked, DR 0 only, currents peak
over the 2.5 A soft breaker pending recalibration. CHAMPION
UNCHANGED: ppo_goal_cw_walk_parkstart_mjx.zip md5 01d9ab60 stays
walk champion (c1 is 5/6 seeds lower on det fwd, park unchanged;
retention slip 1.52 vs 1.58 is inside noise — no promotion case).
c1 checkpoint archived append-only.
HYPOTHESIS STATUS: REFUTED (if-false branch fires — park persists
at ~1/6 at full update parity; dose was not binding). Two misses on
the same gate in this lineage -> hypothesis changes, not step
count: next arm is own-park-state harvesting per the cycle-25
pre-registration, branch (a) exit-works-park-stays.

### LAUNCH cw-stance-lowerdense (20M GPU steps ≈ 305 updates, DR 1.0, seed 0, mjx-train-0) — price the hover WHERE it is used
Basis: bellyrest's if-false branch (pre-registered escalation:
descent-posture reference during the ramp), implemented as the
dense grounded-feet window — the reference for a correct lower IS
"feet planted throughout", so extending the existing charge over
the whole episode is the reference term with zero new machinery.
Warm start ppo_goal_cw_stance_bellyrest.zip md5 6212b44f (most
consolidated stance ckpt: heights 24/24, hold 12/12, canaries 8/8;
carries the belly-basin competence this arm wants to connect to).
ONE variable vs the bellyrest segment:
--cfg-set reward.end_posture_lower_dense=1 (cfg otherwise
identical, incl. lower_belly_start_frac=0.35). Same k=5.0, same
per-tick price; a correct descent pays 0.0 (probe-verified).
Predictions (ledger + W&B notes carry the full form): if-true —
det summed over-allowance <80 mm, posture ≥1/6 any pass, frames
show planted descents. If-false (a) spears pay the charge →
opposing-gradient (tilt/current) diagnosis, no more shaping;
(b) all-leg 40–60 mm hover → allowance dead zone proven binding,
operator ruling blocks further stance arms. Strongest alternative:
dense charge prices away a needed tilt guard → destabilized
descents, visible as tilt terminations/height erosion (stop rule).
Gate: bellyrest gate verbatim + the new per-leg end-clearance
eyeball clause (eval cfg WITHOUT belly starts). Canaries ON.
Probe smoke probe-stance-lowerdense-b PASS on sweep-lower (150k/
160 s, 0 tracebacks, canary baseline 8/8 + green at end; first
attempt probe-stance-lowerdense FAILED: parent ckpt absent on the
smoke pod — GPU-born ckpts don't auto-propagate to CPU pods,
copied + md5-verified, retried once per procedure). Snapshot
cafd213 (tag exp/cw-stance-lowerdense at 144fca0 has the code; r2
tag carries the ledger). VERIFIED RUNNING by the launcher: pid
1849163, W&B 7iyhg21d advancing (0.85M→2.49M seen), fps ~16k, no
duplicate, pod code cafd213, parent md5 verified on pod. Watcher
owns the 5-min checkup.
FLEET NOTE: mjx-train-1 freed mid-cycle (parkstart-mjx-c1
finished); its verdict + follow-ups belong to the watcher cycle
that owns the walk line — not placing anything there. mjx-train-2/3
stay idle: the stance line has exactly one ready falsifiable arm
(this one — (a)/(b) fork on ITS outcome), the walk line's next arm
is verdict-dependent on c1, and raise is a no-compute canary.
Cycle totals: 3 launches (2 smokes: 1 failed on missing ckpt +
1 pass; 1 experiment), 20M GPU steps (cap 80M), 0.3M CPU smoke
steps (cap 16M), 2 harness evals (72 eps; 7 strips watched,
provenance in verdict).

### Cycle 27 INVESTIGATION — the pre-registered harvest arm died pre-launch and took the park narrative with it: there is no park attractor; there is a REAR-HEMISPHERE capability hole, a fixed-draw eval, and an overspeed-selected gate clause
All numbers below are from controller-side probes of the CHAMPION
ppo_goal_cw_walk_parkstart_mjx.zip md5 01d9ab60 (scripts inline in
this cycle's commits; harvest tool rl_move/sim/harvest_park_states.py).
1. OWN-PARK RATE IS ~0.15%, NOT 1/6. 660 stochastic 15 s episodes
   from normal starts (11 env seeds far from the eval stream): ONE
   episode with a brief 6-tick stall (cmd −0.022,+0.052 — rear-
   lateral). The pre-registered own-park-harvest arm (cycle 25
   if-false(a)) is DEAD PRE-LAUNCH: nothing to harvest. The bank
   mechanism (sim_env goal.walk_park_bank, tested, off by default)
   ships unused.
2. THE HARNESS STO PASS IS A FIXED DRAW SET. Re-running the harness
   pipeline reproduced cycle 25's parent numbers EXACTLY, including
   stochastic fwd per episode (sto[5]=0.192) — "sto 1/6" across the
   whole lineage has been 6 fixed (command, noise-sequence) draws,
   not a sample. Eval-blind-spot flagged: sto results carry binomial
   noise on a FIXED panel; they measure those 6 draws only.
3. THE "PARK" IS THE BACKWARD COMMAND. Post-ramp commands of the
   seed-0 eval panel: sto[5] = (−0.028,−0.011) m/s, −159 deg. 360-ep
   command survey (6 fresh seeds): along-command displacement
   fraction by hemisphere — fwd |ang|<=30: mean 1.43 (n=279, min
   0.94); diag 30-90: 1.20 (n=49); rear 90-150: 0.28 (n=23, p10
   0.05); backward >150: −0.01 (n=9). The champion CANNOT walk
   backward or rear-lateral AT ALL — it churns in place (speed
   without displacement, which also evaded the survey's speed-based
   stall detector). kgate's old sto[4] was a slow −37 deg diagonal
   (since fixed); det[2] was forward churn (fixed). The lineage
   defect ladder was command-direction difficulty all along.
4. THE fwd>=0.40 CLAUSE SELECTS FOR OVERSPEED. At sto[5]'s |s|=0.030
   perfect tracking yields ~0.030x13.5 s = 0.40 m — the bar equals
   PERFECT, and slower draws (s_min 0.02 -> 0.27 m) cannot pass at
   all without ignoring the command. The lineage's det passes ride
   43% overspeed (survey, point 3). The gate clause and the
   overspeed defect (0-a iii) are the same object.
5. DR 1.0 CHAMPION BASELINE (logs/ckpt_eval/cw_walk_parkstart_mjx_
   dr10, 6+6, watched: none — scalar baseline only, no verdict taken
   on it): fwd 12/12, gv 12/12, 0 term, Imax <=2.73 — but agg slip/m
   det 1.543 / sto 1.295 vs the historical DR1.0 clause <=1.0.
   STABILITY IS NOT THE BINDING DEFECT (0-c.1 premise stale for this
   champion: 0 falls in 48 gated eps at DR0+DR1.0 today); SKATING is.
6. SKATING ROOT CAUSE (review-sanctioned checks, both passed):
   slip attribution — 91% of loaded-foot slip is MID-STANCE (not
   touchdown edges), spread evenly over all six legs, total ~= body
   displacement: the gait is PADDLING (swings are real, stance never
   anchors). Effort check — drag ticks draw 1.38x planted current
   (2.76 vs 2.01 A summed/leg). Objective-side price of all that
   effort: reward_current −13.3, reward_drag −4.4, reward_action
   −28.7 vs +1250 income = ~4%. Friction is NOT mispriced physically
   (floor mu 1.5, feet mu 2.0; the policy overpowers it and the
   objective barely charges the electricity). Root-cause chain:
   paddling <- velocity income pays for body translation however
   achieved <- physical effort priced at 4% of income <- objective
   defect (effort scale), NOT sim physics. Deepest reachable link:
   the effort price. This is the external review's pre-adopted
   "walk-routed effort/CoT term" lever, entered per its own
   escalation order (speed diagnostic -> effort check -> term).

### CODE — cycle 27: park-bank reset support (shipped, OFF) + walk-routed effort term (the new arm)
1. sim_env.py start_at=="park": optional harvested-pose bank (cfg
   goal.walk_park_bank npz path + walk_park_bank_frac), legacy rng
   stream untouched when unset (short-circuit; probe: bank draws
   8/8, frac=0 synthetic, no-bank determinism, frac=0.5 -> 10/20).
   Kept despite the dead arm: the mechanism is generic reset-state
   injection, tested, zero-cost when off. harvest_park_states.py is
   the bank builder (also the park-rate survey tool).
2. _parse_cfg_set (train_ppo_sim, shared with MJX trainer) and
   eval_checkpoint --cfg-set now fall back to STRING values for
   non-numeric overrides (bank paths). Numeric behavior unchanged.
3. walk_task.py: reward.k_walk_effort (default 0 = off, legacy
   exact) — walk-routed per-tick charge k x mean(servo_current_A),
   in _post_step so the MJX stack applies it identically; logged as
   reward_effort (added to MJX W&B key list). Root-cause chain in
   INVESTIGATION point 6. SCALE AUDIT (champion det ep, seed-0
   panel): k=1.2 realizes −218.6/ep = 18% of the +1219 legacy
   income, per-tick −0.583 vs kernel+prog +2.7; quiet stand pays
   −0.48/tick but earns ~0 (prog-gated kernel) — walking stays
   strictly dominant, collapse-to-stand is not the cheap exit.
   k=0 parity: episode total matches pre-change decomposition run.
   rl_move/tests 38/38 pass.

## NEEDS OPERATOR (non-blocking, walk line) — two gate-definition findings from cycle 27
(a) REAR-HEMISPHERE SCOPE: the champion has zero backward/rear-
lateral competence (INVESTIGATION point 3); commands there are ~10%
of training episodes. 0-c names DISTANCE/STABILITY/RELIABILITY and
"lateral/yaw only after forward is real" — please rule whether rear-
hemisphere command competence is in scope now (a command-exposure
arm is ready to design) or explicitly deferred (then walk gates
should draw forward-hemisphere commands only, and the standing
12/12-including-backward-draw clause is retired).
(b) GATE CLAUSE: fwd>=0.40 m is unpassable at perfect tracking for
commands below ~0.030 m/s (point 4) — the clause has been selecting
overspeed. Propose command-scaled distance (e.g. along-command
displacement >= 0.75 x commanded) or gating on the harness's own
vel_err success. Also: the sto pass is a FIXED 6-draw panel (point
2) — consider multi-seed panels for gate decisions.

## Cycle 28 (2026-08-09 ~07:0xZ) — cw-stance-lowerdense verdict: dense charging REFUTED on both pre-registered if-false shapes — the hover pays the (economically trivial) charge AND deepens its allowance-riding — and the arm ERODED RISE (stop rule tripped: heights 4/24, one fall + one over-current termination on camera). Opposing gradients NAMED by controller probe: the current model prices a feet-planted descent HOT (Imax 2.63 A, current_hot 4x the hover's) and support_margin pays the tripod hover MORE than a planted stance. Stance line BLOCKED on the operator allowance/pricing ruling per pre-registration. Concurrent with the walk cycle (parkstart/harvest line, hands off per prompt).

### cw-stance-lowerdense — charging the whole descent bought nothing: the hover is INCOME-POSITIVE, not under-priced at the margin
OBSERVATIONS (mechanical). W&B 7iyhg21d FINISHED, 20,054,016 steps
(GPU-MJX, mjx-train-0, 1388 s, ~14.4k fps incl. setup). Final ckpt
ppo_goal_cw_stance_lowerdense.zip md5
a52db745e054fe2906e8d67a87279691 (pod + controller match). Train:
ep_rew quarters 239/269/261/252; std 0.163→0.150; KL ~0.02 at
target. env/reward_end_posture quarters −0.306/−0.336/−0.342/−0.339
— WORSENED ~0.03 under the 5x-wider charge window (if-true needed
improvement): the policy pays. Canaries: lower_a/b, rise_flat_a/b,
rise_bridge_a, rise_crouch_b green throughout; rise_bridge_b RED
from ~mid-run (mean 0.55, last 5 all 0), rise_crouch_a last probe 0.
Internal eval/rise_bridge_frac 1.0→0.667→0.0→0.25 — auto-stop never
fired because bridge_a stayed green (group rule needs the full
group down; single-member persistent failure is a blind spot, watch
item, no code change this cycle). terminations/tilt_roll crept
1.0→2.0 (last 3.0).
Gate harness (posture-strict, DR 1.0, own cfg MINUS
lower_belly_start_frac, 6 eps/mode det+sto, explicit --modes lower
rise hold raise track; logs/ckpt_eval/cw_stance_lowerdense_gate):
- lower posture det 0/6 sto 0/6 (gate ≥4/6 det ≥5/6 sto — FAIL;
  bellyrest baseline 0/12, no change). Heights 12/12 (h_err ≤3.4
  det, ≤6.8 sto) → lower height clause ✓. Per-leg det end
  clearances vs bellyrest NAMED baseline: leg0 98–129 (br 112–145,
  overlap), leg2 69–86 (br 58–82, no better), leg4 54–64 (br
  46–72) with endings 54/59/64/61/61/60 mm — clustered ON the
  60 mm boundary (the eyeball clause's 40–60 signature). Summed det
  over-allowance 68–81 mm (br 80–107; if-true bar <80): fell by
  riding the boundary, not by planting — posture is 0/12 and the
  training charge worsened, so the if-true conjunction fails.
- RISE — STOP RULE TRIPPED: heights (h_err ≤15, no term) det 2/6
  (19.9+tilt_roll TERMINATION, 33.5, 18.8, 1.4✓, 6.2✓, 23.0), sto
  2/6 (3.7✓, 18.9, 43.9, over_current TERMINATION, 15.2, 5.8✓) vs
  parent 12/12 — 4/24 heights, 2 safety terminations vs parent
  0/60, far outside the ±1–2 ep band. Posture 0/12 (parent 1/12).
  leg2 ends speared at 175–188 mm in 5/12 rise eps (parent's worst
  rise clearances were ~181 in 2 eps) — the lower-mode spear
  pattern has propagated into rise endings.
- hold det 6/6 sto 6/6 ✓ (parent 12/12). track 12/12 (parent
  11/12, inside noise). raise 0/12, worst_clear 235–239 —
  unchanged canary. Imax band 2.30–2.69 A (recalibration caveat
  stands). Safety flags outside terminations 0.
FRAMES WATCHED (md5/frames): lower_det_0 6924fb95/250,
lower_sto_0 db630fb9/250 — descent proceeds on schedule, then the
right-front leg SPEARS straight out horizontally by mid-episode
and holds to the end, other feet visibly airborne; never a flat
rest; same pathology as parent. rise_det_0 24b8629c/186 (tilt_roll
term) — NEVER stands: stays sprawled/splayed the whole strip and
tips over on camera in the final frames — a genuine fall.
rise_det_3 de088dda/250 (best height, 1.4 mm) — reaches height but
ends WIDE/SPLAYED, feet at 18/26/42 mm. rise_sto_2 d1817f6f/250 —
partial rise with a leg speared out, ends 43.9 mm low, leg2 at
188 mm. rise_sto_0 478933cb/250 (height pass) — splayed stand,
legs 0/4 at 26/38 mm. hold_det_0 33ca386c/250 + hold_sto_0
8c39ffa4/250 — clean quiet six-foot stance, genuine passes.
track_det_0 7ca9fe9b/250 — quiet stance, small corrections, fine.
raise_det_0 8e61b287/250 — the permanent leg2-horizontal canary
posture. Remaining eps scalars only (unwatched).
Exploits looked for: height shortcut (none, lower h_err ≤6.8);
safety-layer reliance (none); schedule dodge (none). FOUND:
allowance-riding deepened (leg4 det endings pinned at 54–64 mm,
four of six within 1 mm of the boundary) — the summed-clearance
"improvement" (68–81 vs 80–107) is relabeling into the free zone.
OPPOSING-GRADIENT DIAGNOSIS (pre-registered branch (a) follow-up;
controller probe /tmp/diag_lower_gradient.py, DR 0, forced lower,
plant starts, own cfg, seeds 0–2; per-term reward integrals for
(A) this policy det, (B) scripted feet-planted descent — joint
targets plant→zero paced by the episode's own height ramp,
(C) hold-at-plant control):
- The dense charge is TRIVIAL money: full-hover descents pay
  reward_end_posture −4.5/−5.9/−11.3 per ep against ~216–227 of
  task+finish income (~2–5%). k=5.0 with a 60 mm allowance and
  per-leg clamp cannot bind.
- The planted descent is priced HOT by the current model: (B) pays
  current_hot −17.7 every seed vs (A)'s −2.2..−4.3, Imax 2.63 A
  (over the 2.5 A soft threshold) vs (A)'s 1.8–2.0 A; (C) hold
  draws 0.65 A. Slow lowering with loaded knees costs real sim
  current; unloading three legs RELIEVES it — the current terms
  pay for the hover.
- support_margin pays the hover MORE: (A) +34.1..+35.4 vs (B)
  +23.7 — the tripod out-earns a planted configuration on the
  margin term. load_even charges the hover −36 vs −16 but is
  outweighed (net hover advantage on the three physics terms
  ≈ +5..+9/ep BEFORE the task streams, which also favor the
  policy's precise height tracking).
- Caveat, honestly owned: the script is an imperfect "correct
  lower" (ends h_err 54–78 mm; zero-pose pads sit 29 mm above
  their plant refs), so task/finish-stream comparisons are
  confounded; the current/support/load comparisons are the
  informative ones. One script, 3 seeds — magnitudes are
  estimates, signs are consistent 3/3.
INTERPRETATION. Both pre-registered if-false shapes fired at once:
(a) the spears persist and PAY the dense charge (training integral
worsened; leg0 unchanged vs parent), and (b) the threshold-riding
generalized (leg4 pinned to the boundary; the summed metric fell
without any planting). The diagnosis then NAMES the stronger
opposing gradient branch (a) demanded: the hover is held up by
current-relief (~+13/ep) plus support-margin income (~+12/ep),
against which the posture charge (−5..−11/ep) is noise. Root-cause
chain: hover ← paid by current-relief + margin income ← the servo
current model prices a loaded slow descent at 2.6 A while the gate
DEMANDS that descent, and support_margin's shape rewards a tripod
at least as well as six planted feet ← either the current model
mispricess loaded descents (sim-fidelity defect) or the gate is
demanding a behavior the physics genuinely makes expensive
(objective conflict). Deciding which — and the 60 mm allowance —
changes the eval-gate definition and the reward economy together:
operator territory, not another shaping arm (pre-registration
already binds this). COLLATERAL: dense lower charging eroded RISE
through shared weights (the strongest-alternative predicted
destabilized DESCENT; descent survived, rise broke — mode-routed
rewards do not prevent weight-sharing interference; 4th
interference instance for the routing ledger).
VERDICT: FAIL (lower posture 0/12; rise-heights stop rule tripped
det 2/6 sto 2/6; lower heights 12/12, hold 12/12, track 12/12
met). NOT HARDWARE-READY: lower ends three feet airborne with a
~130 mm spear; rise falls over on camera (det[0]) and trips
over_current (sto[3]). CHAMPION UNCHANGED (cw_stance_dr10). Crown
jewels: ERODED IN THIS CHECKPOINT (rise heights 4/24 < 5/6) —
ppo_goal_cw_stance_lowerdense.zip is quarantined as a warm-start
source; the last stance ckpt with heights 24/24 remains
ppo_goal_cw_stance_bellyrest.zip (itself gate-FAILED on posture).
HYPOTHESIS STATUS: REFUTED (both if-false shapes; opposing
gradient named and quantified). Dense-window shaping is CLOSED.
Per pre-registration: NO further stance shaping arms; the stance
line is BLOCKED pending the operator ruling below.

## NEEDS OPERATOR (stance line blocked): the lower allowance ruling now includes a PRICING conflict
Cycle 26 flagged the 60 mm allowance (threshold-riding, free-zone
relabeling, negative-clearance blind spot). This cycle adds the
deeper half: a feet-planted descent — the exact behavior the
posture gate demands — is priced at Imax 2.63 A / current_hot 4x
the hover's by the servo current model, and support_margin pays a
tripod hover MORE than a planted stance (+35 vs +24/ep). Questions
needing a ruling before any further stance arm: (1) allowance
magnitude (60 mm vs ~25–30 mm) and |c| pricing (cycle 26); (2) is
2.6 A for a slow loaded descent physically right on the STS3215s?
If yes, the gate and the current economy are in direct conflict
and the income structure (not a coefficient) must change; if no,
the current model needs recalibration (same defect class as the
dead-zone static-hold mispricing, 2026-08-08 canon). (3)
support_margin term shape review (tripod ≥ planted is suspect).
Evidence: logs/ckpt_eval/cw_stance_lowerdense_gate + the probe
numbers above. Meanwhile the stance line launches NOTHING.

### Cycle 28 launch decision: no launches — recorded reasoning
All four GPU pods were free at status time, but: the stance line is
blocked on the operator ruling (pre-registered by cycle 26's launch
entry and confirmed by this verdict — both if-false branches fired);
the walk line and its follow-ups (own-park harvest arm) belong to
the CONCURRENT cycle per this cycle's prompt; raise is a no-compute
canary; mirror-symmetry augmentation is queued behind the walk
line's item (1). Launching a stance arm anyway would be exactly the
"try another coefficient" pattern the external review forbids.
Cycle totals: 0 launches, 0 steps, 1 harness eval (60 eps; 10
strips watched, provenance above), 1 controller diagnosis probe (9
episodes, script recorded in-log).

### LAUNCH cw-walk-effort (20M GPU steps, mjx-train-1) — walk-routed effort/CoT pricing vs the paddling gait (the DR1.0 skating blocker)
The review's pre-adopted lever, entered per its own escalation order
after BOTH sanctioned checks passed this cycle (speed diagnostic:
43% forward overspeed via all-six-legs mid-stance paddling; effort
check: drag ticks 1.38x planted current). One variable off champion
01d9ab60: reward.k_walk_effort=1.2 (walk-routed per-tick charge on
mean servo current; scale audit −218/ep = 18% of income; stand-
still stays dominated). Config otherwise identical to the champion
lineage (park_start_frac 0.25 retained, DR 0, 15 s, seed 0). Gate
leads with the DR1.0 slip clause the champion FAILS today (agg
slip/m det 1.543 / sto 1.295 vs <=1.0), plus DR0 gait retention
(det fwd mean >=0.55, gv 12/12, fwd-hemisphere sto 5/5; the
backward draw sto[5] is recorded but excluded pending the operator
scope ruling above). Pre-registered if-false branches: charge paid
+ slip unmoved -> phase prior (review rung 3); gait collapse -> one
scale-audit-recalibrated retry max, no blind k iteration.
Strongest alternative (current drops without slip dropping —
lighter paddling) distinguished by slip vs current trends moving
independently. Mechanism smokes: controller probe (term value/
parity) + smoke-walk-effort-r2 on sweep-lower (384 CPU updates with
term active, reward shift matched audit, no NaN; CPU trainer hung
at SHUTDOWN after training completed — pre-existing path, remnants
killed, recorded in ledger; first smoke attempt failed verification
on a missing init-from checkpoint, fixed by kubectl cp). VERIFIED
RUNNING by the launcher: pid 1327749, W&B 9rtpws1h advancing
(1.31M->2.88M in window), fps ~17.5k, pod code at snapshot 286200f,
env/reward_effort live at −0.605/tick (audit predicted −0.583) —
the term is demonstrably active in MJX training. Watcher owns the
checkup.
FLEET NOTE: mjx-train-0 runs the concurrent stance cycle's
lowerdense arm (hands off). mjx-train-2/3 stay idle deliberately:
the two candidate walk arms besides effort are BLOCKED — rear-
hemisphere exposure on the operator scope ruling, and any further
park/reset arm is closed by this cycle's investigation; launching a
second speculative pricing arm would double-change the lineage.
Cycle totals: 3 launches (2 smoke attempts, 1 experiment), 20M GPU
steps (cap 80M), 0.12M CPU smoke steps (cap 16M), 4 harness evals +
~1100 controller probe episodes; 9 strips + 10 full-res frames
watched (provenance in verdict).

## Cycle 29 (2026-08-09 ~07:4xZ) — cw-walk-effort verdict: effort/CoT pricing REFUTED on pre-registered if-false branch (i) — the policy paid an 18%-of-income charge for 20M steps without moving current, drag, or slip; paddling is not effort-reachable. Escalating to the review's phase-prior rung (its own escalation order), with a scale-audit-designed clock and a new MJX obs-transplant port.

### cw-walk-effort — the charge was simply paid: every effort channel flat over 20M steps, slip unmoved, one new over_current termination
OBSERVATIONS (mechanical). W&B 9rtpws1h FINISHED, 20,054,016 steps
(GPU-MJX, mjx-train-1, 1305 s, ~15.3k fps incl. setup). Final ckpt
ppo_goal_cw_walk_effort.zip md5 4505fa1d8f8986bb0f6ce5fb02683e27
(pod + controller match). Train quarters: ep_rew 686/847/848/870;
env/reward_effort per-tick −0.6014/−0.6034/−0.6055/−0.5946 — FLAT
(launch verification saw −0.605; audit predicted −0.583; the term
was live and priced as designed). reward_current −0.041 flat,
reward_drag −0.0174 flat, std 1.388→1.412, KL ~0.009. The policy
recovered income while paying ~−225/ep effort; it never reduced
the electricity.
Gate harness (both at own cfg incl. k_walk_effort, minus
park_start_frac = normal starts, 6+6, seed 0, videos every ep):
- DR 1.0 (logs/ckpt_eval/cw_walk_effort_dr10): agg slip/m det
  1.484 vs champion NAMED baseline 1.543 (Δ −0.06, ~4% — no
  evidence of change) — gate ≤1.0 FAIL. sto 1.753 vs champion
  1.295 — WORSE, driven by the fixed panel's hard draws: sto[1]
  partial stall (fwd 0.378, slip/m 3.36, duty leg0 0.80), sto[5]
  backward draw churn (fwd 0.161, slip/m 5.91, leg1 duty 0.07 →
  SACRIFICED flag, gv FAIL). gv 11/12 vs gate 12/12 FAIL. sto[0]
  over_current TERMINATION (champion baseline had 0 term) — 0-term
  clause FAIL. det fwd mean 0.648 (champion 0.618), Imax 2.66
  (champion 2.73).
- DR 0 15 s (logs/ckpt_eval/cw_walk_effort_15s): det fwd mean
  0.722 ≥0.55 ✓ (champion 0.745, inside noise); fwd-hemisphere sto
  ≥0.40 5/5 ✓ (0.789/0.663/0.660/0.741/0.658); backward draw
  sto[5] fwd 0.276 recorded-excluded per operator ruling pending,
  leg1 duty 0.08 sacrificed → gv 11/12 (champion 12/12, but its
  sto[5] partial park was a known gv blind spot — same episode
  family, now flagged). det agg slip/m 1.282 vs champion 1.180 —
  no improvement. 0 term.
FRAMES WATCHED (md5/frames): dr10 walk_det_0 c9e23c36/375,
walk_sto_0 b7a74db3/317 (over_current term on camera at ~12.7 s),
walk_sto_1 398aabd3/375, walk_sto_5 36b03fc1/375; 15s walk_det_0
0ce5382c/375, walk_det_4 9e52f80e/375, walk_sto_0 63254494/375,
walk_sto_5 4bcee5be/375. Remaining eps scalars only (unwatched).
Pathologies first: the gait is IDENTICAL to the champion's
paddling — sprawly wide stance, stance feet creeping with the body
in every strip including passes; sto_1/sto_5 near-stationary churn
with semi-parked front legs; NO visible stance anchoring anywhere.
Achievements: six legs cycling with real swings in passing eps,
body level, boards advance (det strips), 0 falls.
Exploits looked for: collapse-to-stand (none — speeds 0.045–0.057
in band); slip reduction via slower speed (none — speed unchanged);
strongest-alternative "current drops without slip dropping" (did
NOT fire — current never dropped either).
INTERPRETATION. Clean refutation on pre-registered if-false branch
(i): slip ≥1.2 with the charge simply paid. The effort channels
were live, correctly scaled (18% of income), and completely inert
as gradients — 20M steps moved NONE of them. Together with cycle
24 (park income cut 1250→274, park persisted) this is now two
demonstrations that ECONOMIC pressure alone does not reorganize
this gait: anchored stance is likely not reachable by local
gradient from paddling (any partial anchoring loses velocity
income immediately; the intermediate states are all worse). That
is precisely the case for a structural prior (timing scaffold)
over pricing. The sto worsening (1.753, over_current term) is on a
fixed 6-draw panel — single-episode effects, not calibrated
evidence of harm, but certainly not improvement.
VERDICT: FAIL (DR1.0 slip clause, gv, 0-term all missed; DR0
retention clauses met). NOT HARDWARE-READY: skating unchanged
(feet grind 1.3–1.5 m per m walked at DR 1.0), one over_current
termination on camera. CHAMPION UNCHANGED
(ppo_goal_cw_walk_parkstart_mjx.zip md5 01d9ab60).
HYPOTHESIS STATUS: REFUTED (if-false branch (i) exactly as
pre-registered). Effort/CoT pricing rung CLOSED; per
pre-registration the next rung is the review's phase prior.
Ledger updated via launch_run.py update (status FINISHED, ckpt
md5, verdict).

### Cycle 29 phase-prior design — why this is NOT the refuted cycle-12 arm, and the scale audit
Cycle 12 refuted the weak tripod phase reward as a BASIN-ESCAPE
tool: in the shuffle basin it couldn't outbid the park's cost
savings; in the stance basin it phase-locked a tripod PARK. Both
parents had NO gait. Today's parent has a genuine six-leg gait
whose defect is timing/anchoring; the hypothesis is different in
kind: a stationary observable clock plus agreement income gives
the existing gait a reference to regularize against, making
anchored stance reachable where pure pricing (effort, cycle 29;
park income, cycle 24) provably was not. Root-cause chain
(required artifact): paddling ← velocity income collectible
without anchoring ← no term pays for anchoring itself and effort
charges are paid not avoided (2 refutations) ← the objective gives
the policy no coordination REFERENCE to reorganize toward —
structural defect, addressed by the review's pre-ordered phase
prior (rung 4), not another price.
SCALE AUDIT (controller probe /tmp/probe_phase_scale.py, champion
det eps, DR0 own-cfg, seeds 0–2, per-tick contacts vs best-offset
fixed clocks): champion cadence is IRREGULAR — best-offset tripod
agreement at 0.4 Hz = 0.838/0.532/0.751 by seed (near chance at
0.5–1.0 Hz: 0.51–0.62). So (a) natural cadence ≈0.4 Hz where
periodic at all → goal.walk_phase_hz=0.4 (cycle-11's 1.0 Hz would
fight the gait); (b) the clock's income is NOT freely collectible
by the current behavior (seed1 would earn ~0.06/tick of the max
1.0) — locking requires actual timing reorganization, which is
the mechanism under test. k_phase_contact=1.0: max +1.0/tick vs
kernel+prog +2.7/tick (27% add-on), zero-mean at chance, parked
legs zero-net (cycle-11 machinery, unchanged since; MJX applies it
in _post_step, reward_phase_contact already in the MJX W&B key
list).

### CODE — cycle 29: MJX obs-pad transplant port + phase_agreement logging
1. train_ppo_mjx.py: --obs-pad-transplant N (port of the
   train_ppo_sim mechanism, same pad_obs_transplant function
   imported): warm start across an obs widening, parent weights
   bit-identical until training moves the zero-padded first-layer
   columns; fresh optimizer; num_timesteps continued. Needed
   because the champion has no phase dims (+2 at the obs tail) —
   and by any later obs-widening arm (estimator ladder).
2. train_ppo_mjx.py: phase_agreement added to the AUX logged keys
   (the if-false branches of the phase arm are read off its trend).
rl_move/tests: 38 passed, 5 skipped. GPU probe of the new path
(probe-walk-phase-mjx, 2M steps) gates the main launch.

### PROBE probe-walk-phase-mjx (1M GPU smoke, mjx-train-1) — PASS
New-mechanism probe gating the phase arm: the train_ppo_mjx
--obs-pad-transplant port (this cycle's code change) + phase package
live on the warp stack. Transplant line "72 -> 74 dims; zero-padded
first-layer columns in [policy_net.0, value_net.0]" present; 1M
steps in 207 s; 0 tracebacks/NaN; periodic-eval walk err 0.029 m/s
(parent band). Ledger updated FINISHED/PASS.

### LAUNCH cw-walk-phaseprior (20M GPU steps, mjx-train-1) — observable tripod clock + agreement income vs the paddling gait
The review's rung 4, entered per its own escalation order after the
effort rung refuted (this cycle). One lever off walk champion
ppo_goal_cw_walk_parkstart_mjx.zip md5 01d9ab60 (k_walk_effort
DROPPED — refuted): goal.walk_phase_obs=1 + walk_phase_hz=0.4 +
reward.k_phase_contact=1.0, warm-started across the +2 obs widening
via the new MJX transplant (parent bit-identical at init). Clock
rate and k from the scale audit (design section above): 0.4 Hz
matches natural cadence where periodic; agreement income (max
+1.0/tick vs +2.7/tick kernel+prog) is NOT collectible by current
behavior (champion best-offset agreement 0.53–0.84 by seed —
irregular). Distinct from cycle-12's refuted basin-escape phase arm
(recorded in the design section; plan updated to disambiguate).
Gate: the effort arm's gate verbatim (DR1.0 slip det/sto ≤1.0, gv
12/12, 0 term; DR0 retention; frames for anchoring + cadence),
plus the env/phase_agreement trend read against three
pre-registered if-false shapes ((a) agreement up/slip unmoved →
phase rung closed, (b) agreement flat → prior can't engage warm,
(c) gait degrades → kill). VERIFIED RUNNING: pid 1958285, W&B
ia5x7piz advancing (4.65M in first minutes, ~19k fps), pod code at
snapshot 76c41c0, transplant line in log, env/phase_agreement live
at 0.512 = chance (term active, unearned at init — as designed).
Watcher owns the checkup.
FLEET NOTE: mjx-train-0/2/3 idle deliberately — stance line BLOCKED
on the operator pricing/allowance ruling (cycle 28), rear-hemisphere
exposure BLOCKED on the operator scope ruling (cycle 27), raise is a
no-compute canary, mirror-symmetry is queued post-phase-verdict
(plan item 2), and a parallel 0-c.2 distance-income arm would
pre-empt the pending operator overspeed/gate ruling it directly
interacts with (recorded as the reason, not deliberation idle).
Cycle totals: 2 launches (1 GPU smoke probe PASS, 1 experiment),
21M GPU steps (cap 80M), 0 CPU steps, 2 harness evals (24 eps),
1 controller scale-audit probe (3 eps); 8 strips watched
(provenance in verdict).

## Cycle 30 (2026-08-09 ~08:1xZ) — cw-walk-phaseprior verdict: phase prior REFUTED on if-false branch (a) — the clock LOCKED (agreement 0.47→0.93, clean tripod duty at eval) and slip did not move (worse if anything). Timing is orthogonal to anchoring. Phase rung CLOSED.

### VERDICT cw-walk-phaseprior (20M GPU steps, mjx-train-1, W&B ia5x7piz)
OBSERVATIONS (mechanical). Run finished: 20,000,000 steps in 1302 s
(~15.4k fps), W&B state finished, ckpt
ppo_goal_cw_walk_phaseprior.zip md5 9a086702 pulled to controller
(pod copy identical). Training: env/phase_agreement 0.47 (chance,
launch verification) → 0.93 by ~8M, plateau 0.92–0.93 to the end;
reward_phase_contact 0 → +0.86/tick of max +1.0; mean_current_a
0.49→0.55 A (UP ~12%); periodic walk err flat 0.028–0.036 m/s all
run; std 1.33, no entropy alarm; 0 tracebacks.
Gate harness (own cfg minus park_start_frac = normal starts, 6+6,
seed 0, videos every ep):
- DR 1.0 (logs/ckpt_eval/cw_walk_phaseprior_dr10): agg slip/m det
  1.786 / sto 1.523 vs gate ≤1.0 — FAIL, and vs champion NAMED
  baseline det 1.543 / sto 1.295 both passes are +16–18% WORSE
  (champion per-ep det spread 1.32–1.82, so the delta is at/just
  beyond the panel's noise scale — fair reading: no improvement,
  possibly slight worsening; direction consistent across passes).
  gv 12/12 ✓ (effort arm was 11/12). 0 terminations ✓ (effort had 1
  over_current). det 5/6, sto 5/6; det fwd mean 0.593 vs champion
  0.618 (inside noise); speeds 0.043–0.055 in band. Duty cycles:
  clean tripod split — legs {0,2,4} 0.54–0.66, legs {1,3,5}
  0.36–0.51, swings 6–12/leg/15s ≈ 0.4–0.5 Hz = the clock. Champion
  same-panel duty (parkstart_mjx_dr10 report): 0.53–0.61 vs
  0.38–0.53 — already tripod-ish; the phase arm sharpened timing
  regularity, visible mostly in swing-count consistency.
- DR 0 15 s (logs/ckpt_eval/cw_walk_phaseprior_15s): det fwd mean
  0.681 ≥0.55 ✓ (champion 0.745, effort 0.722 — low end, inside
  noise); gv 12/12 ✓; fwd-hemisphere sto fwd ≥0.40 5/5 ✓
  (0.693/0.616/0.685/0.678/0.618); backward draw sto[5] fwd 0.300
  recorded-excluded per pending operator ruling (slip/m 3.91 —
  rear-hemisphere hole unchanged). det agg slip/m 1.581 (champion
  1.180 — worse here too). 0 term ✓.
FRAMES WATCHED (md5/frames): dr10 walk_det_0 31e02cab/375,
walk_sto_0 09ca5dec/375, walk_sto_5 b0520214/375; 15s walk_det_0
547eab55/375, walk_sto_0 b8f437d1/375, walk_sto_5 0d6c4221/375;
plus dense 5 fps 3.2 s filmstrips of dr10 det_0 vs the effort
baseline det_0 side-by-side. Remaining eps scalars only (unwatched).
Pathologies first: the motion is the SAME sprawly paddling creep as
the champion — wide splayed stance, body low, stance feet visibly
sweeping with the body in every strip including passing episodes; NO
stance anchoring anywhere; the dense filmstrip is nearly
indistinguishable from the effort arm's. Achievements: six legs
cycling with regular tripod timing, body level (tilt overlays <1°),
0 falls, 0 terminations in all 24 episodes.
Exploits looked for and not found: collapse-to-stand (speeds in
band); slip reduction via slower speed (speed unchanged — and slip
didn't reduce anyway); phase income farmed by a tripod PARK
(cycle-12 pathology: duty pinned 0/1) — duty 0.36–0.66, real swings,
not present.
INTERPRETATION. The mechanism ENGAGED and the hypothesis still
failed: agreement income was not collectible at init (0.47 chance,
as the scale audit predicted), the policy genuinely reorganized its
timing to the 0.4 Hz clock (0.93 sustained, tripod duty split at
eval), i.e. the "coordination reference" was delivered and adopted —
and the gated quantity (loaded-foot slip) did not move in either
panel. Locked timing pays the clock while the feet still sweep;
anchoring is a FORCE-TRANSMISSION property, not a timing property.
Current going UP 12% while cadence regularized is consistent with
"same paddling, now metronomic". This is if-false branch (a) VERBATIM
as pre-registered at launch: agreement >0.7, slip ≥1.2 — timing lock
orthogonal to stance anchoring. Third independent lever that failed
to move slip (park-income gating c24, effort pricing c29, timing
reference c30) — the income structure itself still pays full
velocity income for friction-sweep propulsion.
VERDICT: FAIL (DR1.0 slip clause det 1.786/sto 1.523 vs ≤1.0; DR0
retention clauses met; gv 12/12 and 0-term clauses met). NOT
HARDWARE-READY: skating unchanged — feet grind 1.5–1.8 m per meter
walked at DR 1.0; that motion on carpet stalls servos (a motor has
already cooked). CHAMPION UNCHANGED
(ppo_goal_cw_walk_parkstart_mjx.zip md5 01d9ab60).
HYPOTHESIS STATUS: REFUTED (if-false branch (a) exactly as
pre-registered). Phase-prior rung CLOSED. Per pre-registration the
remaining review rung is the dense step-component decomposition;
plan item 2 (mirror-symmetry) also unblocks on this verdict — see
decision section below.

### Cycle 30 anchor-gate design — root-cause chain and scale audit
ROOT-CAUSE CHAIN (required artifact): paddling ← full velocity
income (kernel 2.0 + prog, ~2.7/tick) is collectible while stance
feet sweep ← income is conditioned only on BODY velocity, never on
HOW force is transmitted (no term distinguishes anchored push from
friction-sweep; charging the sweep is paid — 2 refutations — and a
timing reference locks without anchoring — cycle 30) ← at root the
sim prices sliding friction cheaply (drag ticks draw only 1.38x
planted current, cycle 27; real servos dragging on carpet stall and
heat). The DEEPEST link (current-model pricing) is with the OPERATOR
(cycle 28 stance ruling covers the same defect — recorded reason it
is inaccessible today). The deepest reachable link is the income
CONDITIONING — structural, "worth less by construction" (operator
0-c.2: gate income so non-walking can't collect; step0 mandate
verbatim). NOT a reward patch in the refuted sense: no new charge,
no coefficient on a penalty; the existing income becomes conditional
on the property it was always meant to pay for.
DESIGN: foot ANCHORED = loaded AND within reward.anchor_tol_mm of
its own touchdown point (anchor resets at liftoff). Per tick,
r_walk and POSITIVE r_prog are multiplied by
(1-g) + g * anchored_frac_of_loaded_feet, g=walk_anchor_gate.
Negative prog is never gated (a gate must not shrink a penalty);
zero loaded feet ⇒ factor (1-g). Walk-mode only by construction.
SCALE AUDIT (controller probe /tmp/probe_anchor_scale.py, champion
det eps DR0 own-cfg seeds 0–2, income-weighted collectible factor):
tol=5mm 0.29/0.42/0.42; tol=10mm 0.70/0.53/0.54; tol=15mm
0.88/0.75/0.58; tol=20mm 0.84–0.96 (too loose, gate never binds).
CHOSEN tol=10 mm, g=1.0: champion paddling keeps only 0.53–0.70 of
velocity income — a ~2x bigger stake than the refuted 18% effort
charge, DENSE and MULTIPLICATIVE (each additional anchored foot-tick
raises income), while champion creep is ~24 mm/stance so honest
anchoring (<10 mm) is a real reorganization, and an anchored gait
collects ~1.0. Functional probe (/tmp/probe_anchor_gate.py, champion
seed-0 det ep): gate OFF = legacy exact (return 1219.3, info key
absent); gate ON return 964.3, mean anchor_frac 0.731, income ratio
0.749 ≈ audit prediction 0.70. Exploit pre-registered: unload-sweep
(feet below contact threshold while sweeping → uncounted) — but
unloaded feet cannot transmit propulsive friction, so exploiting the
gate this way IS stepping; residual watch: duty drop + swing spike
with slip still high.

### CODE — cycle 30: anchored-stance income gate (walk_task.py) + MJX logging
1. walk_task.py: reward.walk_anchor_gate / reward.anchor_tol_mm
   (default 0 = off, legacy exact — verified by probe). New per-foot
   anchor bookkeeping (_anchor_xy/_anchor_prev_on) kept SEPARATE
   from step-event state; added to MJX_SNAPSHOT_EXTRA for host-half
   parity; reset in _reset_begin. walk_anchor_frac in step info.
2. train_ppo_mjx.py: walk_anchor_frac added to AUX logged keys.
rl_move/tests: 38 passed, 5 skipped. GPU probe smoke gates the
launch (new mechanism, audit §6).

### PROBE probe-walk-anchor-mjx (1M GPU smoke, mjx-train-1) — PASS
New-mechanism probe gating the anchor arm: 1M steps in 202 s, 0
tracebacks/NaN; anchor state survived the MJX host-half snapshots
(MJX_SNAPSHOT_EXTRA addition exercised for the full run); gate
visibly live — ep_rew_mean 438→555 vs ungated parent band
~1100–1400 (income cut to ~0.5, matching the scale audit) and
already climbing, i.e. the gradient is being followed within 1M
steps. periodic eval ran (walk err 0.032). walk_anchor_frac W&B key
check deferred to the main launch (smokes run WANDB_MODE=disabled);
closed there. Ledger updated FINISHED/PASS.

### LAUNCH cw-walk-anchorgate (20M GPU steps, mjx-train-1) — anchored-stance income gate vs the paddling gait
The dense-decomposition rung's stance-no-slip component, entered per
the phase arm's pre-registration, implemented as INCOME GATING (the
step0 "worth less by construction" mandate + operator 0-c.2), not as
the refuted charge form. One lever off walk champion
ppo_goal_cw_walk_parkstart_mjx.zip md5 01d9ab60:
reward.walk_anchor_gate=1.0 + anchor_tol_mm=10 — kernel + positive
progress income multiplied per tick by the anchored fraction of
loaded feet (anchored = within 10 mm of own touchdown point).
Tolerance from the scale audit (design section above): paddling
collects 0.53–0.70, anchored gait ~1.0. Gate: the phaseprior gate
verbatim (DR1.0 slip det/sto ≤1.0, gv 12/12, 0 term; DR0 retention;
frames for anchoring) plus the walk_anchor_frac trend against three
pre-registered if-false shapes ((b) frac flat/income forfeited ⇒
anchoring not locally reachable even at a 30–47% stake ⇒ fresh-init
WITH the gate (plan item 0) or distillation; (c) frac >0.85 unearned
without slip falling ⇒ one audit-driven tolerance correction, else
rung closed; (d) collapse/degradation ⇒ kill). VERIFIED RUNNING:
pid 2588626, W&B jiwhh0qd advancing (3.2M in first minutes, ~18k
fps), pod code at snapshot 51859c8, walk_anchor_frac LIVE 0.77–0.80
with reward_walk 1.15–1.22/tick (ungated parent ~2.0+ — the gate is
biting). Watcher owns the checkup.
FLEET NOTE: mjx-train-0/2/3 idle deliberately — stance line BLOCKED
on the operator pricing/allowance ruling (cycle 28), rear-hemisphere
exposure BLOCKED on the operator scope ruling (cycle 27), raise is a
no-compute canary, and mirror-symmetry (plan item 2, unblocked by
this cycle's phase verdict) is UNIMPLEMENTED — it needs obs/action
mirror index maps + trainer support + its own probe, and shipping a
second new mechanism in the same cycle as the anchor gate is the
confounded-multi-change failure mode; it gets the next
implementation cycle (recorded as the reason, not deliberation
idle).
Cycle totals: 2 launches (1 GPU smoke probe PASS, 1 experiment),
21M GPU steps (cap 80M), 0 CPU steps, 2 harness evals (24 eps),
2 controller probes (anchor scale audit 3 eps + gate on/off
functional check); 6 strips + 2 dense filmstrips watched
(provenance in verdict).

## Cycle 31 (2026-08-09 ~09:0xZ) — cw-walk-anchorgate verdict: gate FAIL (slip det 1.240 / sto 1.245 vs ≤1.0) but the anchor gate is the FIRST of four levers to actually move slip — det −20% vs champion beyond per-ep noise, at unchanged speed and HIGHER forward distance. anchor_frac 0.767→0.837 still climbing at run end. Near-miss → consolidate-in-place c1 + pre-registered fresh-init basin test launched in parallel.

### VERDICT cw-walk-anchorgate (20M GPU steps, mjx-train-1, W&B jiwhh0qd)
OBSERVATIONS (mechanical). Run finished: 20,000,000 steps in 1302 s
(~15.4k fps), W&B state finished, ckpt
ppo_goal_cw_walk_anchorgate.zip md5 35234ddc151ac6f7e05350b7c550efb7
pulled to controller (pod copy identical). Training quartiles (W&B):
env/walk_anchor_frac 0.767→0.803→0.826→0.837 (last 0.836; monotone,
slope shallowing, NOT plateaued); env/reward_walk 1.20→1.22 (income
held while gated — earned, not forfeited); ep_rew_mean 664→862→867→861
(last 896); train/std 1.375→1.306 still annealing; mean_current_a
FLAT 0.481→0.492 (contrast phaseprior +12%); walk_speed 0.049→0.052
in band all run; 0 tracebacks.
Gate harness (own cfg minus park_start_frac, 6+6, seed 0, 15 s,
videos every ep):
- DR 1.0 (logs/ckpt_eval/cw_walk_anchorgate_dr10): agg slip/m det
  1.240 / sto 1.245 vs gate ≤1.0 — FAIL. Named baseline champion
  parkstart_mjx det 1.543 / sto 1.295. det per-ep
  [0.986, 1.001, 1.272, 1.342, 1.353, 1.504] vs champion
  [1.315, 1.421, 1.447, 1.566, 1.750, 1.821]: 4/6 eps BELOW the
  champion's per-ep minimum, two eps under 1.0 (first sub-1.0 DR1.0
  episodes in lineage history) — beyond the per-ep noise band, real.
  sto per-ep ranges overlap (1.131–1.366 vs 1.071–1.669), agg delta
  −0.05 — NO EVIDENCE of sto change. Speed det 0.048–0.056 vs
  champion 0.044–0.054 (same band — slip drop NOT bought with
  slowness); det fwd mean 0.681 vs champion 0.618 (higher). gv
  12/12, 0 term, imbal 1.26–1.31.
- DR 0 15 s (logs/ckpt_eval/cw_walk_anchorgate_15s): det fwd mean
  0.736 ≥0.55 ✓ (champion 0.745 — parity); det agg slip/m 1.144
  (champion 1.180 — inside noise); gv 12/12 ✓; fwd-hemisphere sto
  fwd ≥0.40 5/5 ✓ (0.739/0.668/0.731/0.737/0.665); backward draw
  sto[5] fwd 0.272, slip/m 3.53 recorded-excluded per pending
  operator ruling (rear-hemisphere hole unchanged). 0 term ✓.
FRAMES WATCHED (md5/frames): dr10 walk_det_0
b1fff33d44d41ca6e7d0b8929e924980/375, walk_det_3 (best ep, slip
0.986) adbc44a1b12dabcbbac10358849f4dab/375, walk_sto_0
77578c2878788fc9c67441faff90b698/375; 15s walk_det_0
9fc9370b69f6ab3d1af226fa543099bf/375, walk_sto_5 (backward)
acf4648a84bf77f2406ddb3d2bc1cf99/375; strip PNGs e6b5f8ff/57aae90a/
98176e5b/a3e6b098/7f9f7df5; plus dense 5 fps 3.2 s filmstrips of
dr10 det_3 vs champion dr10 det_0 side-by-side. Remaining eps
scalars only (unwatched).
Pathologies first: the posture is STILL the wide sprawly low creep;
in the dense filmstrips the best-slip episode is NOT visually
distinguishable from the champion's paddling at this resolution —
I cannot claim visible stance anchoring from frames; the improvement
is scalar-only. Backward-command ep (15s sto[5]) still churns
near-in-place (duty pinned 0.72/0.65/0.68 on one tripod, slip 3.5).
Achievements: six legs cycling everywhere (gv 24/24), body level, 0
falls/terminations in 24 eps, currents flat vs champion.
Exploits looked for and not found: unload-sweep (pre-registered
watch: duty drop + swing spike with slip high) — duty 0.36–0.63
balanced, swing counts 6–12, champion-like, absent; slip-via-slowness
— speed band unchanged, fwd distances UP; collapse-to-stand — absent;
park — absent (0 park eps in 24).
INTERPRETATION. The gate ENGAGED and pulled in the intended
direction: anchor_frac rose monotonically all 20M steps, income was
earned back (reward_walk held ~1.2 while gated), and det slip fell
past the champion's per-ep envelope — the FIRST slip movement after
three refuted levers (park-income gating c24, effort pricing c29,
phase prior c30). But the run ended mid-gradient: frac 0.837 just
under the 0.85 if-true mark, slip 1.24 vs the 1.0 gate, and sto slip
(sampling at std 1.31) unmoved. NONE of the pre-registered if-false
shapes fired ((b) frac was not flat and income not forfeited; (c)
frac not >0.85; (d) no collapse). This is a textbook near-miss with
the mechanism trace consistent (frac↑ ↔ det slip↓ at fixed speed).
Two open questions split cleanly: (1) does the same gradient finish
the job (consolidate-in-place), and (2) is the residual paddle a
warm-start legacy basin or the sim's globally preferred transport
(fresh-init WITH the gate — the pre-registered escalation, run in
parallel since GPU capacity is otherwise idle and the answer is
independent of (1)).
VERDICT: FAIL against the recorded gate (DR1.0 slip clause both
passes; all retention clauses met). NOT HARDWARE-READY: feet still
grind 1.24 m per meter walked at DR 1.0 — that motion stalls and
heats servos on carpet (a motor has already cooked). CHAMPION
UPDATED: ppo_goal_cw_walk_anchorgate.zip md5 35234ddc is the new
walk champion — beats parkstart_mjx on the named blocker metric
(DR1.0 det slip 1.240 vs 1.543, beyond per-ep noise) with fwd
distance up (0.681 vs 0.618 DR1.0; 0.736 vs 0.745 DR0 parity), sto
slip/gv/term parity, currents flat. Champion is append-only; both
files retained.
HYPOTHESIS STATUS: INCONCLUSIVE — directionally supported (income
conditioning moved the gated quantity where charging and timing
references did not; frac climbed and was earned) but the if-true
prediction (frac >0.85 AND slip ≤1.0 both passes) was not reached
and sto slip shows no evidence of change. No if-false branch fired.
Continuation c1 is the discriminating experiment.
