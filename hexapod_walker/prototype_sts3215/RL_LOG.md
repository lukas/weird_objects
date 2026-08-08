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
