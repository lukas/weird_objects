# dynrep — Dynamics-representation pretraining

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
