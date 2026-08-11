# REWARD.md — the reward function, explicitly

What every reward term pays or charges, where it lives in code, and
how to find the EXACT values a given run trained with. Operator
directive 08-10: the reward function of a run must be documented, not
reverse-engineered from launch commands.

## Where a run's actual reward config is recorded

Every run records its RESOLVED reward section (config.yaml + that
run's `--cfg-set` overrides) in three places, written at launch by
`train_ppo_sim.py` / `train_ppo_mjx.py`:

1. **W&B run notes** — a `=== REWARD FUNCTION ===` block listing every
   `reward.*` key=value; cfg-set overrides are tagged `[cfg-set]`.
2. **W&B run config** — `reward_cfg` (queryable dict, same content).
3. **Per-term training curves** — `env/reward_*` panels (one scalar per
   active term, mean per rollout), so you can see what each term
   actually PAID, not just what it was set to.

This file documents what the keys MEAN. Semantics are enforced by
`rl_move/tests/test_task_semantics.py` (MDP_PREFLIGHT): before a term
change trains, the bank must show honest behavior out-earning every
known cheat. See RESEARCH_RULES.md.

## Design principles (violations caused real incidents — keep them)

- **No unconditional alive bonus** (`reward.alive: 0`). Survival paid
  ~96% of achievable reward once; PPO froze and collected it.
- **One substantial income: the task kernel.** Everything else is
  small shaping, one-time bonuses, weak penalties, or a GATE.
- **Gates multiply INCOME, never penalties** ("worth less by
  construction", operator 0-c.2). Additive charges on exploits are a
  CLOSED move for slip and yaw drift — the policy prices them in and
  keeps the exploit. A gate makes the cheat earn ~0 instead.
- **Every gated/optional term defaults to 0/off = byte-identical
  legacy behavior.** A run's reward is base stack + whatever its
  `--cfg-set` enables.
- **Declared routing.** A term is GLOBAL (prices physics: tipping,
  heat) or MODE-ROUTED (prices task shape). Routing changes must be
  declared and are refutable — e.g. all-modes `k_flag_leg` was refuted
  (taxed rise's legitimate curl transients) and re-routed walk-only.

## 1) Base stack — every tick, every mode

`rl_move/env.py compute_reward()`, shared by hardware env and sim.
Total = kernel income + weak shaping + weak regularizers.

| cfg key (reward.) | default | what it does |
|---|---|---|
| `k_track` | 1.0 | THE income. Product of per-objective Gaussians in [0,1]: tilt vs ref (`track_sigma_deg` 1.5°), height vs ref when the goal has one (`height_sigma_mm` 20), leg unload force when the goal asks (`unload_sigma_n` 1 N). Perfect tracking ≈ +1/tick; ignoring the goal ≈ 0. |
| `track_sigma_deg` | 1.5 | tilt width of the kernel (widened to `unload_tilt_sigma_deg` 4.0 in unload episodes — leaning is the mechanism there). |
| `height_sigma_mm` | 20.0 | height width; wide so PARTIAL rises earn partial credit. |
| `k_roll`, `k_pitch` | 10.0 | small quadratic tilt shaping — gradient far from ref where the kernel is flat. |
| `k_height` | 100.0 | same, for height error (quadratic, in m²). |
| `k_gyro` | 0.05 | −k·Σ gyro² — quietness. |
| `k_action` | 0.005 | −k·Σ action². |
| `k_action_delta` | 0.01 | −k·Σ (a−a_prev)² — smoothness. |
| `k_current` | 0.005 | −k·Σ current² (A²) — "don't cook motors", kept weak. |
| `k_current_max` | 0.0 (off) | −k·max(|current|)² — prices load CONCENTRATION the sum-square misses. |
| `k_still` | 0.0 (off) | quiet-stance bonus: Gaussian on mean qd², MULTIPLIED by the kernel so a frozen belly-rest earns nothing. `still_sigma_rad_s` 0.3. |
| `k_unload` | 0.2 | weak linear gradient toward zero load on the unload leg. |
| `alive` | 0.0 | keep at 0 (see principles). |
| `safety_termination_penalty` | 10.0 | one-time −10 on safety termination (tilt trip etc.). |

## 2) Rise / lower / raise terms

`rl_move/sim/sim_env.py _post_step()` — episodes with a height target.

Income and one-time bonuses:

| cfg key (reward.) | default | what it does |
|---|---|---|
| `k_rise_progress` | 100.0 | potential-based +k·Δ|height_err| per tick (telescoping; full 50 mm rise ≈ +5). Freezing while the ref ramps away CHARGES. Never gated. |
| `k_rise_milestone` | 2.0 | one-time bonus at 25/50/75/90% of the signed height target. Scaled by the posture/plant gates below. |
| `k_rise_finish` / `rise_finish_sigma_mm` | 1.0 / 8.0 | narrow arrival kernel paid only after the ref has fully ramped — kills "park 20 mm low for 61% pay". |
| `rise_finish_gate_signed` | 0.0 (off) | =1 fixes a lower-mode bug: the legacy `ref >= target` ramp-done test is always-open for negative targets, so the arrival kernel paid a freeze at start height (~+57 of the freeze plateau's +74). |
| `k_curl_progress` | 120.0 | rise only: +k per meter of mean foot-XY distance closed toward the plant footprint. Potential-based — crouch starts and foot-parking earn nothing net. |
| `curl_milestone_mm` | [40, 15] | one-time `k_rise_milestone` each when mean curl distance first drops below each threshold. |
| `rise_hold_curl_sigma_mm` | 20.0 | during the pre-ramp HOLD window the tracking kernel is SWAPPED to pay curl distance instead of tilt/height stillness — otherwise lying frozen earns ~1/tick and preparation is priced as a loss. |
| `k_rise_ref_track` / `rise_ref_path` / `rise_ref_sigma_deg` | 0 / None / 12° | trajectory-scaffold: joint-space RMS kernel against a recorded rise (npz), time-aligned at ramp start. Seed the skill at full weight, then ANNEAL to 0 — not the objective. |

Income GATES (each in [0,1], scales income terms only — milestones,
finish bonus, post-ramp kernel; progress and penalties never scaled):

| cfg key (reward.) | default | what it closes |
|---|---|---|
| `rise_income_prog_gate` | 0.0 | freeze plateau: once the ramp leaves zero, income × fraction-of-target-covered. Measured: a frozen lower banked +74/ep, above every honest imperfect attempt. |
| `rise_posture_gate` | 0.0 | torso-at-height-feet-flying (bridge/flail): income × fraction of pads within `end_posture_allow_m` (0.02, stand) / `end_posture_allow_lower_m` (0.06, lower — honest lowers leave pads 17–43 mm up) of grounded z. |
| `rise_plant_polygon_gate` | 0.0 | stilt/splay/edge stands the clearance gate is blind to: income × continuous PLANT_SPEC factor (CoM depth in the down-feet support polygon, level attitude, body-frame footprint near plant anchors). Rise only. See RISE.md §spec. |

## 3) Stance-quality terms (global or stance-routed)

`sim_env.py`, all default OFF:

| cfg key (reward.) | default | routing | what it does |
|---|---|---|---|
| `k_current_hot` / `current_hot_a` | 0 / 1.0 A | global | −k·Σ max(current−threshold,0)² per servo — prices load concentration (one knee at 1.8 A hurts, six at 0.4 A free). |
| `k_support_margin` | 0 | global | ±k·clip(CoM depth inside support polygon, ±40 mm)/40 mm. Belly rest exempt (<3 contacts). |
| `k_load_even` | 0 | global | −k·(Herfindahl of foot forces − 1/n): even load charges 0, all-on-one-foot charges max. |
| `k_stance_contact` | 0 | hold/lean/track/unload/raise | +k·(loaded feet)/n — anti-tripod. Unload target leg excluded. |
| `k_stance_clearance` | 0 | hold/lean/track/unload | −k·Σ pad height above episode-start grounded z — dense gradient pulling hovering feet down. Raise exempt (refuted: collapsed raise 0/6). |
| `k_flag_leg` / `flag_leg_allow_m` / `flag_leg_walk_only` | 0 / 0.05 / 0 | all modes, or walk-only with the flag | −k·clearance above a 50 mm allowance — prices the parked vertical leg while normal swing stays free. All-modes routing REFUTED (taxed rise curls); use walk_only=1. |
| `k_end_posture` (+ `end_posture_ref_mm` 15, `end_posture_grace_s` 0.25, `end_posture_window_s` 1.5, `end_posture_allow_m` 0.02, `end_posture_allow_lower_m` 0.06, `end_posture_lower_dense` 0) | 0 | rise/lower/raise | terminal clearance charge, SCHEDULE-windowed (after the height ref settles): flag-leg endings pay, motion phase untaxed. `lower_dense=1` extends to the whole lower episode (no legitimate lift transient exists there). |

## 4) Walk terms

`rl_move/sim/walk_task.py _post_step()`, walk mode only by
construction. Hardcoded kernel constants: `K_WALK = 2.0` (peak),
`SIGMA_V = 0.05` m/s (width), `K_PROG = 1.0`.

Income:

| cfg key (reward.) | default | what it does |
|---|---|---|
| (kernel, always on) | K_WALK 2.0 | Gaussian on |v − v_ref| — up to +2/tick. |
| `k_walk_prog` | 1.0 | linear progress: k·clip(along-command speed fraction, −∞, 1.25). Negative when moving against the command. |
| `k_walk_yaw` / `yaw_sigma_rad_s` | 0 / 0.15 | yaw-rate tracking kernel, paid every walk tick incl. wz_ref=0 (heading-hold income prices drift). |
| `k_yaw_prog` | 0 | SIGNED rotation income on turn segments: k·clip(wz/wz_ref, −1.5, 1.25) — genuinely negative against the command (the Gaussian kernel never is). Anti-drift, see TURN.md. |
| `k_yaw_still` | 0 | quadratic drift charge on heading-hold segments (wz_ref=0): −k·wz². At the measured 0.09 rad/s drift, k=50 costs ~0.4/tick; gyro noise stays ~free. See TURN.md. |
| `k_phase_contact` | 0 | ±k on tripod-clock contact agreement (paired with `goal.walk_phase_obs`); parked/dragged legs average 50% = zero net. |
| `k_walk_swing` | 0 | one-shot +k per completed real swing (≥2 ticks airborne, lands ≥15 mm away). |
| `k_step_event` / `step_disp_budget_mm` | 0 / 0 | one-shot per-leg credit for a touchdown displaced ≥10 mm along command, scaled by along/30 mm cap 1.5×. The budget makes each paid credit CONSUME banked body displacement — cadence inflation and stride-in-place earn nothing by construction. |
| `k_quad_clear` / `k_quad_plant` / `quad_clear_cap_mm` | 0 / 0 / 30 | quad mode: pay lift-leg clearance (only while OFF the ground) and the loaded fraction of the four support legs, after `goal.quad_grace_s`. |

Income gates (each in [0,1]; scale kernel + positive progress only):

| cfg key (reward.) | default | what it closes |
|---|---|---|
| `walk_kernel_prog_gate` | 0 | the paid park: at 0.02–0.06 m/s commands the absolute-error kernel pays a parked robot up to 93% of peak. Income × clip(along/s_ref, 0, 1). |
| `walk_yaw_kernel_gate` | 0 | same construction for turn segments: yaw income × clip(wz/wz_ref, 0, 1). Hold segments stay ungated (that income IS the drift pricing). |
| `walk_kernel_yaw_gate` | 0 | the turn-in-place freeze floor (collapsed `cw-omni-mirror1-r1`, 08-11): on yaw-commanded ticks with NO linear command the LINEAR kernel pays a frozen robot full income (v_lin=0=ref; the prog gate needs s_ref>1e-3). Linear kernel × clip(wz/wz_ref, 0, 1) on those ticks; genuine stop segments (both refs ~0) stay paid. Freeze-floor bank in test_task_semantics.py pins it. |
| `walk_anchor_gate` / `anchor_tol_mm` | 0 / 10 | paddling: income × anchored fraction of loaded feet (loaded and within tol of own touchdown point). |
| `walk_loadslip_gate` / `loadslip_ok` 0.75 / `loadslip_max` 1.50 / `loadslip_floor_m` 0.05 | 0 | cadence-reset exploit of the anchor gate: income × factor of EPISODE-ACCUMULATED loaded slip per meter of progress (the same ratio the eval harness scores — no touchdown resets it). The `walk_loadslip_ratio` metric logs regardless of the gate. |
| `walk_height_gate` / `walk_height_sigma_mm` | 0 / 30 | hardware sag (08-10): deployed walk policies ride a COMMANDED crouch 54–70 mm below the spawn stance; base `k_height` (~0.36/tick at 60 mm) is outbid by walk income (~3/tick). Income × Gaussian on body height vs the episode `_z0` anchor — upright gait keeps 0.99 of income, the −51 mm crouch keeps 0.13 (probe + MDP_PREFLIGHT height bank). Symmetric, so stilting up is never a strategy. The `walk_height_factor` metric logs regardless of the gate. |

Charges:

| cfg key (reward.) | default | what it does |
|---|---|---|
| `k_drag_loaded` | 0 | −k per meter of foot XY translation while in contact (skating); 0.5 mm/tick deadband. |
| `k_park_duty` | 0 | −k·(per-leg contact duty outside [0.1, 0.9]) over a trailing 2 s commanded window — a tripod park pays ~0.6k/tick, a real gait pays nothing. |
| `k_walk_effort` | 0 | −k·mean servo current per walk tick (cost of transport; thermal load is the hardware-fatal quantity). |

## 5) Changing the reward — checklist

1. New terms: cfg-gated, default 0 = byte-identical legacy. Income
   modifiers are GATES, not additive charges.
2. Declare routing (global vs mode) and why.
3. Extend/run the MDP_PREFLIGHT bank
   (`pytest rl_move/tests/test_task_semantics.py`) — honest behavior
   must out-earn every known cheat BEFORE training.
4. Log the term into `parts`/`info` so it gets an `env/reward_*` curve.
5. Document the key HERE (one table row) in the same change.

Champion-comparability caveat: `SCORE/*_total_reward` values are only
comparable between runs with the same reward config — check the run's
`reward_cfg` before comparing (rl_docs/EVALS.md).
