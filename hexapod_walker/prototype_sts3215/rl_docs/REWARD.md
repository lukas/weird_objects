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
| `hold_still_gate` | 0 (off) | hold/track stillness+feet pricing (08-11, from `cw-stand-bc1-hard1`'s dig-in): the tracking kernel pays torso pose with no opinion on the legs, so hold/track converged to continuous leg-cycling (2M) and a frozen flag-leg park (10M) at near-full income — measured in the HOLD bank: legacy pays the flag pose 368.0 vs the quiet stand's 367.9 (a tie) and stepping 0.82×. Scales kernel income on hold/track ticks by feet-down² × HARD no-flag zero (`PLANT_SPEC.flag_leg_mm` 60 mm — honest adjustment swings stay below it) × stillness Gaussian (`still_sigma_rad_s`, applied only while the reference is stationary so TRACK's commanded motion is never charged). Blend `(1-g)+g·f`. Scoped strictly to hold/track: quad lifts legs and unload opens a contact on purpose; rise/lower/raise keep their own stacks. Gated ordering: quiet 368 > stepping 107 > flag 9.5 (bank, 3 seeds). Logs `hold_feet_factor` / `hold_still_factor`. |
| `hold_flag_fade` | 0 (off) | fade variant of the gate's no-flag factor (08-11, from `cw-stand-holdstill1` FAIL): the hard zero priced the flag park correctly but is a zero-gradient plateau — the trained run kept a leg parked ~110 mm for 2M steps because every nearby behavior also earned ~0. With fade=1 the no-flag factor ramps linearly over [`flag_leg_mm`, 2×`flag_leg_mm`] (60→120 mm): compliant poses keep exactly 1.0, the observed ~113 mm park earns scraps (51 vs quiet's 368, 0.14×) WITH a downhill slope toward feet-down, the ~190 mm class still earns 0 (12.6). Bank: ordering preserved, gradient exists (51 > 12.6), park stays <25% of quiet. Only meaningful with `hold_still_gate` on. |
| `k_unload` | 0.2 | weak linear gradient toward zero load on the unload leg. |
| `alive` | 0.0 | keep at 0 (see principles). |
| `safety_termination_penalty` | 10.0 | one-time −10 on safety termination (tilt trip etc.). |
| `term_cost_per_remaining_s` | 0.0 (off) | early-fall horizon cost (08-15, operator directive fb_20260815T114414): adds k × REMAINING episode seconds to the flat penalty on safety terminations (never on truncation), so a drag-then-fall cannot bank income a survivor would keep earning — `cw-mt-c2` retained ~+166/ep from ~6 s drag-then-fall at the flat −10. Bank-calibrated in the FULLCIRCLE bank (`test_task_semantics.py`): at k=12 a 6 s fall in a 60 s episode goes negative while the tripod gait/freeze orderings survive. Logged inside `reward_termination`. |

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
| `k_quad_clear` / `k_quad_plant` / `quad_clear_cap_mm` | 0 / 0 / 30 | quad-family modes (quad hold AND quadwalk, 08-13): pay lift-leg clearance (only while OFF the ground) and the loaded fraction of the four support legs, after `goal.quad_grace_s`. |
| `k_quad_still` / `quad_still_floor_m_s` | 0 / 0.005 | quad-family stillness (08-13; the learned quad hold creeps ~0.33 m/15 s — `hold_still_gate` exempts quad by design): −k·max(body planar speed − floor, 0) per tick, ONLY while no velocity is commanded (never fights a quadwalk command). |
| `k_quad_lift_contact` | 0 | quad-family (08-13, after cw-quadwalk1/2 closed pure income pricing: 3× clear/plant moved front tail duty only 1.0→0.62/0.32, never <0.15): −k·(fraction of commanded LIFT legs in ground contact) per tick after `goal.quad_grace_s`. Makes fronts-down walking strictly unprofitable; the honest lifted form pays ~0 by construction. Semantics: `test_quad_lift_contact_*`. |
| — quadwalk mode (08-13) | — | `--goal-mix quadwalk=<p>`: walk pricing stack + quad clear/plant income, with two lift-leg exemptions: the `k_park_duty` window spans only the support legs, and lift legs never earn step/swing credit (drag/slip charges still apply to them). Sampler keys: `goal.quadwalk_speed_min/max_m_s` (0.02/0.05), `quadwalk_heading_max_rad` (0 = fwd only), `quadwalk_hold_s` (2.0). Ordering bank SKIPS pending an accepted reference (test_task_semantics.QUADWALK_REFERENCE_BLOCKED); per the 08-13 operator ruling, training arms are LAUNCHABLE and the first policy passing the pre-registered gate in `rl_docs/tracks/quad/QUADWALK_REF_GATE.md` becomes the bank reference. |
| `walk_gait_gate` / `gait_gate_window_s` / `gait_gate_fade_s` / `gait_gate_stride_mm` | 0 / 2.0 / 2.0 / 10 | all-support-legs gait gate (08-13, quad track, after cw-quadwalk1-5 measured additive pricing exhausted for BOTH cheat families: quadwalk3 PAID the −575/ep lift-contact charge and kept six-legging; quadwalk5's 6× `k_park_duty` reprice changed the mid-leg-park scoot not at all — and the anchor gate never sees an air-parked leg, its fraction spans LOADED feet only): velocity income (kernel + positive progress, plus quadwalk clear/plant on commanded ticks) × [(1−g) + g·MIN over commanded SUPPORT legs of a per-leg "completed a real swing recently" score] — 1.0 if the leg finished a ≥2-ticks-airborne swing with XY stride ≥ `gait_gate_stride_mm` within the trailing `gait_gate_window_s` of COMMANDED ticks, linear fade to 0 over `gait_gate_fade_s` (fade, not hard zero — holdstill1 lesson). MIN, not mean: fractional discounts are measured-payable; sacrificing ANY support leg collapses transport income to the (1−g) floor by construction. Lift legs exempt; episode start counts as "just stepped" (window+fade commanded grace); penalties never shrink. Walk-family modes only. Semantics: `test_walk_gait_gate_*`. |
| `goal.quadwalk_start` / `goal.quadwalk_mid_splay_m` | "plant" / 0.06 | quadwalk spawn kind (08-13, after cw-quadwalk3 proved pricing exhausted: the −575/ep lift-contact charge fired and the policy still walked on six legs — exploration from the six-foot plant start is the blocker). `"quad"` spawns episodes ALREADY in the four-leg stance (env kind `quadstance`: TripodGait plant with mid feet splayed `quadwalk_mid_splay_m` forward — the bank's statically-surviving freeze stance; the bare plant+tuck pitch-trips — lift legs at the feasibility tuck claw, ±2° jitter). Tilt refs anchor to LEVEL like tipped starts (the limp settle sags the stance ~15–17° nose-down onto the claws; anchoring at the sag would train holding it and trip on recovery); runs enabling it MUST widen `safety.max_roll/pitch_deg` past the sag (25 = deployment envelope). Default "plant" = legacy bit-exact. Semantics: `test_quadwalk_start_*`. |

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
| `k_drag_stance` / `drag_stance_allow_mm` | 0 / 6 | structural stance-slip charge (charge-magnitude audit 08-11, `probe_drag_audit.py`): accumulated loaded XY travel per STANCE PERIOD, charged incrementally beyond the allowance — prices the dragging STROKE where the per-tick form cannot separate skating from touchdown scuff. Audit operating point k=7000/m @ 6 mm: a learned skater's drag cost ≈2.4× its income, the honest gait pays on <5% of stances. |
| `k_park_duty` | 0 | −k·(per-leg contact duty outside [0.1, 0.9]) over a trailing 2 s commanded window — a tripod park pays ~0.6k/tick, a real gait pays nothing. |
| `sched.key` / `sched.v0,v1,t0_steps,t1_steps,n_envs` (own `sched.` section, NOT `reward.`) | unset = off | **In-run coefficient scheduler** (08-13, nobc GAIT P3 lever 2 "annealed-up charge"): linearly ramps ONE existing cfg coefficient DURING a run, by GLOBAL env steps (env ticks × `sched.n_envs` — set n_envs to the run's `--n-envs`, it is REQUIRED and validated loudly). Applied in `_step_begin` on every stack (CPU + both MJX vec envs), clock is monotone per-process (never rewound by episode pool-restores; restarts on resume — use on fresh runs). Not a reward term: the scheduled key's own bank ordering must hold at the ENDPOINT value (e.g. `test_drag_stance_stack_prices_skating_below_stepping` pins k_drag_stance=8000). Live value logs as `env/sched_value`. CAVEAT: eval-harness envs sit at tick ~0 → the scheduled key reads ~v0 in evals; judge scheduled runs on measured behavior (slip/gait/travel), not eval reward panels. Tests: `test_coef_sched.py`. |
| `ease.gravity_scale` / `ease.vel_ceiling_scale` (own `ease.` section, NOT `reward.`) | unset = off (1.0) | **Physics easing** (08-13, nobc GAIT P3 lever 3): per-EPISODE multipliers on gravity magnitude (slope-DR direction preserved) and the servo velocity ceiling, read from cfg at EVERY reset so `sched.*` can anneal them in-run (eased physics early → nominal by t1; per-episode constant, never mid-episode). Not a reward term. Applied by scaling the episode's `_ep_rand` draw — the object BOTH stacks consume (private model `apply_to_model`; batched MJX per-world rows via `rows_for`/`tp_rows`) — so no DomainRandomizer change; with `randomize=False`, private-model envs (eval harness DR-0) use a direct fallback and shared-model shims raise loudly. Default 1.0 / unset = bit-exact off. DESIGN RULES for scheduled use: end at v1=1.0 so gates/evals and the run's tail are exactly nominal physics, and keep `t1_steps` below one eval episode's global-step equivalent (episode_ticks × sched.n_envs) so eval-harness envs (own sched clock from ~0) read nominal at every reset after episode 1 (episode 1 reads the pre-first-tick default = nominal). Batched-pool caveat: pooled resets can lag the schedule by a few episodes. Tests: `test_physics_ease.py`. |
| `k_walk_effort` | 0 | −k·mean servo current per walk tick (cost of transport; thermal load is the hardware-fatal quantity). |
| `k_drag_trans` / `drag_trans_allow_m` 0 / `drag_trans_allow_rise_m` 0.55 | 0 | **NON-walk modes** (rise/lower/raise/hold/track/lean/unload/quad): −k per meter of loaded foot-XY translation (0.5 mm/tick per-foot deadband) beyond a per-EPISODE allowance, charged incrementally as it accrues — the stand/sit foot-scrape the operator watches the robot do (08-11 night) was completely unpriced outside walk. A loaded foot that pivots/slides pays; a foot that lifts and STEPS to its new spot is free. Allowances are measured (probe 08-11): the demonstrated belly→plant rise inherently slides its pads 463 mm during the curl (rise/raise episodes default to a 0.55 m free budget), the honest anchored-feet lower and the quiet stand measure ~0 (everything else charges from the first excess mm). `trans_drag_mm` metric logs on every non-walk tick regardless of k (watch the dragging without coupling metric to price). TRANS-DRAG bank in `test_task_semantics.py` pins the orderings; tested operating point k=400. |

## 4b) GETUP mode — unified recover→stand→walk (08-11 redesign)

`rl_move/sim/walk_task.py _getup_reward()`, mode `getup` only (walk
env; enabled per-run via `--goal-mix getup=...` — the mode is absent
from every default mix, so all defaults below are inert = byte-exact
legacy). One episode = spawn ANYWHERE (tangle/zero/partial/crouch/
plant/park, built in `sim_env._reset_begin` `start_at="any"`), quiet
head, then a joystick velocity schedule. Falls are NOT terminal: runs
must widen `safety.max_roll/pitch_deg` (e.g. 60°); the tilt reference
anchors to gravity-level like tipped starts.

Structure (every prior stand lesson baked in): NO kernel income (a
level belly-rest is an alive bonus in disguise — `reward_task`/
`k_roll`/`k_pitch` are stripped on getup ticks; `h_err` is never fed);
income = one-shot staged ratchet + S-gated steady pay. The supported-
stand score `S = f_height · f_feet · f_level · f_footprint · f_flag`
is the RL_PLAN queue-2b structural height↔contact coupling: height
only counts when carried by MEASURED foot load. All factors are fades
(holdstill1 zero-gradient lesson).

| cfg key (reward.) | default | what it does |
|---|---|---|
| `getup_k_progress` | 60.0 | one-shot ratchet: +k·Δmax(P) where P = `getup_w_zero`·untangle + `getup_w_load`·weight-on-feet + `getup_w_stand`·S. Baseline seeds at the first tick (spawn posture is never income); regressions/re-farming pay 0 by construction. Full belly→stand ≈ +43. |
| `getup_w_zero` / `getup_w_load` / `getup_w_stand` | 0.15 / 0.25 / 0.60 | stage weights: joint-space proximity to the zero pose (untangle, `getup_untangle_deg` 60°); fraction of body weight on the feet `min(Σtouch/(0.85·m·g),1)` — bank-measured the ONLY scalar monotone along an honest rise from the crouch on (footprint barely moves during the curl, and the crouch is joint-wise FARTHER from the plant than zero is), Newton-capped so pressing harder is never a strategy; supported-stand score S. The curl itself is unpaid but never punished — the ratchet banks bests and the "any" start distribution backward-chains across it. |
| `getup_load_n` | 1.0 N | per-foot load saturation: `f_feet = (Σ min(touch/load_n,1)/6)²`. GRADED, not a threshold — bank-measured, an honest plant carries its light tripod at only ~0.7-1.2 N and must read ~0.95, while an airborne flag leg reads exactly 0. |
| `getup_z_belly_mm` / `getup_z_full_frac` | 38.0 / 0.80 | `f_height = clip((z−z_belly)/(z_full−z_belly),0,1)` with `z_full = z_belly + frac·(z_plant−z_belly)`; z_plant from plant-pose FK. Full credit at 80% of the rigid-FK span because servo/contact compliance sags the physical stance ~22 mm below FK (bank-measured 148.5 vs 170.8 mm). Overshoot fades to 0 over +20..+80 mm above FK plant — stilt pops price themselves. |
| `getup_level_deg` | 20.0 | `f_level` linear fade of true (gravity) attitude. |
| `getup_fp_ok_mm` / `getup_fp_hi_mm` | 40 / 120 | `f_footprint` fade of mean foot-XY distance to the plant anchors (PLANT_SPEC footprint bar, with slope). |
| `getup_flag_mm` | 60.0 | `f_flag` fade of the pad-height SPREAD (highest−lowest pad, world z — ground-reference-free so arbitrary spawns work) over [60, 120] mm: honest gait swings (~25-40 mm) keep 1.0, the video-confirmed 100-160 mm flag/tripod poses fade to ~0. |
| `getup_k_hold` | 0.8 | zero-command ticks: +k·S³·stillness Gaussian (`still_sigma_rad_s`) — quiet honest stands earn a living, partial/flagged stands earn scraps with a downhill slope. |
| (walk income, commanded ticks) | K_WALK/K_PROG | `S_gait·(kernel·clip(prog,0,1) + K_PROG·min(prog,1.25))` with `S_gait = f_h·f_level·f_fp·f_flag·min(load_sat/3,1)²` — a loaded tripod is full credit mid-stride; a belly-shuffle earns ~0 through f_h regardless of progress; a parked robot earns ~0 through the built-in progress gate. |

Per-tick metrics logged: `getup_S/P/best`, `getup_feet_loaded`,
`getup_f_load`, `getup_f_height/level/footprint/flag`,
`reward_getup_prog/hold/walk`, `getup_gait_gate`. Banked in
`test_task_semantics.py` (GETUP banks — measured orderings, 08-11:
honest replay ret ≈ +72, ratchet 0.88; freeze/flag-leg/thrash all
negative; the hip-0 "stilt" is in-sim a legitimate NARROW crouch-stand
whose fast time-to-stand beats the replay on episode TOTALS, so it is
priced per-tick: tail 0.20 vs the plant stand's 0.55. Walk side: gait
+378 vs park −1.6 / belly-shuffle −3.6).

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
