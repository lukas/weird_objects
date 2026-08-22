# joystick - RL from the programmatic gait to joystick control

Last updated: 2026-08-22 (phasedir5 dig-in verdicted: slip-financed
progress cheat root-caused; band retighten queued as phasedir6). Keep
this a
short screenful: Goal / Now / Next. Run detail lives in
`rl_docs/runs/`, W&B, and `RL_LOG.md`.

## Goal

Start from the simple programmatic gait (the scripted tripod teacher
and its BC clones) and use RL to make it genuinely joystick
controllable in sim.

**DONE gate (pre-registered, operator 08-21):** one policy (or the
session-controller stack) follows a randomized 60-second joystick
command script in MuJoCo — direction changes, stops, reverses, turns —
with:

- ZERO falls across the full panel (n>=12 episodes, det+sto, DR-0 and
  the run's own DR, held-out command seeds);
- directions actually followed (heading obedience judged against the
  teacher clone's measured ~35 deg tick-level stride-sway floor —
  compare deltas, not raw values);
- little slip: slip/m no worse than the scripted teacher's measured
  band at the calibrated plant (<= ~2.9; teacher band 1.4-2.9).

## Now (inherited state, 08-21)

- Scripted tripod teacher verified clean at the measured tibia-150
  plant: 0.06-0.10 m/s x 4 headings, zero falls, slip/m 1.4-2.9,
  full fast servo profile.
- Best starting checkpoint: the phase-conditioned BC clone
  `ppo_goal_cw_bcgait_init_fullprof_phase1` (holdout act err 0.0040)
  passes the entire direction-first curriculum with ZERO RL — all
  fixed headings incl. rear, irregular heading changes, stops.
- Fallback baseline: the download hierarchy
  (`footlow2_hard1` stance + `bcgait1_hard1` walk + session
  controller; held-out session gate det 0.967 / sto 0.853, n=600).
  Note: pre-08-22 checkpoints trained on the old 128 mm plant.
- Hard-won evidence: five fast-gait RL levers failed AS RUN because
  the reward was not aligned with the eval (faster cadence, tracking
  price, speed-obs+charges, more steps, phase-obs+fixed-speed).
  Per the 08-21 interpretation ruling those are MISALIGNMENT results,
  not dead ends: RL on this track requires a reward whose optimum is
  the 60 s gate behavior, then enough budget. The 5th verdict
  (`cw-dep-bcgait4-phasedir1`) is additionally ENV-CONFOUNDED (it
  trained on the convention-corrupted sim).
- 08-22 rung A verdict (`cw-dep-bcgait4-phasedir2-staged-fwd`,
  staged curriculum fb_20260822T032514, aligned-reward stack): FAIL
  on the pre-registered obedient-but-slow branch — zero falls,
  gait 6/6, slip 1.06x clone, dir_err -4.4deg BETTER, but progress
  0.836x clone (<0.9x) at the 0.060 band floor. Rung B NOT launched
  per gate. WIN inside the fail: the phase-locked BC anchor fully
  preserved the gait (first phasedir arm with zero behavioral
  damage). ROOT CAUSE: overspeed/loadslip charges are per-tick on
  the stochastic rollout, so exploration noise (std stuck 0.36,
  clone pays the same sto bill) pays them and the cheapest gradient
  is a slower mean gait. Charges must price stride-EMA/mean
  behavior, not tick noise, before any relaunch.
- 08-22 rung A verdict (`cw-dep-bcgait4-phasedir3-fwd-reprice`, the
  reprice from item 3 below): FAIL again, but a DIFFERENT and
  NARROWER miss than phasedir2. The reprice worked exactly as
  designed — walk_loadslip_factor sat 0.98-1.00 the whole run
  (training ratio ~5.0-5.4, never zeroed) and det speed matched the
  clone (0.069 vs 0.071 m/s, no det-overspeed, the recorded known
  hole q_20260822T0640Z did NOT fire) — but that same widened
  loadslip band (sized to spare the STOCHASTIC clone's noisy slip,
  4.8-6.4) left loadslip_excess ~0 for the whole run regardless of
  realized noise, so nothing kept DET-mode footfalls as clean as the
  clone: slip/m drifted to 1.41x clone (cap 1.15x, clear miss) while
  progress improved to 0.897x clone (cap 0.9x, still short but much
  closer than phasedir2's 0.836x). Zero falls, gait 6/6, contact
  sheet clean (no exploit) — a pure quantitative slip regression.
  ep_rew_mean fell steadily through most of training (quarters
  -37/-114/-176/-187) with only a small late uptick: reads as
  converged at 2M, not undertrained. ROOT CAUSE: PPO's action std
  stayed pegged 0.355-0.365 the entire run (never annealed), so ANY
  flat threshold wide enough for the noisy rollout is too wide for
  the tight det band the gate enforces. ACTED ON: built + on-pod
  smoke-tested `train_ppo_mjx --ent-coef-final/--ent-coef-anneal-frac`
  (rollout-end callback linearly annealing `model.ent_coef`, same
  pattern as the existing servo-profile ramp; default off = bit-exact,
  verified both paths on-pod). This is the "anchor dose/std
  annealing" lever the gate itself named as the next step. Rung B
  (heading-set respec) NOT launched per gate.
- 08-22 findings inherited from the reset window: the semantics-bank
  convention leak is repaired (`sim_gait_compat.py`) with a 7-test
  tibia-150 recalibration residue; the download hierarchy hard-fails
  the session gate at tibia-150. Fix arm `cw-dep-bcgait1-plant150-1`
  landed PASS(core) this cycle (0/6 falls DR-0+own-DR, session
  back-fall gone, fwd yaw -21.8->-10.6deg; promoted as the walk half).
- 08-22 bank recalibration (3/7 CLOSED, code landed
  `exp/c0822-riseref-bank-recal`): **CLOSED** — `trans_drag_honest_rise`
  (measured rise-curl drag grew 463->656mm at the corrected tibia,
  `reward.drag_trans_allow_rise_m` default bumped 0.55->0.75, band/
  docstring re-measured), `rise_rock_feedback_levels_it` (P-controller
  leveling peak grew 4.6-6.7deg->8.6deg, still zero terminations,
  bound relaxed 8->9deg), `recover_floor_rungs_remain_distinct` (the
  tangle_70/80 settle-spread gap narrowed to 1.9mm, margin relaxed
  2.0->1.5mm, ordering still strictly monotonic) — all pure re-measure-
  the-threshold fixes, no behavior change, no reward-mechanism edit.
  **STILL RED (4)**: `rise_valid_plant`/`score_replay_ends_in_valid_plant`
  (the rise reference's final pose still misses PLANT_SPEC —
  `height_ok: False` despite 6/6 feet down/no-flag/support/footprint
  all OK, i.e. a fine-grained height-window miss, not a fall; measured
  this cycle: h_err ~23.7mm vs the 15mm PLANT_SPEC tolerance, and the
  reference npz itself predates the tibia-150 change (file mtime
  Aug 11, before the Aug 21-22 geometry fix) — LEAD for next cycle:
  check whether `RISE_OVERRIDES`'s `goal.rise_height_mm=[108,114]`
  target window (not just the blend) is the stale-for-150mm constant,
  since the reference's own delivered height did not move but the
  tolerance math around it might need to),
  `getup_honest_ordering` (honest partial-crouch hold now pays LESS
  than freezing — a genuine reward-ordering defect, not a stale
  number), `fastprof_obeying_the_command_beats_overspeed` (separately-
  tracked fast-gait saturation, unrelated to the rise reference).
  BUILT + TESTED, NOT SHIPPED: `extract_rise_ref.py --blend-mode ik`
  (default) replaces the raw joint-angle lerp with a per-leg
  foot-anchored (FK/IK, `tripod_gait.foot_rz_from_hip_knee`/`_leg_ik`)
  Cartesian blend, plus `--validate-seeds`/`--validate-margin-deg` to
  replay a candidate into FRESH resets before accepting it (08-22
  finding: a candidate can be safe for its own extraction seed and
  still marginal elsewhere). Strictly better than lerp (found
  self-consistent candidates on 9/100 seeds vs 0/25 for lerp;
  lerp still falls on every seed 0-99, confirmed) — but the best
  candidate found from `ppo_goal_cw_stance_dr10` (a PRE-tibia-150
  checkpoint) still nets MORE bank failures than the current shipped
  reference (a NEW rise_rock_default_off_is_inert regression, 10.3deg
  vs the required <4deg) because that source checkpoint's crouch pose
  is itself asymmetric/splayed at the new geometry (some legs near
  max reach ~235mm of 240mm max, one leg folded to ~14mm) — no blend
  method fixes a bad source shape. REVERTED, not shipped
  (`rise_ref_belly2plant.npz` back to the pristine committed file).
  ROOT CAUSE, sharpened: the blocker is upstream of the blend — a
  fresh/fine-tuned STANCE source checkpoint trained AT tibia-150
  (mirroring `cw-dep-bcgait1-plant150-1`'s own geometry re-adaptation)
  is what's needed, not a smarter interpolation of the old one. That
  launch is itself a reward-mechanism arm on the SAME red rise bank
  (rise_valid_plant/getup residue), so it stays gated until those 2
  remaining items close by other means (PLANT_SPEC height-window
  fix; getup partial-crouch pricing fix) — CIRCULAR, filed as
  `OPERATOR_QUESTIONS.md` q_20260822T0600Z with the assume-and-go
  default (fix PLANT_SPEC/getup pricing first, since they don't need
  a training run to diagnose).

- 08-22 rung A verdict (`cw-dep-bcgait4-phasedir4-entanneal`, the
  ent-coef-anneal lever from item 3 below): FAIL, and WORSE than
  phasedir3 on both axes the reprice was supposed to fix. The anneal
  mechanism itself worked exactly as coded (wandb `ent_coef_anneal/
  value` fell 0.000951->0.0001, monotone, confirmed) but the resulting
  policy std barely moved (0.368->0.352 over the whole 2M-step run,
  vs phasedir3's pegged 0.355-0.365) — entropy bonus is too small a
  fraction of PPO's total loss to meaningfully drag a WARM-STARTED
  log_std down in 2M steps. Clone-relative (same control as
  phasedir3, `logs/ckpt_eval/phasedir3_clone_control_gate`): progress
  0.830x clone (cap 0.9x, worse than phasedir3's 0.897x), slip 1.518x
  clone (cap 1.15x, worse than phasedir3's 1.41x); zero falls, gait
  6/6, dir_err/speed both pass. Exactly the pre-registered
  "weak-anneal" branch. ACTED ON: built + on-pod smoke-tested a
  DIRECT lever instead of the indirect entropy-coefficient one —
  `train_ppo_mjx --warm-log-std-override <logstd>` forcibly resets a
  warm-started policy's log_std parameter(s) right after `--init-from`
  loads (works on plain-MLP `log_std` and gru-experts `_log_stds()`);
  verified on-pod it lands exactly on the requested value (-2.0 ->
  std=0.135 on all 18 dims) and is a no-op/no-print when unset.
  Launched `cw-dep-bcgait4-phasedir5-stdoverride` (train-0, respec of
  phasedir4, same unchanged reward stack, `--warm-log-std-override
  -2.0`) as the direct test of the noise-band theory.
- 08-22 `cw-dep-bcgait4-phasedir5-stdoverride` VERDICTED (dig-in
  cycle): gate FAIL on slip only (1.590x clone, cap 1.15x), noise-band
  theory REFUTED per its own pre-registered fork — std held 0.13 all
  run, progress 0.984x clone (cap 0.9x, first arm ever to pass) and
  dir_err 0.794x/28.2deg PASS, zero falls, gait 6/6. DIG-IN ROOT
  CAUSE (per-leg harness metrics, det DR-0, matched clone control):
  every phasedir arm converges on the SAME cheat gait family — swing
  time 0.255->0.335s, swings/leg ~30->~22, stride 0.0335->0.042m,
  duty skew (leg5 0.56-0.61 overstance vs tripod-A 0.42-0.47; clone
  is uniform 0.505-0.535); per-episode swing_s-vs-slip/m r=0.75 with
  ZERO overlap between clone and RL-arm clusters. Video clean (no
  exploit pose): the slip is loaded-foot DRAGGING that finances
  progress. WHY IT PAID: the loadslip band (ok=7.0/max=10.0, sized to
  the std-0.368 clone's noisy sto slip 4.8-6.4) never re-tightened
  after the std override — W&B rollout ratio sat 4.2-4.7, factor
  ~0.99, excess ~-0.01/step: slip was economically FREE while
  k_walk_course paid for longer-stride progress. Textbook 08-21
  MISALIGNMENT. ACTED ON: cheat encoded in the bank
  (`test_phasedir_*` loadslip-band pricing pins; scripted TripodGait
  twins CANNOT reproduce the drag regime — IK-clean, tops out at
  ratio 1.84 even at period x2/lift x0.35 — so the real cheat policy
  is the bank behavior via a matched-env pod pricing A/B,
  `logs/ckpt_eval/pd5_newband_ab_{clone,drag}`). Fix arm phasedir6:
  retighten the band to the std-0.13 regime (single change), gated on
  the A/B ordering flip (clone must out-earn the phasedir5 checkpoint
  under the new band in the training env).

## Next

1. **Close the last 2 rise-bank items with root-cause fixes, not
   re-measurement** (3/7 closed 08-22, see above): (a) PLANT_SPEC's
   height-window check on the demonstrated rise's final pose
   (`height_ok: False` with everything else OK — likely the window
   itself, not the pose, is stale for the corrected geometry); (b)
   `getup_honest_ordering`'s partial-crouch pricing (partial now pays
   LESS than freezing — a real reward-shape defect). Once those are
   green, `extract_rise_ref.py --blend-mode ik` (built+tested this
   cycle) can remint a compliant reference once a tibia-150 stance
   source checkpoint exists — which itself needs the bank green
   first (circular; see finding above). fastprof residue is a
   separate, already-tracked fast-gait item, not a blocker here.
2. **Build the gate harness**: a 60 s randomized joystick session
   evaluator (held-out command scripts, falls / heading obedience /
   slip-per-m / per-leg gait metrics, video) + a
   `test_task_semantics.py` bank proving the training reward ranks
   gate-passing behavior above every known cheat (park, paddle-creep,
   overspeed attractor, sacrificed leg).
3. **DONE 08-22 — noise-taxed charges repriced (phasedir3) FAIL; ent-
   coef anneal (phasedir4) FAIL and worse; direct log_std override
   built + launched as phasedir5.** phasedir3's reprice fixed
   det-overspeed but left det slip unpriced (1.41x clone, cap 1.15x).
   phasedir4 tested whether annealing `ent_coef` 10x would let std
   actually shrink from its pegged 0.355-0.365: the anneal itself
   worked (confirmed in wandb) but std barely moved (0.368->0.352),
   and both slip (1.518x) and progress (0.830x) got WORSE than
   phasedir3, not better — the entropy-coefficient lever is too
   indirect/weak on a warm-started log_std. See the `Now` bullets
   above for full tabulation. NEXT ARM (launched, `cw-dep-bcgait4-
   phasedir5-stdoverride`, train-0): the newly-built default-off
   `train_ppo_mjx --warm-log-std-override -2.0` forcibly resets the
   warm-started log_std to std=0.135 right after `--init-from` loads
   (verified on-pod: lands exactly on the requested value, no-op when
   unset) — a direct test of the noise-band theory instead of hoping
   gradients get there. PASS -> rung B heading-set respec. FAIL with
   std confirmed low in eval = noise-band theory REFUTED, DIG-IN
   required before any further reward edit (do not launch another
   anneal/override variant blind).
4. RL fine-tune from the phase clone (and a walk-champion arm as
   control) with the reward aligned to the gate metrics, resuming
   the staged heading curriculum; extend budget while reward and
   gate metrics rise together.
5. Widen the command distribution toward the full joystick envelope
   (speeds, yaw, strafe, stops) and DR-harden to own-DR zero-fall.

## Rules of the road

- Reward rising + gate metrics bad = realign reward with the gate
  and/or continue longer — never a one-line FAIL
  (`RUN_INTERPRETATION_RULES.md`).
- Do not park on operator input; assume-and-go with a recorded
  assumption. Physical-robot items are the only true waits.
