# joystick - RL from the programmatic gait to joystick control

Last updated: 2026-08-22 (phasedir9b dig-in RESOLVED/FAIL:
init-basin selection, not checkpoint recency — see bullet below;
lineage rule adopted: repairs re-init from a pre-cheat checkpoint.
Earlier same day: phasedir9 VERDICTED UNDERTRAINED, not
FAIL: zero falls 24/24, gait_valid 6/6, slip/dir_err/speed all
inside gate, only progress narrow-missed 0.873x clone vs 0.9x cap
— best of lineage — while reward/tick was still climbing steeply
and drag charge still falling at 2M; continuing as phasedir9-cont1
per the 08-21 ruling instead of re-speccing the reward). Keep this a
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
- 08-22 phasedir3-8 lineage summary (full detail: RL_LOG + ledger
  verdicts; every arm zero falls, gait 6/6, clean video): six
  consecutive FAILs of the aligned-reward stack on the clone-relative
  rung-A gate, each refuting one lever class. phasedir3 (loadslip
  reprice to the noisy band): det slip unpriced -> 1.41x clone.
  phasedir4 (ent-coef anneal): std barely moved, worse. phasedir5
  (--warm-log-std-override -2.0, std held 0.13): progress finally
  0.984x PASS but slip 1.59x — noise-band theory REFUTED; dig-in
  found SLIP-FINANCED PROGRESS (lower-cadence/longer-stride/duty-skew
  drag family; bank-pinned + pricing A/B). phasedir6 (band retighten
  3/6): band VALUE lever refuted. phasedir7/7b (k_drag_stance 8000 ->
  4000): STEP FUNCTION — identical slow optimum at both doses, speed
  pinned 0.059. Dig-in found (A) allow=6 under the honest det stance
  tail + (B) instantaneous kernel taxing stride sway (income flat in
  speed); repaired as phasedir8 (allow 24, stride-EMA kernel
  reward.walk_kernel_vel_ema, k_prog 2; bank 29/29).
- 08-22 `cw-dep-bcgait4-phasedir8-emakernel-allow24` VERDICTED FAIL +
  DIG-IN RESOLVED (deep cycle; evidence
  `logs/ckpt_eval/pd8_digin_regime/`): det prog 0.770x / slip 1.254x
  (best of lineage) / speed 0.0575 — misses persist. ROOT CAUSE, now
  measured: the det/DR-0 pricing calibration DOES NOT TRANSFER to the
  optimization regime. probe_stance_slip_dist (new --action-noise-std
  / --dr-scale knobs) shows the honest clone at action-noise std
  0.135 pays 0.76-9.7x its income in drag-stance charge at allow=24
  (det: 0.002-0.36x); a CPU replication of the exact training env
  reproduces the run's -2.87/tick drag bill to the decimal. NO
  separating allowance exists: the noisy-honest per-stance tail needs
  >=48mm untaxed while the pd6 det drag-cheat pays ZERO beyond 36mm —
  the whole per-stance absolute-travel lever class is structurally
  refuted under PPO exploration noise. Secondary Warp-side defect:
  the EMA overspeed band divides by the RAMPING command speed, so
  ramp-in drift paid the -12/tick clip (W&B mean charge
  -2.38/charged-tick vs mean exceedance 0.002 m/s — arithmetically
  requires tiny s_ref; doesn't reproduce on CPU). Also: the run's
  falling ep_rew was an episode-LENGTH artifact (12->375 ticks),
  not a learning collapse. REPAIRED: (1) new
  `train_ppo_mjx --log-std-final/--log-std-anneal-frac` — forced
  log_std schedule (the proven override mechanism, scheduled) so the
  optimization regime CONVERGES to the det regime where full-stack
  pricing is measured-aligned (clone 1031 > pd7-slow 978 > pd6-drag
  639); (2) `reward.walk_course_overspeed_ref_floor_m_s` (default 0 =
  bit-exact) floors the overspeed reference. Bank 34/34 incl 5 new
  pd9 rows (floor inert off / spares ramp class / keeps insurance /
  orderings survive / regime gap pinned). LAUNCHED
  `cw-dep-bcgait4-phasedir9-stdanneal` (pd8 stack + std anneal
  0.135->0.04 by 60% of run + ref floor 0.06). NOTE: a concurrent-cycle race left a duplicate in flight under a different name, `cw-dep-bcgait4-phasedir9b-stdanneal` (train-2), running the identical config+seed alongside the original (train-3) — harmless (~5-10 GPU-min), verdict off whichever finishes against its own gate first, treat the other as a redundant confirmation.
- 08-22 CORRECTION + DIG-IN RESOLVED: the "duplicate" note above is
  WRONG — `-9b` shares the pd8 stack + anneal + seed 13 but continues
  from the cheat-committed pd8 CHECKPOINT while `-9` restarts from
  the raw BC phase clone. VERDICTED FAIL as the accidental A/B
  control: `-9b` came out WORSE than pd8 itself on every det gate
  axis (prog 0.704x clone vs 0.770x, slip 1.506x vs 1.254x, speed
  0.054 vs 0.0575; zero falls, gait 6/6, video: six legs cycling but
  walking in place) despite drag charge falling 4x — it dodged the
  bill by shrinking per-stance travel under the 24mm ABSOLUTE
  allowance (translating less, not slipping less). ROOT CAUSE of the
  -9/-9b divergence: INIT-BASIN SELECTION — `-9b`'s final TRAINING
  reward is equal-or-better than `-9`'s (ep_rew -90 vs -138,
  rew/tick EMA -0.048 vs -0.141, identical prog income 0.359), so
  the optimization-regime reward surface is ~flat across the honest
  and drag basins; at annealed-low std PPO is a local polisher and
  init decides the basin. Checkpoint-recency-at-anneal-start is NOT
  a general escape lever; pd8 branch (ii)'s "anneal earlier" framing
  is resolved as "re-init from a pre-cheat checkpoint". LINEAGE
  RULE: pricing/regime repairs re-init from teacher/clone, never
  continue a cheat-committed checkpoint. No further budget on `-9b`;
  the lineage's live arm is `-9-cont1` (finished, awaiting its own
  triage cycle: W&B ep_rew climbed to +152 over +4M steps).

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
2. **Evaluator half DONE 08-22** (`rl_move/sim/eval_joystick_gate.py`
   + `test_eval_joystick_gate.py`, 8/8 pure-aggregation tests, no sim
   touched): a reusable, versioned 60 s randomized joystick-session
   gate — pure orchestration around `eval_checkpoint.py` with ONE
   pinned held-out command bundle (`goal.walk_cmd_mode=stress_mix`
   [full family: random_hold, flip_180=reverse, sweep_circle/
   square=turns, stop_go, jitter] + `walk_cmd_resample_s=4.0` +/-
   jitter 0.5, `--episode-seconds 60`, seed base 90000 — a range no
   other harness/training run draws from), run at DR-0 + the
   checkpoint's own DR, det+sto, aggregated into ONE PASS/FAIL against
   the DONE gate's own numbers (zero falls, slip/m <= 2.9, dir_err
   median within 5deg of the teacher's 35deg floor). FIRST-EVER
   READING (n=12x2 det/sto, DR-0, `logs/ckpt_eval/
   phase1_clone_joystick_gate_v1/`): the phase clone
   (`ppo_goal_cw_bcgait_init_fullprof_phase1`) — previously verified
   only against FIXED heading-per-episode panels — FAILS the real
   randomized multi-segment session hard: zero falls / gait_valid
   24/24 (the gait itself never breaks), but slip/m med 15.9 (cap
   2.9) and dir_err med 65.4deg (allow 40deg) once commands actually
   change direction/stop/reverse mid-episode instead of holding one
   heading. This is the first measured, reusable confirmation that
   "passes the direction-first curriculum" (fixed headings) and
   "joystick controllable" (this DONE gate) are different bars — the
   gap RL fine-tuning (Next item 4) must close. STILL OPEN: per-leg
   gait metrics in the aggregate (eval_checkpoint already reports
   these per-episode; the gate script doesn't summarize them yet —
   cheap follow-up) and video-strip export (currently `--no-video`
   for eval speed; pass `--extra-cfg-set` through unaffected, video
   just needs re-adding to the wrapper's eval_checkpoint call when a
   candidate needs visual triage). Semantics-bank half NOT started
   dedicated to this exact 60s session (the walk task's own reward is
   already alignment-tested per-mechanism via `test_phasedir_
   semantics.py`, 34/34); a session-specific bank is lower priority
   until a candidate gets close enough to the gate numbers to need
   cheat-proofing at this exact horizon.
3. **CLOSED 08-22 (superseded by the regime-gap finding in Now):**
   the loadslip-band, drag-stance-dose AND drag-stance-allowance
   lever family is measured-refuted for this lineage — no
   det-calibrated per-stance/band charge can separate the noisy
   honest gait from the det drag cheat (no separating threshold
   exists). phasedir9 (`-stdanneal`) tested the remaining coherent
   repair: anneal exploration noise to ~det so the measured-aligned
   det pricing becomes the operative optimum. 08-22 RESULT:
   UNDERTRAINED, not exonerated or refuted — zero falls 24/24
   episodes (best of lineage), gait/slip/dir_err/speed all inside
   gate, progress 0.873x clone (best of lineage, still < 0.9x cap),
   reward/tick and drag-stance charge both still moving (climbing
   toward 0 / falling toward 0 respectively) at the 2M checkpoint.
   Per the 08-21 ruling, continuing from checkpoint
   (`cw-dep-bcgait4-phasedir9-cont1`, +4M steps, std held at the
   annealed floor) BEFORE any reward re-spec or the anchor/phase-lock
   dig-in below.
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
