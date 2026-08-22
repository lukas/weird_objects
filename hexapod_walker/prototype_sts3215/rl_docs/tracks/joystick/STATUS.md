# joystick - RL from the programmatic gait to joystick control

Last updated: 2026-08-22 (phasedir9-cont1 VERDICTED FAIL-as-a-
continuation: +4M steps from the lineage-best checkpoint regressed
hard on every clone-relative axis — progress 0.873x->0.66-0.71x,
slip 1.08x->1.6-1.7x, speed dropped below the floor — matching the
SAME init-basin-flatness mechanism the concurrent phasedir9b dig-in
just root-caused, from the opposite direction [good init drifting
worse instead of bad init staying bad]. pd9 itself is unaffected,
still UNDERTRAINED/near-pass on its own 2M reading. LAUNCHED
phasedir9-seed17: same recipe, seed 13->17, tests whether pd9's
near-pass reproduces before further lineage investment. Also this
cycle: eval_joystick_gate.py per-leg-metrics + --video follow-up
landed (7/7 new tests)). Condensed 08-22 for the <=120-line budget —
full lineage detail lives in `RL_LOG.md` + ledger verdicts, not here.
Keep this a short screenful: Goal / Now / Next.

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
  the reward was not aligned with the eval. Per the 08-21 ruling
  those are MISALIGNMENT results, not dead ends. `phasedir1` is
  additionally ENV-CONFOUNDED (convention-corrupted sim).
- **phasedir2-8 lineage (full detail: RL_LOG + ledger verdicts;
  every arm zero falls, gait 6/6, clean video)**: eight consecutive
  FAILs of the aligned-reward stack on the clone-relative rung-A
  gate, each refuting one lever class in turn — staged curriculum
  (obedient but slow, 0.836x progress), loadslip reprice (slip
  unpriced 1.41x), ent-coef anneal (std barely moved), warm-log-std
  override (progress PASS 0.984x but SLIP-FINANCED — drag family,
  dig-in-pinned), band retighten (VALUE lever refuted), k_drag_stance
  8000/4000 (STEP FUNCTION, identical slow optimum both doses), and
  finally phasedir8's stride-EMA kernel (allow 24, `walk_kernel_vel_ema`)
  which still missed (prog 0.770x, best of lineage to that point) —
  dig-in found the det/DR-0 pricing calibration DOES NOT TRANSFER to
  the noisy optimization regime (no separating allowance exists
  between the honest noisy tail and the det drag cheat under PPO
  exploration). REPAIRED via `train_ppo_mjx --log-std-final/
  --log-std-anneal-frac` (forced noise-anneal so std converges to
  the det regime where pricing IS measured-aligned) +
  `reward.walk_course_overspeed_ref_floor_m_s` (ramp-drift insurance).
  Bank 34/34.
- **phasedir9 (anneal from raw BC clone) vs phasedir9b (anneal
  continuing the pd8 cheat-committed checkpoint), same stack+seed**:
  `-9` UNDERTRAINED/near-pass at 2M (zero falls 24/24, gait 6/6,
  slip/dir_err/speed all inside gate, progress 0.873x clone — best
  of lineage, narrow miss of 0.9x cap, reward+drag-charge still
  moving). `-9b` VERDICTED FAIL: WORSE than pd8 itself on every axis
  (prog 0.704x, slip 1.506x, speed 0.054) despite drag charge
  falling 4x — dodged the bill by shrinking per-stance travel, not
  slipping less (walking in place). ROOT CAUSE (dig-in resolved):
  INIT-BASIN SELECTION, not checkpoint recency — `-9b`'s final
  TRAINING reward is equal-or-better than `-9`'s, so this reward
  stack's optimization-regime surface is ~FLAT across the honest and
  drag basins at annealed-low std; PPO is a local polisher and INIT
  decides which basin gets polished. **LINEAGE RULE (binding for this
  reward stack): pricing/regime repairs re-init from a pre-cheat
  checkpoint (teacher/clone), never continue-train an
  already-converged one.**
- **phasedir9-cont1 VERDICTED FAIL as a continuation** (+4M steps
  from the `-9` checkpoint): regressed hard on every clone-relative
  axis (progress 0.873x->0.66-0.71x, slip 1.08x->1.6-1.7x, speed
  dropped below the 0.06 floor), zero falls/gait-6-6 preserved. W&B
  ep_rew crashed 27->-670 by ~1.4M then only partly recovered to
  152-213 by 4M — collapse-then-partial-recovery, not steady
  climbing. Same mechanism as `-9b`, opposite direction: a GOOD init
  drifted into a WORSE basin because the reward surface can't tell
  them apart at annealed-low std. `-9` itself is unaffected, still
  the lineage's best 2M reading. LAUNCHED `cw-dep-bcgait4-
  phasedir9-seed17` (same recipe, seed 13->17, `--now`, no reward
  change): tests whether `-9`'s near-pass reproduces on a second
  seed before any further lineage investment — do not respec a 3rd
  continuation off `-9`'s checkpoint without this reproducibility
  read first.

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
   gap RL fine-tuning (Next item 4) must close. **DONE 08-22**: the
   two follow-ups flagged here (per-leg duty/swing/sacrificed-frac
   aggregation across the whole panel, and an opt-in `--video` flag
   replacing the hardcoded `--no-video`) are landed and tested
   (`test_eval_joystick_gate.py` 11/11, +3 new per-leg tests incl. a
   regression test for the exact frozen-tripod pattern seen this
   cycle on the AMP track). Semantics-bank half NOT started
   dedicated to this exact 60s session (the walk task's own reward is
   already alignment-tested per-mechanism via `test_phasedir_
   semantics.py`, 34/34); a session-specific bank is lower priority
   until a candidate gets close enough to the gate numbers to need
   cheat-proofing at this exact horizon.
3. **CLOSED 08-22**: the loadslip-band/drag-stance-dose/allowance
   lever family AND the "anneal noise, continue training" lever are
   both measured-refuted for this lineage (see Now: phasedir6/7/7b,
   and the `-9`/`-9b`/`-9-cont1` init-basin-flatness finding). The
   open question is narrower now: is `-9`'s 2M near-pass itself
   reproducible (`phasedir9-seed17`, running) — if yes, the next
   coherent lever is a from-scratch run at 2M budget straight to the
   full-envelope curriculum (item 5) rather than any further
   continuation off an already-converged checkpoint; if the seed
   doesn't reproduce, dig at the BC-anchor/phase-lock family boundary
   next (pd8 branch (ii), never tried).
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
