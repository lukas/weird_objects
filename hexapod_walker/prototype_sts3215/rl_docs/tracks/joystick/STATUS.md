# joystick - RL from the programmatic gait to joystick control

Last updated: 2026-08-22 (phasedir8 FAIL + regime-gap DIG-IN
RESOLVED: det-calibrated pricing does not transfer to the noisy
optimization regime and no separating drag allowance exists — the
fix is annealing the noise itself; phasedir9 launched). Keep this a
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
  0.135->0.04 by 60% of run + ref floor 0.06).

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
3. **CLOSED 08-22 (superseded by the regime-gap finding in Now):**
   the loadslip-band, drag-stance-dose AND drag-stance-allowance
   lever family is measured-refuted for this lineage — no
   det-calibrated per-stance/band charge can separate the noisy
   honest gait from the det drag cheat (no separating threshold
   exists). phasedir9 (`-stdanneal`) tests the remaining coherent
   repair: anneal exploration noise to ~det so the measured-aligned
   det pricing becomes the operative optimum. If phasedir9 fails
   WITH std annealed and drag charge ~0 late, pricing is exonerated
   and the binding constraint is the BC-anchor/phase-lock family
   boundary (pd8's branch (ii)) — dig there, not at the reward.
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
