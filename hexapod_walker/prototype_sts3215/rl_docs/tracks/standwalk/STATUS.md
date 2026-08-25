# standwalk — mesh-model stance retrain, then distill into walking

Last updated: 2026-08-25 ~05:2x (**stage-1 gate reads in: 2/3 seeds
CATASTROPHIC, DIG-IN flagged, both runs left UNVERDICTED for the deep
cycle.** `cw-standwalk-stance-mesh1-rr1` (seed 0) and `-seed2-rr1`
both finished 20M steps; `-seed1-rr1` still training this cycle (left
alone per assignment, not read here). Plain English: the ported
footlow2-style stance recipe, bank-checked clean on mesh pre-launch,
collapses on the ACTUAL stage-1 gate (pod_eval joint_goal panel,
hold/rise/lower, n=6/mode, DR-0 + own-DR 0.2, det+sto) almost every
episode, on both completed seeds:
- `rr1` DR-0 gate: 33/36 episodes terminated (31 `over_current`,
  2 `tilt_roll`), `cur_max_a` pinned at exactly **2.64 A** on every
  over_current episode (the trip ceiling, not a soft overshoot) —
  fwd drift only 0.02-0.11 m/15s (not walking away, just failing in
  place). own-DR(0.2): 28/36 terminated (23 `over_current`, 4
  `tilt_roll`, 1 `tilt_pitch`), `cur_max_a` 2.52-2.66 — same signature,
  slightly less severe than DR-0 (noise-injected joints likely break
  the exact load-lock pose sometimes).
- `seed2-rr1` DR-0 gate: **36/36** (100%) terminated — 17
  `tilt_pitch`, 10 `over_current`, 9 `tilt_roll`; own-DR(0.2):
  **36/36** terminated too, evenly split 12/12/12 across the same
  three reasons. `cur_max_a` up to 2.66 A.
- Video (rise_det, seed2): the rise motion itself looks basically
  right for the first ~2/3 of the ascent (frame-by-frame: flat ->
  crouch -> good mid-height stance under the body) and then the body
  **rolls/tips sideways** partway up and never recovers (`tilt_roll`
  term) — reads as a lateral-balance margin failure during the
  dynamic rise, not a scripted/static posture bug. Video (hold_det,
  rr1): starts at a normal-looking stand and progressively **splays/
  sprawls** into a twisted, legs-akimbo crouch over the hold window,
  consistent with the load-imbalance -> `over_current` signature
  (some servos pinned near max duty while others idle).
- Training reward: `rr1` rose net positive by the end (quarters
  -245/-581/-127/**+90**) while its in-training eval callback's
  survived_frac/height_err ALSO looked fine at the very last logged
  point (hold survived 1.0, height_err 2.7mm @ 19M) — this
  contradicts the harness verdict above; the in-training eval is
  either a much easier config (shorter horizon / no full posture
  check) than the pod_eval gate, or an evaluator-loophole. `seed2-rr1`
  stayed net negative the whole run (quarters -265/-471/-149/-93) with
  survived_frac ~0 from ~8M onward. Per the 08-21 ruling this reads as
  MISALIGNED for `rr1` (reward up, gate catastrophic) but closer to a
  genuine stuck/FAIL signature for `seed2-rr1` (reward never really
  escapes negative, eval flat-bad throughout) — the two seeds do NOT
  even agree with each other on which failure mode this is, which is
  itself informative (recipe-level instability, not one unlucky seed).
- **CROSS-TRACK CORRELATION (not yet root-caused, flagging for
  whichever cycle digs in):** the joystick track's mesh-family runs
  this same 08-25 cycle (`gaitgate-scratch1`, `tf64-mesh-acq1`) also
  died via `over_current` with `cur_max_a` in the same **2.6-2.7 A**
  band, on a completely different task (walk, not stance) and
  completely different reward stack. Two independent tracks hitting
  the identical current ceiling on the same mesh model in the same
  cycle is suspicious for a SHARED root cause (servo current-trip
  threshold/model miscalibrated for the 3.5 kg mesh body's real
  torque demand, or both reward stacks independently lacking any
  current-aware shaping) rather than two unrelated per-track
  misalignments. The joystick track already has deep tooling/history
  on this exact signature (`reward.k_walk_current`, current-dwell
  charges) — worth checking whether that machinery (or its lesson)
  transfers to `joint_goal`/stance before inventing a parallel fix
  here.
- **Per the gate's own reading rule** ("2-3/3 healthy = recipe
  robust; 0-1/3 = seed-dependent or recipe gap"): 0/2 read so far is
  already at the recipe-gap threshold regardless of how `seed1-rr1`
  lands.
- **DIG-IN, not verdicted**: this is a stage-1-gate-deciding fork
  (whether the ported recipe needs a current/balance-aware reward
  redesign before any further seed spend, per the 08-21 "audit
  reward/eval/sim before more budget" rule) — leaving
  `cw-standwalk-stance-mesh1-rr1` and `-seed2-rr1` UNVERDICTED for a
  deep-toolkit read (per-leg current traces, matched-parent-style
  current decomposition, and a check of whether the mesh servo
  current-trip constant itself is calibrated correctly) rather than
  recording a shallow PASS/FAIL this cycle. Evidence:
  `logs/ckpt_eval/cw_standwalk_stance_mesh1_rr1_gate/`,
  `..._seed2_rr1_{gate,owncfg}/`, W&B `hq7zyih9`/`bpbesoyb`.

Previous entry (2026-08-25 ~03:3x (operator registration — track
created, nothing launched yet).

## Goal (operator, 08-24 evening)

Retrain the best rising-and-lowering (stance) model on the NEW mesh
MuJoCo model at 100 Hz, then use it as a teacher to distill rise/lower
plus the best walking behavior into one policy. Product: a single
mesh-family 100 Hz policy that, starting from sit, rises, follows a
randomized 60 s joystick session with zero falls, and lowers back.

## Binding constraints (why this is a retrain, not a resume)

- Families do NOT transfer (CURRENT_TRUTHS "SIM MODEL FAMILIES"): the
  legacy stance champion `ppo_goal_cw_stance_dr10` and walk champion
  `ppo_goal_cw_dep_bcgait4_phasedir9_stotight45_seed13` are
  primitive-family 25 Hz policies. NO `respec --from` / warm-start of
  them onto mesh — stage 1 is a recipe rerun on the new model.
- New launches already get `control.hz=100` (launcher-injected) and
  `env.model_source=mesh` (the default) — do not pin legacy values
  here, and never pin `model_source=primitive` in this track.
- Legacy champions MAY be queried as teachers (same obs layout), but
  they carry 25 Hz action scale and primitive dynamics: any
  distillation mechanism must handle the 25->100 Hz gap (query at
  25 Hz + interpolate, distill trajectories, DAgger with rate
  conversion, ...) and must MEASURE whether primitive-trained advice
  is good on mesh dynamics before trusting it.

## Stage 1 — mesh/100 Hz stance retrain (rise + lower)

Recipe basis: the `stance_dr10` lineage recipe (exact cfg in the
ledger/W&B). The rise-reference machinery (`extract_rise_ref.py`,
rise bank) is green as of 08-24. Bank/semantics-check the stance
reward ON MESH before the first launch (mass went 2.104 -> 3.50 kg;
thresholds calibrated on primitive may rank behaviors differently).

GATE (pre-registered): stance panel rise/hold/lower (pod_eval stance
modes), n>=12, det+sto, DR-0 + own-DR: zero falls/tips, quiet hold
(no creep), rise/lower height tracking comparable to the legacy
champion's band. Absolute numbers shift with the +66% mass — the
first passing run's numbers become the recorded mesh reference band.

## Stage 2 — teacher distillation into the best walking model

Use the stage-1 policy as the rise/lower TEACHER. Walking source: the
joystick champion lineage (`stotight45-seed13`) or its mesh-era
successor if the joystick track's in-flight mesh arms produce one
first — either adoption is PRE-REGISTERED here, never a silent
teacher swap (cpg containment rule applies). Mechanism is
cycle-designed (BC clone + RL fine-tune a la bcgait, KL-to-teacher,
phase-scheduled multi-teacher, ...); every mechanism arm pre-registers
its gate and a matched control.

DONE GATE (the track's): ONE mesh-family 100 Hz policy, from sit:
rise -> randomized 60 s joystick command script -> lower to sit.
Zero falls, directions followed, slip/m within the joystick band
(<=~2.9), held-out panel n>=12, det+sto, DR-0 + own-DR.
`eval_joystick_gate` covers the walk segment; the sit->rise->walk->
lower session harness is stage-2 tooling to build.

## Now

Stage-1 batch RUNNING 08-25 ~04:4x (operator kick):
`cw-standwalk-stance-mesh1-rr1` + `-seed1-rr1`/`-seed2-rr1` on
train-0/1/2, 3 seeds one wave, 20M each, from-scratch footlow2-style
joint_goal on the mesh default @100 Hz (goal-mix hold=.1/rise=.45/
lower=.45, 15 s eps, DR 0.2, log-std 0, ent .005, rise-ref tracking
k=2.0 + posture/income/finish gates + hold_still_gate/hold_flag_fade
+ rise RSI 0.5, NO warm start / NO bc_anchor). The un-suffixed first
wave crashed at startup: `bus.servo_params=loaded` carries a measured
125 ms latency > the MJX backend's 12 pending-command slots at hz=100
(dt=0.01) — pin dropped (fit at 25 Hz; loaded-latency robustness
deferred to a hardening rung), W&B names burned, relaunched as -rr1.
rr1 (seed 0) already completed 20M in ~22 min: final periodic eval
rise/lower 0/2 with over_current terminations — triage judges vs the
pre-registered ladder alternative (20M@DR0.2 = first rung).

Stage-1 mesh calibration facts (measured 08-25, kick cycle):
- Mesh plant settles at h_rel = **82.96 mm** (primitive tibia-150:
  131.94) => `goal.rise_height_mm=[79,87]`, `actions.max_height_mm=88`.
- The 25 Hz `rise_ref_belly2plant.npz` EXECUTES on mesh: time-aligned
  open-loop replay ends valid plant 3/3 seeds. (The bank's replay was
  rate-broken at hz=100 — fixed test-side via `_ref_row`; the trainer
  consumer `_rise_ref_clock` was always time-based.)
- Bank-check ON MESH under the exact launch stack: RISE replay 2703
  (plant 3/3) > mesh-honest partial 536 > flagleg 395 > stilt -47 >
  freeze/thrash negative; LOWER honest 2131 > partial 629 > refuse
  -64 (<0), posture-strict rejects outrig/aloft; HOLD quiet 1472 >
  stepping 870 > flag 50. Caveats: (a) LOWER thrash seed 2 lucky-
  collapses the 3.5 kg body onto the belly target (banks 1852; thrash
  mean 869 > partial 629, mesh-only; honest dominates 2.4x) — watch
  lower videos for crash-lowering; (b) the committed bank's "partial"
  (primitive j_half row) yields h=1 mm on mesh — a mesh partial must
  hold ref row ~283 (~50 mm); (c) endpost-era shaping extras
  (k_stance_clearance/k_end_posture/k_load_even/k_still/k_current_*)
  BREAK bank orderings on mesh (partial below cheats, or k_still pays
  refusal) — deliberately excluded; re-add only via a bank-proven
  hardening rung.

## Next

1. DONE 08-25 ~04:4x (see Now): recipe ported, bank-checked on mesh,
   3-seed batch launched.
   RECIPE ARCHAEOLOGY (08-25 ~04:0x cycle, saves the re-dig):
   `stance_dr10` is PRE-LEDGER (no extra_args entry); its W&B config
   (run `cw-stance-dr10`, 2026-08-08) shows: task `joint_goal`,
   3M steps, warm from `cw-stance-dr08` (ladder even->clear->raisefix
   ->dr08->dr10, DR 0->1.0), init lineage rooted at `init.zip`, and —
   critically — the 6-channel BODY-POSE action space (roll/pitch/
   height/x/y/curl via fixed-foot body IK), NOT the 18-joint space.
   The better-documented stance recipe is the `footlow2` lineage
   (`cw-stand-footlow2-hard1[-s1]`/`-stable1`, full extra_args in the
   ledger, PASS, 4-clause gate incl. eval_session): joint_goal,
   goal-mix hold=0.1,rise=0.45,lower=0.45, rise-ref tracking
   (`reward.rise_ref_path=rl_move/sim/refs/rise_ref_belly2plant.npz`)
   + posture/income gates — but every footlow2 run warm-starts a
   primitive checkpoint and uses `train.bc_anchor_coef=1.0`, both
   impossible on mesh (families do not transfer). So stage-1 needs
   either (a) a from-scratch footlow2-style arm minus init/bc-anchor
   (2M mechanism canary first), or (b) a rerun of the pre-ledger
   discovery ladder. The rise ref is a JOINT-SPACE 25Hz trajectory
   (`q_rad`,`dt=0.04`,`ramp_i0`) extracted from the primitive
   champion (`extract_rise_ref.py` REQUIRES a policy that rises — no
   mesh champion exists yet, so stage-1 must reuse the primitive-
   extracted ref and measure whether it rises the 3.50 kg mesh model;
   the training-side consumer `sim_env._rise_ref_clock` is time-based
   so hz=100 is safe, and the 08-25 bank rate fix (`_ref_row`,
   test_task_semantics.py) makes the bank honest at hz=100 too).
   Mesh current context (hist64-mesh-acq1 dig-in, 08-25): standing
   holds on mesh draw ~0.15 A mean/servo, max 0.41 A (probe_hold_
   current.py, loaded params) — far under the 2.5 A trip, so stance
   itself is NOT current-constrained on mesh; only load-concentrating
   cheats would trip.
2. Record the legacy stance champion's primitive-band panel numbers
   in the first triage as the comparison reference; extend pod_eval's
   stance panel for mesh if any flag is missing.
3. After the stage-1 gate: pre-register the stage-2 walking-source x
   mechanism matrix and launch it as a BATCH.

## Landmines

- Sim only — hardware stand/plant transfer stays operator-owned.
- No stage-2 arm may warm-start from a primitive checkpoint.
- The joystick track owns generic mesh walking; this track owns
  rise/lower + the unification. Coordinate via STATUS, don't
  duplicate its mesh conversion arms.
