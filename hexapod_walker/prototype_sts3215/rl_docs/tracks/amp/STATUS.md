# amp - AMP locomotion from scratch

Last updated: 2026-08-22 (M2 freeprog DIG-IN RESOLVED: the topple was
SUICIDE ECONOMICS, a pricing defect — with per-tick charges ~-1.4 to
-3/tick and reward.term_penalty=0, dying was FREE; a scripted 1 s
topple netted +19/ep vs park -243 / stall -143, the best-paying
behavior in the bank short of walking. Both arms LEARNED survival
first (ep_len 28->310) then flipped to fast death in q4 (tilt terms
59->132 / 90->241) at CONSTANT std 0.367 — so log-std anneal and
charge ramp-in are both REFUTED as fixes; style05's q4 reward
"recovery" is confirmed faster death (ep_len 292->230, terms x2.7).
AMP exonerated (healthy all run) but a style channel cannot price
termination. Cheat encoded:
test_slipwalk_toppling_fast_is_not_an_escape + term_penalty=400 in
SLIPWALK_OVERRIDES (topple -381 < park; bit-exact for survivors; bank
7/7 PASS, commit d9554b04). Fix pair LAUNCHED (style05 train-1, noamp train-3, + accidental bit-identical dup noamp-rr1 train-2 = free repro replicate):
cw-amp-m2-freeprog-term400-{noamp,style05}, single change
reward.term_penalty=400 — re-runs the Wave-1 style-vs-control fork
the suicide basin short-circuited. Prior finding for lineage: the
M2 -c1 dig-in found both
legacy-priced pilots MISALIGNED — a statue paid ~1.9/tick
(rise_finish + posture/height kernels + the sigma-0.05 velocity
kernel paying ~0.45/tick to v=0 across low/stop commands) while
locomotion income was an unreachable needle from scratch — ALL 38M of
reward rise was statue-polishing (rise_finish 0.09->0.86/tick,
walk_speed flat). The AMP mechanism was healthy all run (d_real 0.97
vs d_fake -0.96, never saturated) but its ~0.03/tick effective style
income was priced out ~30-60x. Cheat encoded:
test_slipwalk_stork_statue_is_priced_out (stork statue -238 vs gait
+558, bank 6/6 PASS).
Charter:
`rl_docs/AMP_LOCOMOTION.md` (binding, incl. the repo-adaptation
section — no Isaac Lab, MJX/Warp is the primary trainer). Keep this a
short screenful: Goal / Milestones / Now / Next.

## Goal

One compact learned policy, trained from scratch with Adversarial
Motion Priors + massively parallel PPO + privileged critic +
observation history + actuator/fault randomization, that:

- accepts continuous joystick commands (vx, vy, yaw_rate);
- produces a smooth alternating-tripod gait;
- starts, stops, reverses, strafes, turns without phase resets;
- recovers from pushes; degrades gracefully under joint/leg faults;
- transfers to plain MuJoCo unchanged (M5 = track DONE; M6 hardware
  is operator-owned).

The demonstration gait is training data, not the deployed controller.
Build every tool this needs; do not pause on operator input.

## Milestones (brief §13)

- M0 infrastructure: IN PROGRESS (actor/critic split + GRU/history +
  command generator + the widened joystick envelope all now confirmed
  composable on the primary trainer, 08-22 smoke; discriminator,
  motion library, replay buffer, and fault injection still open —
  see Now/Next)
- M1 motion library: **DONE 08-22** (generator + v1 dataset;
  discriminator core + style reward + banks; live reward-loop wiring
  landed and smoke-verified — see Now item on AMPStyleVecWrapper).
  **FRAME AUDIT 08-22 (fb_20260822T145428): teacher_v1.npz is
  convention-corrupted** — `build_motion_library.py` imported RAW
  `tripod_gait.TripodGait` (absolute-tibia since 30660b51) and fed
  `desired_deg()` to the sim unconverted, with the SIM-RELATIVE
  canonical plant (20,80) misread as absolute; measured stream
  divergence vs the verified `sim_gait_compat` path: knee up to
  15.7 deg (mean 80.6 vs 85.2), coxa up to 4.9 deg. v1's clips are
  physically valid (15/15, low slip) but are NOT the verified
  teacher's gait. FIXED + REBUILT same day: builder now imports via
  the `sim_gait_compat` boundary; shipped
  `rl_move/sim/motion_library/teacher_v2.npz` (45/45 clips accepted,
  264 s, slip/m 0.45-2.03 all inside the teacher band; loads clean
  through `MotionLibrary` incl. the neutral-consistency hard-fail;
  discriminator bank 8/8 still green on the untouched v1 default).
  v1 kept append-only; ALL FUTURE AMP launches must point the
  discriminator/motion-prior at teacher_v2 (in-flight 08-22 runs
  trained against v1 — interpret their style numbers with that
  caveat).
- M2 beautiful normal gait: IN PROGRESS (pilot pair -> -c1 statue
  MISALIGNED -> freeprog pair FAIL by suicide economics (dig-in
  resolved 08-22, see banner) -> term_penalty=400 fix pair RAN:
  noamp control (+bit-identical -rr1 repro) VERDICTED FAIL 08-22 —
  suicide fix held (0/12 terminations) but the control still
  shuffles in place (fwd travel 0.026-0.032m vs 0.10m bar,
  freeprog_pen flat ~-1.5/tick the whole run at constant std=0.368,
  genuine not-learning). style05 twin owned by a concurrent cycle —
  Wave-1 style-vs-control read still pending that verdict. Follow-up
  QUEUED+RAN same cycle on idle capacity, single lever:
  `cw-amp-m2-freeprog-term400-stdanneal` (--log-std-final=-2.0
  --log-std-anneal-frac=0.5 on top of the noamp control) — VERDICTED
  FAIL, decisively: std 0.368->0.135 made the SAME stationary basin
  MORE regular (gait_valid 6/6 det+sto vs noamp's 3-5/6, tight slip
  10.3-11.5/m) while fwd travel got WORSE (0.005m vs noamp's 0.026m)
  and reward_per_tick_ema got WORSE (-3.29 vs -2.84) — a textbook
  in-place march. `env/reward_walk_freeprog_pen` plateaus flat
  regardless of noise level. CONCLUSION: this is a REWARD-SHAPE
  defect (`walk_freeprog_score` has a real local optimum at stable
  in-place cycling, no net-displacement floor), NOT an exploration
  problem — std/anneal/entropy levers CLOSED for this reward family.
  Next candidate fix (untested, needs a semantics-bank test first):
  a freeprog analog of `k_walk_idle_charge` keyed to episode-window
  net displacement, not instantaneous speed.
- M3 push recovery: NOT STARTED
- M4 fault adaptation: NOT STARTED
- M5 MuJoCo transfer (= DONE gate): NOT STARTED

## Now

08-22 audit of the shared MJX/Warp stack against the M0 checklist
(§17 item 1: reuse before building) found more already reusable than
the 08-21 "nothing built yet" note assumed, plus one real gap that's
now closed:

- **DONE THIS CYCLE**: `--asym-critic` (privileged critic obs split)
  ported from `train_ppo_sim.py` (CPU/SB3) to `train_ppo_mjx.py`
  (the GPU/Warp primary trainer) — it did not exist there before
  (`git rev b69a46e2`). Reuses `asym_policy.AsymActorCriticPolicy`
  and `train_ppo_sim._privileged_idx` verbatim (no duplicated logic);
  additive-only (80 insertions, 0 deletions), new flag defaults off.
  Verified on-pod (`hexapod-mjx-train-0`, smoke `smoke-amp-
  asymcritic-mjx`): 2048/2048 steps trained, finite losses, and the
  saved checkpoint loads as `AsymActorCriticPolicy` with
  `privileged_idx=(70,71)` correctly masking the walk task's 2
  measured-velocity obs dims on the actor path only. Warm-start
  transplant path (MLP champion -> asym policy) ported too, mirroring
  train_ppo_sim's proven weight-copy.
- **ALREADY THERE (confirmed, not built this cycle)**: GRU/history
  actor + deterministic recurrent eval (`--gru`, `--gru-dual`,
  `--gru-experts`, `gru_policy.py`) and a causal-transformer
  alternative (`--transformer`) are already live on `train_ppo_mjx.py`
  — M0's "recurrent state resets correctly" checkbox is largely
  covered already. Domain rand (`domain_rand.py`), per-world model DR,
  canaries, eval/video logging, and desync are all live per
  `guardrails.yaml`.
- **ALREADY THERE, PARTIAL**: `walk_task.py`'s existing command
  sampler (`goal.walk_cmd_mode=stress_mix` with
  `random_hold/flip_180/sweep_circle/square/stop_go/jitter`, plus
  `goal.walk_yaw_cmd` for yaw-rate) already draws continuous
  (vx, vy[, wz]) commands with hold/ramp/resample/stop segments —
  most of AMP §6's shape. Gaps vs. the brief: current defaults are a
  narrower (speed, heading) envelope, not the full independent
  vx in [-0.35,0.60] / vy in [-0.30,0.30] / yaw_rate in [-1.0,1.0]
  band with the brief's exact resample (0.5-3.0 s) / zero (0.15) /
  abrupt-change (0.35) probabilities, and the measured-velocity obs
  goes to BOTH actor and critic today — the new `--asym-critic` flag
  fixes the second half; the envelope is a cfg-tuning task, not new
  code.
- **DONE THIS CYCLE (M1 start)**: `rl_move/sim/build_motion_library.py`
  — generates + validates the §4 motion-prior dataset from the
  hardware-proven scripted teacher (`linux_control/tripod_gait.py`
  TripodGait) driven through REAL MuJoCo physics (not a bare
  kinematic replay) at the measured tibia-150 plant. Covers all of
  §4.2's required command families (forward x3 speeds, backward x2,
  lateral left/right, turn CW/CCW, forward+turn both signs, diagonal
  both signs, accel-from-rest, decel-to-rest) by driving the teacher's
  own vx/vy/omega directly — no mirroring transform needed, the sim is
  left/right symmetric so negative vy/omega already produces the true
  trajectory. Records the §3.6 discriminator feature set per tick
  (joint pos relative to a per-clip neutral, joint vel, base angular
  velocity, projected gravity, foot positions relative to the body)
  as `obs_style` (60-dim), plus phase/command/raw-pose metadata per
  §4.5. Validates each clip against the SAME slip/m and fall
  vocabulary the eval harness already uses elsewhere (reject on fall,
  dragging, or joint discontinuity) — no new pass/fail definitions.
  Shipped `rl_move/sim/motion_library/teacher_v1.npz` +
  `_manifest.json`: 15/15 clips accepted, 88 s (spec target 20-60 s),
  slip/m 0.76-1.43 for translational clips (inside the teacher's own
  1.4-2.9 hardware band or better) and 2.72-3.07 for the two
  turn-in-place clips (a rotation-as-speed proxy metric, noted as a
  known rough edge in the script). CPU-only (no GPU pod used).
  ASSUMPTION recorded (`OPERATOR_QUESTIONS.md` q_20260822T0900Z):
  "neutral pose" is per-clip (that clip's own post-reset spawn stance),
  not one global constant — whoever builds the discriminator must
  confirm/override this to match the policy's own obs convention;
  the raw (non-relative) joint_position array is kept in the npz so
  this can be redone without re-running the sim.
- **DONE THIS CYCLE (08-22, discriminator core)**:
  `rl_move/sim/amp_discriminator.py` — `MotionLibrary` (loads
  `teacher_v1.npz`, samples real (s_t, s_t1) transitions that never
  cross a clip boundary, fits dataset mean/std for input
  normalization per §3.6), `AMPDiscriminator` (plain-torch MLP over
  the concatenated 2x60-dim normalized transition, matching
  `asym_policy.py`'s no-framework-beyond-torch style), least-squares
  GAN `style_reward` (bounded [0,1], no reward cliff for a
  not-yet-competent policy) and `discriminator_loss` (+ R1 gradient
  penalty on real transitions only — the exact mechanism the brief
  asks for "so the style reward does not saturate immediately").
  Bank-tested (`rl_move/tests/test_amp_discriminator.py`, 8/8 PASS,
  CPU, ~10s): after a short training loop the discriminator
  separates held-out real motion-library transitions from BOTH i.i.d.
  noise fakes and temporally-shuffled fakes (mean D(real) > D(fake) +
  0.5 margin in both cases), gradient penalty and loss stay finite
  throughout (no instant-saturation blowup), style reward correctly
  prefers real transitions. Standalone CLI smoke
  (`python3 -m rl_move.sim.amp_discriminator`) confirmed live: loss
  1.09->0.40 over 200 steps, D(real) 0.04->0.57, D(fake) 0.03->-0.63.
  **NOT YET WIRED into train_ppo_mjx's live reward loop** — that
  requires computing this SAME 60-dim `obs_style` feature vector from
  the batched Warp/MJX env each rollout tick.
  **PREREQUISITE CLOSED THIS CYCLE (08-22)**: an earlier note here
  guessed the GPU/Warp vec-env path "does not currently expose
  per-foot Cartesian positions or projected gravity as a reusable
  array" and proposed reconstructing them from the actor's own obs +
  forward kinematics — CHECKED DIRECTLY against `mjx_host.py` and this
  is not the actual gap: `mjx_host.FakeData` (the per-env numpy mirror
  EVERY shim env's `self.data` points at when `MjxVecEnv` drives it)
  already carries `xpos` for every body (chassis AND foot pads, via
  `push_output_row`'s `pad_xpos`) and `xmat` for the chassis — real
  Cartesian state, not reconstructed. The actual gap is narrower: it
  has NO `xquat` field, and `build_motion_library.py`'s extraction
  rotates world->body via `xquat` + `mju_rotVecQuat`, which would
  simply crash (`AttributeError`) the moment it touched a shim env.
  Fix: `rl_move/sim/amp_features.py`'s `obs_style_from_data` rotates
  via `xmat` instead (`R = xmat[chassis_bid].reshape(3,3); R.T @ v` —
  the SAME operation `walk_task.py`'s own `_body_vel_xy`/`_body_wz`
  already use for velocity) — proven mathematically IDENTICAL to the
  xquat method on a real rollout, not just plausible
  (`test_amp_features.test_xmat_matches_xquat_rotation`, CPU, max
  diff <1e-5 over 30 ticks). `build_motion_library.py`/`teacher_v1.npz`
  are untouched (no re-generation, zero behavior change to the shipped
  dataset) — `amp_features.py` is the one place both backends now
  share. Verified end-to-end on a live batched env (`hexapod-mjx-train-1`,
  jax/mjx installed there, not on the controller):
  `test_amp_features_mjx.py::test_mjx_vecenv_obs_style_batched` builds
  a real `MjxVecEnv(SimHexapodJointWalkEnv, B=3)`, steps it, and
  computes `obs_style` for all 3 envs (first time ever computed from
  the live trainer's actual physics backend, not the offline CPU
  generator) — shape/finite-checked. `test_discriminator_on_real_mjx_rollout`
  takes that live rollout's OWN transitions as the discriminator's
  "fake" input (replacing the synthetic noise/shuffle placeholders
  this module's docstring flagged) and confirms `discriminator_loss`
  is finite — the harder, more realistic version of M1 item 4's
  "gradients flow / no instant saturation" check. REMAINING gap,
  unchanged in size/scope: the live reward-loop change itself (blend
  `style_reward` into the task reward per step + an online
  discriminator-update step against the PPO rollout buffer) — a
  separate, larger, carefully-tested change (default-off,
  bit-exact-when-off), not started this cycle on purpose (the prereq
  above needed proving safe FIRST).
- **CONFIRMED NOT STARTED**: demo replay-buffer <-> PPO co-training
  loop, fault injection, push-disturbance curriculum, and the
  dedicated joystick eval suite (`eval/joystick_script.py` etc. per
  brief §15). Motion library v1 has no augmentation yet (mirroring/
  speed/phase scaling per §4.2) — the 15-family/1-seed base already
  clears the 20-60s target so augmentation is not currently a
  blocker, only a future diversity improvement.

## Next (brief §17 order — M0/M1)

1. **DONE 08-22** (`amp-m0-joycmd-asymcritic-smoke-v3`, W&B-disabled
   infra smoke, n_envs=4096, 500k steps, train-0): confirmed
   `--asym-critic` composes end-to-end with a FRESH (from-scratch,
   no `--init-from`) policy over a widened `stress_mix` cfg bundle
   matching AMP §6's envelope — `goal.walk_speed_min/max_m_s=0.0/0.60`,
   full-circle heading (default `walk_heading_max_rad=-1`),
   `goal.walk_yaw_cmd=1` + `walk_yaw_max_rad_s=1.0` +
   `walk_yaw_zero_frac=0.15`, `walk_cmd_resample_s=1.75` +
   `walk_cmd_resample_jitter=0.714` (spans exactly 0.5-3.0s),
   `walk_cmd_blend_s_min/max=0.05/1.0` (mixed abrupt/ramped
   transitions). Result: finite losses/values the whole run
   (value_loss ~29-32, explained_variance 0.64-0.76, std stable
   ~0.368, no NaN/crash), video reel ok, checkpoint verified on-pod
   loads as `AsymActorCriticPolicy` with the expected 73-dim actor
   obs space. TWO FAILED ATTEMPTS FIRST (v1/v2, both FAILED/logged):
   warm-starting from the joystick track's phase-clone checkpoint hit
   obs-width mismatches (the source checkpoint has neither the new
   yaw_cmd dim nor a compatible phase_obs width), and
   `--obs-pad-transplant` (the tool that would normally patch an
   obs-width change onto a warm start) is explicitly incompatible
   with `--asym-critic` — the fix was going from-scratch, which is
   the track's own charter default anyway, not a new tool. LESSON:
   AMP-track smokes must never `--init-from` a joystick-track
   checkpoint; obs-width plumbing only needs to agree with itself.
   The independent-vx/vy-vs-polar-speed+heading gap noted below is
   NOT a blocker — full-circle heading + speed magnitude covers
   reverse/lateral/diagonal commands functionally.
2. **DONE 08-22 (base library)**: motion-library generation from the
   teacher (all §4.2 command families) + validation (reject dragging/
   fall/joint-discontinuity clips) — see Now. STILL OPEN: mirroring +
   speed/phase augmentation (only needed if the discriminator turns
   out to want more than 88s / more per-clip diversity than the
   single-seed deterministic base gives it — cheap to add later,
   not gating discriminator work from starting).
3. **DONE 08-22 (core)**: `rl_move/sim/amp_discriminator.py` —
   discriminator, real-transition replay sampler, style reward,
   gradient penalty + input normalization, all consuming
   `teacher_v1.npz`'s `obs_style` field directly per §3.6/§4.5. Bank
   `test_amp_discriminator.py` 8/8 PASS (real-vs-noise AND
   real-vs-shuffled separation, finite gradient penalty, bounded
   style reward). **DONE 08-22 (wiring prerequisite)**:
   `rl_move/sim/amp_features.py` computes the SAME `obs_style` vector
   from the LIVE batched MJX/Warp vec env (`xmat`-based rotation —
   the shim's `FakeData` has no `xquat`; proven mathematically
   identical to `build_motion_library.py`'s method, see Now) —
   verified on a GPU pod against a real `MjxVecEnv` rollout
   (`test_amp_features_mjx.py`, both tests PASS).
   **DONE 08-22 (the live reward-loop wiring itself)**:
   `rl_move/sim/amp_style_vec.py` (`AMPStyleVecWrapper`) sits between
   the batched vec env and VecMonitor when `--amp-style-weight > 0`
   (default 0.0 = wrapper/callback/discriminator never constructed,
   bit-exact legacy — verified by an on-pod default-flags run with
   zero amp output). Per tick it pairs the env's new cfg-gated
   `info["amp_obs_style"]` emission (`goal.amp_style_obs`, default
   off, `sim_env._post_step`, raw joints — the LIBRARY's neutral is
   subtracted trainer-side so the convention lives in exactly one
   place, `MotionLibrary.neutral_pose`, which hard-fails if a future
   library's per-clip neutrals ever diverge) into episode-boundary-
   masked (s_t, s_t1) transitions, computes the bounded [0,1]
   least-squares style reward, and blends
   `r = task_w * r_env + style_w * r_style` into the ACTUAL PPO
   training signal (ep_rew_mean reflects the blend). A rollout-end
   callback co-trains the discriminator (real = motion library,
   fake = policy-transition ring replay, default 500k rows; R1
   gp=10 per brief §16) and logs `amp/*` to W&B; discriminator
   state round-trips via `<out>.amp_disc.pt` + `--amp-disc-init`
   for continuations. Flags: `--amp-style-weight/--amp-task-weight/
   --amp-motion-lib/--amp-disc-lr/--amp-disc-steps/--amp-disc-batch/
   --amp-gp-weight/--amp-replay/--amp-disc-init`. Banks:
   `test_amp_style_vec.py` 7/7 (env contract on/off, single-neutral
   enforcement, blend math, first-tick + done boundary masking, ring
   wrap, finite disc training with real>fake separation, save/load
   round-trip) — 20/20 across all four AMP banks ON-POD.
4. **DONE 08-22**: gradients flow through PPO with the discriminator
   co-training on POLICY rollouts — on-pod smoke
   `smoke_amp_style_wire_v1` (warp, 512 envs, 40k steps, task/style
   0.5/0.5): finite losses/kl/value across all 5 rollouts, 20 disc
   updates (4/rollout as configured), checkpoint + amp_disc.pt saved,
   no NaN/crash; plus a default-flags control run with zero amp
   codepath output (off = bit-exact).
5. Wave 1 across 8 pods: 3 seeds at task/style 0.5/0.5, no-AMP
   ablation, recurrent vs fixed-history, higher/lower AMP weight.
   Select on videos + tracking/stability metrics, never scalar return.
   **PILOT QUEUED 08-22 (M2 start, before committing 8 pods)**:
   matched pair `amp-m2-pilot-style05` (from-scratch, asym-critic,
   AMP-brief §6 stress_mix envelope = the v3 smoke bundle, task/style
   0.5/0.5) vs `amp-m2-pilot-noamp` (identical minus amp flags) —
   decides whether the style reward produces a recognizably cleaner
   six-leg tripod than task-only at equal budget, and whether the
   discriminator stays un-saturated at scale (watch amp/d_real vs
   amp/d_fake and amp/style_reward_mean).

- **M2 PILOT VERDICT (08-22, `cw-amp-m2-pilot-style05` vs
  `cw-amp-m2-pilot-noamp`, 2M discovery)**: mechanism PASS, behavior
  pre-gait. Discriminator co-training at 4096 envs stayed healthy the
  whole run (amp/d_real 0.78 vs d_fake -0.96 separated, never
  saturated; style_reward_mean ~0.08 — low, disc winning, but not
  pinned 0; gp finite; replay full; 124 updates). Both arms are
  equally pre-locomotion at 2M (~1.3 episodes/env): sprawled
  near-frozen stance, det fwd 0.007-0.026 m/15 s, slip ~9-12/m,
  sacrificed legs (style05 held legs 1&3 off in all det episodes),
  zero falls. No style-vs-control call possible this early; none of
  the pre-registered fail branches fired. CONTINUED matched as
  `-c1` runs (38M more each, policy + discriminator warm-started via
  `--amp-disc-init`) — wave-1 sizing decision moves to the 40M
  comparison.
- **M2 -c1 VERDICT (08-22 dig-in, both arms MISALIGNED per 08-21
  ruling)**: root-cause chain — behavior: half-tripod statue (triad
  0,2,4 planted duty ~1.0, triad 1,3,5 airborne duty ~0.02),
  gait_valid 0/12 both arms, new tilt_pitch/over_current terms from
  the ever-harder lean. Incentive: statue collects ~1.9/tick
  (rise_finish ~0.85 + posture/height kernels ~0.6 + K_WALK=2
  sigma=0.05 velocity kernel paying ~0.45/tick to v=0 across the
  stress_mix low/stop command fraction) vs realized locomotion income
  ~0.05/tick; the style channel (0.5 x style_reward_mean 0.06 =
  0.03/tick) is priced out. Pricing: nothing charges the freeze; the
  velocity needle gives no reachable from-scratch gradient. No sim
  defect. W&B trend is decisive: 100% of the 38M reward rise is
  statue income (rise_finish 0.09->0.86, task 0.09->0.59, walk_speed
  flat 0.029->0.035). Wave-1 NO-GO until a repriced pilot walks.
  Fix pair (launched, 2M discovery, from scratch per the 08-22
  init-basin rule — both -c1 checkpoints are cheat-committed):
  `cw-amp-m2-freeprog-{style05,noamp}` = pilot config with the
  SLIPWALK bank-calibrated pricing (k_walk_freeprog=3/cap 0.05,
  k_loadslip_excess=6, walk_gait_gate=1, k_walk_idle_charge=20,
  k_park_duty=2, k_step_event=1), pure-walk diet, and the
  pre-registered branch-(iii) envelope narrowing (speed 0-0.25 m/s,
  yaw +/-0.5). Key comparison: cw-nobc-slipwalk1-r1 (same pricing,
  no AMP) froze at 2M — style05 stepping where its noamp twin
  freezes IS the first real style-vs-control signal.
- **M2 FREEPROG FIX-PAIR VERDICT (08-22, both `cw-amp-m2-freeprog-
  {noamp,style05}` FAIL as run)**: neither predicted branch happened.
  Both arms show the SAME new failure mode — rapid catastrophic
  instability, not the predicted freeze (`cw-nobc-slipwalk1-r1`'s
  fingerprint) and not stepping. Gate eval (2M, DR-0, own cfg):
  noamp 8/12 episodes terminated tilt_pitch/tilt_roll within 1-2s of
  a stable plant spawn, fwd travel 0.008-0.071 m/15s in all 12
  (bar 0.10 m), slip 7.2-17.0/m; style05 11/12 terminated, fwd
  0.014-0.081 m/15s, slip 5.2-12.4/m — video near-identical topple
  fingerprint in both. Training reward FELL the whole 2M budget in
  both arms (noamp -82->-869/ep, style05 -33->-372/ep quarterly), so
  the 08-21 "reward rising" leniency does not apply — this is a
  genuine not-learning result at this budget/config, not a
  misalignment-with-rising-reward case. AMP mechanism itself stayed
  healthy in style05 (d_real 0.77 vs d_fake -0.94, unsaturated,
  style_reward_mean 0.093, 124 disc updates) — the style channel had
  nothing to rescue because both arms never survive long enough to
  produce a coherent gait to reward. ROOT-CAUSE LEAD (not yet
  confirmed, DIG-IN item): W&B shows `env/reward_walk_freeprog_pen`
  (the cross-track/backward charge) already near its harsh -6 floor
  (~-2.8/tick) from the FIRST logged training step in BOTH arms,
  before any learning — a raw from-scratch actor at std=0.367 flails
  incoherently and draws near-max penalty immediately; 2M steps is
  not enough to learn directionality before repeatedly toppling.
  style05's apparent partial reward recovery in its last quarter
  coincides with `ep_len_mean` SHRINKING (312->219 steps) over the
  same window — consistent with learning to die FASTER to cap
  per-tick penalty exposure, not behaving better; flagged as a
  possible pricing defect (termination underpriced vs. per-tick
  charges) for the semantics bank to check directly. Candidate fixes
  for the next arm (untested, do not launch blind): (a) forced
  log-std anneal at launch, mirroring the joystick track's
  phasedir8/9 repair, so exploration noise drops before the freeprog/
  loadslip charges start biting; (b) ramp `k_walk_freeprog`/
  `k_walk_idle_charge` in from 0 over the first few hundred-k steps
  instead of full dose from step 0; (c) a semantics-bank check
  (new scripted "topple-quickly" twin in `test_slipwalk_*`) for
  whether dying fast under-prices relative to surviving-and-flailing.
  Wave-1 sizing stays BLOCKED until a from-scratch config survives
  long enough on video for a style-vs-control comparison to mean
  anything.

## Required status block (update after each wave)

Current milestone / Best checkpoint / Code revision / Samples / Wall
time / FPS / Normal-gait, joystick, visual, push, fault, transfer
verdicts / Top 3 failures / Exact next experiments — per brief §18.
