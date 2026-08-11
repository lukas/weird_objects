# TURN — commanded yaw without the structural drift

Status date: 2026-08-11. Owner problem: RL_PLAN queue item 0 (unified
joystick policy — turning). Companion to rl_docs/RISE.md.

## The failure being solved

Walk-lineage policies carry a **command-invariant ~+0.09 rad/s left
drift**: they track yaw commands near the drift and fight commands
against it, in every scenario including turn-in-place. Three arms
failed on it and CLOSED price escalation as a move:

- `cw-walk-yawcmd1` — kernel alone: with σ=0.15 the ungated kernel
  pays a command-ignoring policy 0.67 of max income every tick; both
  seeds learned exactly the drift.
- `cw-walk-yawgate1` — achieved-rotation income gate: drift persisted.
- `cw-walk-yawgate2` — k_walk_yaw 1.0→2.5: drift persisted.

Root cause reading: the Gaussian kernel is the wrong SHAPE, not the
wrong price. Near wz_ref=0 its gradient at the drift point is tiny
(0.09 ≪ σ=0.15), and on turn segments it never goes **negative** for
wrong-direction rotation — rotating against the command and parking
cost the same. Meanwhile turn-in-place states are ~7.5% of training
segments (yaw is drawn independently; turning only coincides with a
stop via resample), so the skill being scored is barely trained.

## The mechanism set (landed 08-10, all cfg-gated, default 0 = legacy)

1. **Signed rotation income — `reward.k_yaw_prog`.** The k_walk_prog
   analog for turn segments: `k * clip(wz/wz_ref, −1.5, +1.25)`.
   Constant gradient toward the commanded direction; genuinely
   NEGATIVE when rotating against it. This is a new mechanism (sign-
   aware income), not a re-price of the closed kernel family.
2. **Drift charge on heading-hold — `reward.k_yaw_still`.** When
   wz_ref = 0: `−k * wz²`. At the measured drift (0.09 rad/s), k=50
   costs ~0.4/tick — real money against the ~2/tick kernel — while
   gyro-noise wz stays ~free by the square law. This is the term that
   directly disincentivizes DRIFTING while walking straight.
3. **Turn-in-place curriculum — `goal.walk_turn_in_place_frac`.**
   With probability f a walk episode becomes a dedicated turn: zero
   linear command, |wz| drawn in [0.5, 1.0]·walk_yaw_max_rad_s, sign
   50/50 by construction (the drift direction can never dominate
   exposure). Applied last in `_sample_walk`, overrides resample
   segments; rng stream untouched at f=0.

Held in reserve (not landed): mirror-symmetry augmentation /
symmetry loss (reflect obs+actions about the sagittal plane). The
heavy structural fix if the mechanism set above fails in DISCOVERY —
needs trainer surgery, do not start there.

## MDP_PREFLIGHT — the TURN bank (PASSING 08-10)

`test_task_semantics.py::test_turn_reward_separates_command_from_drift`
runs the full turn stack (champion walk cfg + walk_yaw_cmd, kernel
k=1.0 + achieved-rotation gate + k_yaw_prog=1.0 + k_yaw_still=50) on
scripted-gait policies at wz_ref = ±0.25, 3 seeds × both signs:

    turn (full command)   +1463
    partial (35%)         +1222
    drift (fixed +0.09)   +1154
    park                  +1122

Ordering `turn > partial > drift > park` holds — the stack now prices
the exact policy PPO found three times below honest partial turning.
`test_turn_command_signs_priced_symmetrically` additionally requires
CW and CCW turn income within 45% of each other — a live tripwire on
the yaw sign chain (below).

## Sign audit (still OPEN — do this before any hardware turn)

Sim `_body_wz()` is **+CCW** (right-hand z-up). Hardware measured
08-09: scripted gait `+omega = clockwise`. The TURN bank proves the
SIM chain (gait omega ↔ wz_ref ↔ reward) is internally consistent,
so the flip sits at the hardware boundary: the deploy bridge must
map joystick/policy yaw commands with the sign audit's result, or
the first hardware turn will fight its own command. One bench check:
command a small +wz through the bridge, read gyro sign.

## First arm result — FAILED (08-10, `cw-walk-turnfix1`)

Trained exactly the recommended cfg below off `cw-walk-yawgate2`.
Matched-parent control (`eval_yaw.py`, identical scripted panel,
turnfix1 vs frozen yawgate2): turn |wz_err| med 0.232 vs parent's
0.233; hold |wz| med 0.108 vs parent's 0.091 — statistically
IDENTICAL to the already-failed parent, same left/right asymmetry
(arc-left ~0.07–0.21 near the drift, arc-right ~0.22–0.37 fighting
it). The reward-side mechanism set (signed rotation income +
heading-hold drift charge + turn-in-place curriculum) passed its
pre-training bank but produced ZERO measurable behavior change in a
real policy. **Behavioral-impossibility kill — price tuning on this
task is now doubly closed** (first the kernel-price family, now the
signed-income/drift-charge/curriculum family). Straight walk stayed
clean (gv 6/6, 0 falls). Do not re-attempt with more steps or a
different k; the next move is the structural fix below.

## Next move: mirror-symmetry augmentation (was "held in reserve")

Reflect obs+actions about the sagittal plane (symmetry loss or data
augmentation) — needs trainer surgery, [CODE] not a launchable spec
yet. This is now the ONLY untried lever on the turning blocker;
every reward-shape lever (kernel price, achieved-rotation gate,
signed income, drift charge, turn-in-place exposure) has failed to
move a real trained policy off the fixed left-drift. Root-cause
reading: the drift is baked into the WALK GAIT itself (an asymmetric
limb-phase pattern learned once, early, off-center), not into the
turn reward's shape or price — no reward retuning can out-argue a
structural asymmetry in the policy's default gait.

## Mirror-symmetry landed; hardening run hit a reward bug, not a verdict (08-11)

`train.mirror_loss_coef=1.0` landed 08-10 (`rl_move/sim/mirror.py` +
`MirrorPPO`), discovery probe `cw-omni-mirror1` PASSED its
mechanism-health gate (mirror_sym_loss fell to <0.5x peak, reward
climbed cleanly, 0 NaN). The 40M-step hardening follow-up
`cw-omni-mirror1-r1` does **NOT** confirm or refute the mirror
hypothesis: the walk gait itself collapsed into a stand-still/
march-in-place exploit before turn-tracking could be judged.
Harness evidence (own-DR + DR0, vs frozen `cw-arch-hist16-dep1`
same-recipe baseline): forward travel 0.68m med -> 0.01m med per 15s
episode, gait_valid 6/6 -> 3/6, slip_per_m 1.48 -> 3.85 med. Per-
episode returns show WHY: frozen episodes (gait_valid False, ~0.004m
travel) scored ~1130, walking episodes (gait_valid True, ~0.02m
travel) scored 500-860 — **standing still paid more than walking**
under this arm's stack. `train/std` also climbed monotonically
0.39->1.69 over the full 40M (health alarm, RESEARCH_RULES), in step
with `rollout/ep_rew_mean` peaking ~640 near 8-10M then falling to
~320-350 by 40M.

Actual cause (probe-confirmed 08-11 — the earlier k_yaw_still guess
was WRONG; the drift charge summed to ~0 for a scripted gait): on
turn-in-place ticks (`s_ref≈0, wz_ref≠0`) the LINEAR velocity kernel
paid a frozen robot FULL income — v_lin=0 matches the zero linear ref
exactly, and `walk_kernel_prog_gate` only engages when `s_ref>1e-3`.
The same s_ref condition left `k_park_duty`/`k_step_event` inert on
those ticks. With `walk_turn_in_place_frac=0.30`, a freeze banked
~1122/ep (probe; run showed ~1130) — 0.77x of a PERFECT scripted
turner's income and MORE than the mid-training policy earned by
actually walking (500-860). PPO parked, by construction.

**Fixed (08-11):** `reward.walk_kernel_yaw_gate` (walk_task.py,
default 0 = legacy): on yaw-commanded zero-linear ticks the linear
kernel is multiplied by achieved-yaw fraction clip(wz/wz_ref, 0, 1)
— the exact prog-gate analog; genuine stop segments stay paid. The
freeze-floor bank (bottom of test_task_semantics.py) pins the
exploit two ways: park < 0.5x turn on pinned turn-in-place commands,
and gait-follows-commands beats a full-episode freeze on SAMPLED
r1-mixture episodes (resample/stops/turn-in-place). Pre-fix both
FAILED (park/turn 0.77); post-fix: turn 1128 > partial 650 > drift
539 > park 423 (0.38x), mixture gait +1055 vs freeze +424 (0.40x).
OMNI_OVERRIDES now trains the gate ON (1.0) — any omni arm must.
Re-hardening (`cw-omni-mirror2`, warm from the healthy 2M probe
ckpt, one variable vs r1 = the fixed pricing) is the next launch.
Do not read the r1 FAIL as evidence against mirror-symmetry — the
mechanism was never exercised by a real gait in that run.

## `cw-omni-mirror2` — gate fix confirmed, but a NEW gait pathology
blocks the mirror-symmetry test (08-11)

40M hardening, one variable vs r1 (`walk_kernel_yaw_gate=1.0`),
finished. The specific r1 exploit (frozen episode out-earning a
walking one) is GONE — per-episode returns now show walking beats
the degenerate pattern (526–922 vs 399–425) — but the gait itself
still breaks down in ~half of det+sto episodes into a leg-sacrifice/
tripod pattern (one or more legs held near-stationary, duty_cycle
~0.04–0.07 or 1.0-with-0-swings, forward_dist_m 0.005–0.05m/15s,
0/6 success both modes; contact sheets confirm the body barely
translates). `train/std` climbed 0.39→1.30 (>2x, the pre-registered
health alarm) and reward peaked ~447 near 10M then fell to 302 by
40M. **STOP — known exploit (video overrides the mechanism-health
metric); no dig-in.** Mirror-symmetry remains statistically UNTESTED
— the gait never got clean enough to isolate a turn-tracking signal
from a chirality signal. Per the pre-registered outcomes, this is
closest to "collapses again despite the gate": do not launch another
mirror hardening arm; the next move is a term-by-term income
re-probe of the WALK kernel (not just the yaw-gated segments) to
find what still pays for a partial leg-sacrifice, before any new
mirror arm.

**`cw-omni-mirror2-dr02` — DR exonerated (08-11).** Matched twin,
identical spec, dr-scale 0.2 vs mirror2's 0.5 (operator question: is
DR making this task too hard?). Fails IDENTICALLY: det gait_valid
3/6, same leg-sacrifice pattern (legs [0,2,4] / [1,3] held, fwd
0.00–0.01m), walking still out-earns sacrifice (646–889 vs 473–485,
same shape as mirror2's 526–922 vs 399–425); sto gait_valid 6/6 but
slip_per_m 4–18, 0/6 success both modes both configs. `train/std`
1.10 at 40M (milder than mirror2's 1.30 but same terminal pathology).
DR-scale is ruled out as the driver — this is a property of the
reward/task stack, not an optimization-difficulty knob. Do not queue
another DR level on this line.

## Recommended first arm (DISCOVERY, ≤2M steps) — SUPERSEDED, see above

Parent: walk champion (or hist16 twin). Cfg:
`goal.walk_yaw_cmd=1 goal.walk_turn_in_place_frac=0.30
reward.k_walk_yaw=1.0 reward.walk_yaw_kernel_gate=1.0
reward.k_yaw_prog=1.0 reward.k_yaw_still=50` +
champion walk stack. Evidence: the passing TURN bank. Judge with
`eval_yaw.py` (turn |wz_err| median vs the 0.10 gate; hold |wz|
median vs 0.05) AND a matched-parent control — the parent under the
identical eval, so the drift delta is attributable. Early video at
first eval; kill on the behavioral-impossibility rule if both turn
directions still converge to the drift.

## `cw-omni-trans1` — turning removed entirely, gait still collapses
(08-11)

Operator de-scoped commanded turning from the joystick deliverable
(no camera on the robot = no reason it needs a definable "front").
`cw-omni-trans1` tests the narrower goal directly: walk in ANY
commanded direction (full-circle heading), mirror-symmetry loss
still on (coef 1.0), full dep1 contract, k_current=0 — but with the
ENTIRE yaw/turn-in-place stack removed (no `walk_yaw_cmd`, no
`walk_kernel_yaw_gate`, no turn-in-place curriculum), so the
freeze-income exploit that required a yaw gate is structurally
absent. Result: FAIL, a THIRD distinct pathology. Not freezing
(mirror1) and not the leg-sacrifice/tripod pattern (mirror2/dr02):
instead a paddle-stall — legs 1 and 4 stay planted 90–99% duty the
whole episode while the other four take rapid, tiny (~0.01m mean)
strides, slip_per_m 3–13 (vs champion band ~1.2–1.5), along-command
progress_ratio med 0.51 det / 0.22 sto, 0/6 success any mode/DR.
`train/std` climbed continuously with no plateau, 0.37→1.38 (3.7x
start) — worse than mirror2's own 2x alarm and never recovered.
Reading: omnidirectional translation (independent of the yaw
mechanism entirely) is ALSO not yet a solved reward/task spec — three
different arms (mirror1, mirror2/dr02, trans1) each find a different
degenerate attractor once heading leaves the narrow forward band the
champion was built on. Mirror-symmetry remains completely untested
by any of the three; the gait has never been clean enough to isolate
it. Do not launch a fourth omni variant on the current stack — next
move (unchanged from the mirror2 ruling) is a term-by-term WALK-kernel
income re-probe: find what still pays for degenerate partial gaits
(freeze, sacrifice, AND paddle) before trying another mirror/heading
arm.

## Income re-probe DONE (08-11, `probe_walk_income.py`) — pricing
EXONERATED on the deliverable stack; latent yaw-kernel defect found

`rl_move/sim/probe_walk_income.py` decomposes per-term income
(every `info["reward_*"]`, residual 0 by construction) for scripted
references matched to the three video fingerprints AND the actual
collapsed checkpoints, under the exact trans1/mirror2 stacks
(artifacts: `logs/probe_walk_income/`). Results, mean/ep over
4 directions x 3 seeds:

- **trans1 stack (turn-free, the deliverable): NOTHING pays the
  degenerates.** DR0: gait 824 > half-speed gait 643 > 1-leg
  sacrifice 541 > paddle 375 ≈ tripod-sacrifice 341 > freeze 217 >
  **the trained trans1 checkpoint itself 205** — the collapsed policy
  earns BELOW A FREEZE under its own reward. Identical ordering at
  the training DR 0.5 (761/564/515/337/289/233/214). Directions
  priced uniformly (gait 750-897 everywhere). Income is monotone in
  honest progress; the degenerate attractors are OPTIMIZATION
  failures, not paid basins. Reward surgery on this stack is CLOSED
  (matches trans1's pre-registered if-false: "not reward surgery").
- **mirror2/turn stack: real latent defect.** On linear-command ticks
  the ungated yaw kernel (`k_walk_yaw`, wz_ref=0 "heading hold" side)
  pays a MOTIONLESS body full income — 373-375/ep to sacrifice,
  paddle, and freeze alike, the single largest channel in the stack —
  while `k_yaw_still=50` charges the honest gait's natural wz
  oscillation -73/ep and the degenerates ~0. Net: the yaw stack taxes
  honest walking ~-100/ep RELATIVE to body-stillness. Aggregate
  ordering still holds (gait 1100 > paddle 748 > sac3 710 > freeze
  592 > ckpt 410), but this stillness subsidy must be fixed (gate
  heading-hold yaw income on linear progress, or price wz vs the
  gait's own oscillation band) BEFORE any turn re-scope. Not fixed
  now — turn is de-scoped from the deliverable.

**Next lever (supersedes "rot-60 first"):** the collapse signature —
provably unpaid behavior that PPO still converges to, because no
gradient tells a churning leg WHICH WAY to move — is exactly what the
BC anchor fixed twice (rise `cw-stand-bc1`, hold `cw-stand-holdbc1`).
Landed 08-11: walk ticks emit `bc_target` = the command-conditioned
scripted TripodGait pose (the gait that walks/crabs/turns the REAL
robot) one tick ahead; stop ticks unsupervised (the gait marches in
place at v=0); per-episode gait instance on SNAP_ATTRS. Discovery arm
`cw-omni-transbc1` (trans1 + `train.bc_anchor_coef=1.0`, one
variable). rot-60 equivariance stays the reserve lever if imitation
anchoring fails; note mirror-symmetry loss coef 1.0 was ON during the
trans1 collapse and did not prevent it.

## OMNI TRANSLATION RESOLVED IN SIM — rot-60 canonicalization
(08-11, SPECIFICATION result, zero training)

The reserve lever after transbc1 closed BC-anchor/reward tuning. The
robot is a REGULAR hexagon: six identical leg templates at exactly
(i+0.5)*60 deg, axisymmetric chassis inertia, identical actuators —
rotate the world 60 deg + relabel legs is an EXACT symmetry of the
compiled model (unlike the mirror, which the COXA_HIP_ANCHOR_Y
pinwheel only approximates; the pinwheel is rot-60 INVARIANT).
`rl_move/sim/rot60.py` exploits it at eval time: pick the 60-deg
sector nearest the commanded heading, rotate the command/tilt/gyro/
velocity obs into the +/-30 deg wedge, cyclically relabel the leg
channels, un-relabel the action. A wedge-trained policy then covers
the full circle BY CONSTRUCTION. `test_rot60.py` proves the model
symmetry mechanically (rotate+relabel state, permute ctrl, step: <1e-6
divergence over 30 contact steps — a real asymmetry would show ~1e-3
immediately).

Evidence (`logs/rot60/`, eval_drive full-circle panel + harness
per-mode 6 det+sto at full-circle headings, matched naked controls,
seed 0):

- `cw-arch-hist16-dep1` (trans1's parent): naked back
  trk_err 0.069 / 0.102 m of the commanded 0.30 m; laterals
  0.047-0.052. Wrapped: back 0.039/0.305 m, left 0.030, right 0.028,
  ZERO falls at DR 0 and DR 0.5 incl. full-circle instant-flip stress
  (live sector switching). Harness on full-circle commands: naked
  degenerates AT EVAL TIME into the trans1-style leg-sacrifice
  (gait_valid 3-5/6, sacrificed legs [2]/[0,4], prog_ratio 0.41-0.60,
  slip/m 7.3-11.3); wrapped is the honest champion gait everywhere
  (gait_valid 24/24, prog_ratio 0.92-0.98, slip/m 1.3-1.6).
- **`cw-dep-vref1-r1` (THE hardware checkpoint)**: naked backward is
  frozen (0.027 m); wrapped, every direction tracks 0.024-0.036 at
  DR 0 AND own DR 0.35, zero falls; harness success 20/24
  (det/DR0 6/6), gait_valid 23/24, slip/m 1.1-1.3 (its own band),
  video-confirmed six-leg gait. The hardware deliverable gains
  full-circle translation with NO new training.
- Known quirk: dep1 wrapped diag-fr (canonical +15 deg) reads
  0.046-0.051 at both DRs — a dep1 wedge asymmetry (vref1-r1 shows
  0.028-0.032 there); dep1's vel-success was already "coin-flip per
  episode" pre-wrapper (its run doc). Not a wrapper defect.

Interpretation: the four omni collapses were PPO failing to DISCOVER
rotated gaits (matches the income probe: attractors unpaid), and the
fix is structural, not learned. Also supersedes the trans1-stack
training question — there is nothing left for a 40M omni arm to
learn that the wrapper does not already give exactly.

De-scoped turn note: with no camera there is no "front" — the wrapper
makes heading-agnostic driving native (the operator points the stick;
the body never needs to yaw). If turn ever re-scopes, rot-60 also
gives 6 free discrete body orientations by pure relabeling.

NEXT (the only remaining work on this blocker, [CODE], deploy-side):
port the ~60-line numpy canonicalizer into the robot runner's obs/
action path (same contract: reads vx/vy_ref from the obs frame,
per-episode sector state w/ hysteresis + zero-cmd hold) + a
replay-parity check against rot60.py. Until then eval-side omni is
covered by `eval_drive --rot60` / `eval_checkpoint --rot60`.
