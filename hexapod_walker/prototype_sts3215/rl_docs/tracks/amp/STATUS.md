# amp - AMP locomotion from scratch

Last updated: 2026-08-22 ~19:0x (MECHANICAL NOTE on the entry below:
this cycle found the section-5 bank + STATUS text already drafted in
the shared working tree by a concurrent cycle, but ledger/backlog had
ZERO matching entries and all 12 GPUs were idle — the actual
`launch_run.py launch` calls for `cw-amp-m2-sec5-{taskA,taskB,taskC}`
had not yet executed. Snapshotted the shared tree (commit `cc53f4b1`,
includes the AMP_MINIMAL_OVERRIDES bank + this doc's own draft) and
ran the 3 launches exactly as pre-registered below, PLUS a 4th arm
this cycle added for a real (not just scripted-twin) task-only
baseline: `cw-amp-m2-sec5-noamp` (identical minimal reward, zero AMP
flags, train-9) — isolates what the section-5 task reward alone buys
before crediting style for any improvement. All 4 verified RUNNING
(taskA/B/C on train-0/4/8, noamp on train-9); taskA/B/C are 2M-step
arms and finish in ~2-3 min at this fps, so by the time this is read
they may already be FINISHED and awaiting the watcher's prestage —
ledger status was hand-corrected INTENT->RUNNING immediately after
each launch (the multi-launch batch exceeded this cycle's tool-call
time budget mid-verification) so the watcher's finish-detection scans
them correctly instead of stranding them the way the 08-22 16:18
stotight grid got stranded. Two harmless REFUSED artifacts from this
cycle's own independent (pre-discovery) attempt at the identical idea
under the name `cw-amp-m2-min5-noamp` are in the ledger — superseded
by `cw-amp-m2-sec5-noamp` above, no action needed on them.

Previous entry (~18:5x, MEASUREMENT ARMS CLOSED, SECTION-5
REWARD BUILT+BANK-TESTED, A/B/C GRID LAUNCHED: `cw-amp-m2-styleonly-
v2-c1b` (the +10M pure-style continuation) VERDICTED FAIL —
`amp/style_reward_mean` plateaued (quarter means 0.10/0.20/0.16/0.15,
never sustained above the 0.3 pass bar) but DR-0 gate gait_valid
jumped to 5/6 det+sto (was 0/6 for the parent stork statue) and video
shows genuine multi-leg cyclic stepping, not a frozen pose — a
march-in-place/leg-twitch degenerate (fwd 0.02-0.03m/15s, slip
16-21/m): style organizes coordinated leg motion but has ZERO
incentive to convert it into displacement (an AMP-standard
limitation — the 60-dim single-tick feature set is body-frame-
relative, so a translating and a non-translating gait look identical
to the discriminator absent slip). Paired with taskdown01-style1-v3's
FAIL, this CLOSES the pure-style-alone measurement question: task
reward must supply the "go somewhere" signal, style supplies "look
natural" — neither alone suffices, exactly per brief section 5, not
a discriminator defect. BUILT + BANK-TESTED the section-5.1 literal
minimal task reward (`AMP_MINIMAL_OVERRIDES`,
`rl_move/tests/test_task_semantics.py`, 4/4 new tests; full bank
163 pass / 4 skip / 1 xfail / 1 known-red — the sole red is the
pre-existing unrelated `fastprof` item, untouched): needed ZERO
new production code — it is exactly the plain
Gaussian velocity kernel + linear progress (`walk_task.py`'s
`K_WALK`/`SIGMA_V`/`k_walk_prog`) that already fires whenever the
SLIPWALK anti-slip stack (freeprog/loadslip/idle-charge/gait-gate/
step-event) is left at its off-by-default state, plus
`reward.term_penalty=400` (the real anti-suicide pricing fix, kept —
not SLIPWALK-specific). Measured 3-seed scripted-twin bank: real
travel beats every stationary twin by a modest ~230-250/ep margin
(weaker than SLIPWALK's engineered 300+, as expected with no
anti-park apparatus) and stall/park/stork/skate are all bunched
within ~15 points of each other (near-zero "trying beats refusing"
gradient, pinned as a measurement not patched — style is relied on
to supply that gradient, which styleonly-v2/-c1b just proved it can).
LAUNCHED (this cycle) the brief's own pre-registered 3-arm task/style
sweep on this reward — `cw-amp-m2-sec5-{taskA,taskB,taskC}`
(task/style 0.7/0.3, 0.5/0.5, 0.3/0.7), from-scratch, teacher_v2 lib,
2M discovery, same stress_mix command envelope as the retired
freeprog family for comparability. Prediction-if-true (any arm):
gait_valid climbs on real six-leg motion AND fwd travel clears
~0.10m/15s with slip in a sane band — the section-5 reward is the M2
fix. Prediction-if-false: same statue/march basin — the from-scratch
exploration problem is deeper than reward shape (task restructuring
or BC-pretrain phase becomes the next real lever, not further reward
tuning). Previous update ~18:2x retained below (TASK/STYLE DOSE
LADDER CLOSED on the
SLIPWALK-derived reward architecture: `cw-amp-m2-taskdown01-style1-v3`
(task_weight 0.1, the joint twin to styleonly-v2) VERDICTED FAIL —
reward DECLINING every quarter (-2.4/-30.4/-69.2/-85.2, opposite of
styleonly-v2's rise) and `amp/style_reward_mean` 0.087, actually
LOWER than styleonly-v2's pure-style 0.119 — even 10% of the SLIPWALK
task charges re-buries the fragile style gradient. DR-0 gate ticked
up slightly (det gait_valid 1/6, sto 3/6, vs styleonly's 0/6 both)
but video/contact-sheet is still the same near-static splayed statue,
legs [1,3,5] sacrificed. Every task/style ratio on this reward
architecture (0.0/0.1/0.5/1.0/2.0, spanning noamp through pure-style)
has now failed — this CLOSES DOSE-TUNING as a lever on the SLIPWALK-
derived stack entirely; the diagnosis in q_20260822T1815Z (the
reward ARCHITECTURE — not its task/style ratio — is the wrong shape
for a from-scratch AMP actor) is now measured, not just textual. NEXT
REAL ARM (not yet launched, needs design care per the DIG-IN note
below): a clean-slate implementation of `AMP_LOCOMOTION.md` section
5.1-5.4's own minimal Gaussian-velocity/yaw/upright task reward + a
dominant-enough style weight, dropping the SLIPWALK freeprog/
loadslip/drag-stance/idle-charge/anchor/gait-gate apparatus wholesale
— no more task/style ratio arms on the current stack until that
redesign is tried.)

Previous update ~18:2x (STYLEONLY-V2 VERDICTED FAIL, not a
continuation — the podeval videos/gate the previous entry was
waiting on are in: DR-0 gate + fresh eval contact sheet both show
the SAME frozen-tripod statue signature as every prior M2 arm
(gait_valid 0/6 det AND sto, legs [1,3,5] — the OTHER tripod from
the swing/RSI family's [0,2,4] — sacrificed/frozen, speed
~0.006-0.025 m/s, prog_ratio ~0.00); training video (2 late rollouts)
shows the identical splayed near-static pose the whole clip, no
cyclic motion. Per the video-overrides-scalar rule this is FAIL, not
UNDERTRAINED-continue, despite the monotonically rising reward: the
rising income bought a MARGINALLY more discriminator-plausible
static/near-static pose (0.06->0.119 style_reward_mean, still <0.3
informative bar), not progress toward real six-leg cycling — the
same shape as every other M2 statue, just a different tripod frozen.
Genuinely informative, though: the discriminator gradient is
confirmed alive/unsaturated (d_real 0.79/d_fake -0.96) even at zero
task competition, ruling out total AMP-mechanism failure — the style
channel is just too weak/slow at this budget to organize six-leg
coordination on its own. Retried its REFUSED twin (code-sync race,
0 steps trained) as `cw-amp-m2-taskdown01-style1-v3` (task_weight
0.0->0.1), now RUNNING on train-0 to complete the pre-registered
joint read before deciding between GP/disc-tuning, much longer
style-only budget, or the section-5 minimal-reward rearchitecture
(q_20260822T1815Z).

Previous update ~18:1x (RSI-FOR-WALK LEVER CLOSED: both
`cw-amp-m2-freeprog-term400-rsi1-{noamp,style05b}` FAIL —
`goal.walk_gait_start_frac=0.5` mid-gait spawn does not unlock
sustained locomotion on the current freeprog+term400 pricing either
[confirms `cw-gait-rsi1`'s 08-11 finding on the new stack]. Det fwd
travel nominally rose to 0.046-0.058m [vs the ~0.03m statue ceiling]
but `gait_valid` REGRESSED to 0/6 [worse than every reward-side arm,
3-5/6] with 2-3 legs consistently near-frozen — reads as an RSI
RESET ARTIFACT [coast on the seeded head start, then collapse into a
partial-leg-drag], not real sustained six-leg walking. Style added
nothing on top of RSI either. ep_rew_mean fell every quarter both
arms — genuine flat-reward FAILs. This CLOSES EVERY ACCESSIBLE-
GRADIENT LEVER tried for the M2 freeprog statue basin so far
[term-penalty, std-anneal, staging, task-complexity, style-dose,
swing, RSI — reward-side AND initial-state] — the basin is an INCOME
problem, not a discovery-only one. NEXT (untested): task
restructuring [shorter/denser episodes, or a whole-episode net-
displacement score instead of per-tick cross-track charges] or a
short BC-pretrain phase on the motion library before RL, still
consistent with the "demo is training data" charter — not another
coefficient/gradient-source/reset-state tweak. FLAGGED CANDIDATE
ROOT CAUSE (not yet tested, DIG-IN item, see CURRENT_TRUTHS for full
detail): every M2 freeprog arm reused the JOYSTICK track's harsh
SLIPWALK anti-slip pricing stack instead of `AMP_LOCOMOTION.md`
section 5's own specified reward (simple Gaussian velocity/yaw/
upright/weak-height task reward, modest regularizers, "do not make
stance slip the dominant reward", task/style 70/30-30/70) —
`env/reward_walk_freeprog_pen` sits flat at -1.4 to -1.7/tick,
~15-30x the realized style income, in every arm. Section 2 of the
brief describes this EXACT failure signature (paddle-creep/
dragging/reward-exploits from an under-specified task reward) and
names the fix as simple-task-reward + dominant-style, which M2 has
never actually implemented. A clean-slate brief-literal reward arm
is the next real experiment, but needs careful design (which modest
regularizers prevent reopening the OLDER pre-freeprog freeze cheat)
before training. See CURRENT_TRUTHS for full detail; earlier 08-22
history retained below this line.

MEASUREMENT ARMS FOR THAT REDESIGN, LAUNCHED ~18:0x (same cycle
that verdicted rsi1-noamp): before writing the section-5 reward,
directly measure whether the style gradient is learnable AT ALL and
where the task/style burial threshold sits — `cw-amp-m2-styleonly-v2`
(task 0.0 / style 1.0, teacher_v2, pure imitation, no task income or
charges; canonical AMP pretraining) and `cw-amp-m2-taskdown01-
style1-v3` (task 0.1 / style 1.0 — first arm ever with style
capacity ~6x the task charge floor; v3 = launcher self-repair retry
of a code-sync REFUSED v2, identical config). FIRST READING,
styleonly-v2 (FINISHED, 2M): amp/style_reward_mean climbed 0.06 ->
0.119 — 2x the 0.05-0.07 band every buried arm was pinned at, still
below the 0.3 INFORMATIVE bar, but ep_rew (pure style income) rose
MONOTONICALLY all four quarters (14.2/21.9/30.3/37.8) with no
plateau and the discriminator stayed unsaturated (d_real 0.79 /
d_fake -0.96) — the style gradient IS alive without task
competition. VERDICTED INFORMATIVE ~18:1x after harness+video:
the behavior earning that income is a half-tripod STORK STATUE
(legs 1,3,5 sacrificed in all 12 episodes, one front leg raised
waving, fwd 0.02m/15s, prog ~0) — static pose resemblance, not
cyclic motion. Since the gate metric here IS the reward and it was
still rising at budget end (~1.3 ep/env), CONTINUED +10M as
`cw-amp-m2-styleonly-v2-c1b` (policy + disc warm-started,
acquisition): style >=0.3 + cyclic multi-leg motion => M2 fix is
income rebalancing (feeds q_20260822T1815Z); plateau <=0.2 with the
stork persisting => the 60-dim single-step transition feature set
rewards static poses too well — dig-in goes to discriminator
feature/temporal-horizon design, not pricing.)

Previous update, 2026-08-22 ~18:0x (K_WALK_SWING LEVER CLOSED: both
`cw-amp-m2-freeprog-term400-swing-{noamp,style05}` FAIL, landing in
the SAME ~0.03m/15s statue-family ceiling as every prior arm
[noamp/style05-v2/stylew2-v2/fixedcmd] — the swing-completion bonus
was bank-verified (11/11) to rank real gait > shuffle > stall but at
dose 1.0 its realized income stayed pinned ~0.05-0.06/tick, an order
of magnitude below freeprog_pen's flat -1.4 to -1.5/tick, so it never
moved a from-scratch PPO run out of the pre-existing basin — same
failure shape as the style-dose lever. Video: NOT a clean tripod —
duty cycle heavily imbalanced across legs (one leg duty~0.99, another
~0.02-0.19) despite every leg registering nonzero swing_count, close
to the bank's pre-registered "shuffle" cheat shape (farm swing events
without organizing locomotion). ep_rew_mean fell every quarter in
both arms (genuine flat/declining FAIL, not continue-longer). Style
added nothing on top of swing either (noamp/style05 stats
indistinguishable). This CLOSES THE WHOLE REWARD-SIDE LADDER for the
M2 freeprog statue basin: term-penalty, std-anneal, staging, task-
complexity/fixedcmd, style-dose (0.5x-2x), and now swing have all
failed the identical way — a real, bank-verified income channel that
is simply too small to out-bid the basin's incumbent local optimum
at 2M/from-scratch budget. FOLLOW-UP (launched same cycle, first
non-reward lever): `cw-amp-m2-freeprog-term400-rsi1-{noamp,style05b}`
switches from pricing to the INITIAL STATE — `goal.walk_gait_start_
frac=0.5` spawns half of episodes mid-stride in the scripted teacher's
own tall walking pose (built-in RSI-for-walk mechanism, `cw-gait-
rsi1`'s own lever) so PPO only has to SUSTAIN a gait it starts inside
of, not discover one from a static crouch. This exact lever was
tried once before (`cw-gait-rsi1`, 08-11) but on a stack that
predates freeprog/term400 pricing entirely and was refuted with the
same freeze signature — untested on the current pricing. If RSI also
fails, the basin reads as INCOME-not-DISCOVERY after all and every
accessible-income-channel idea (dose or gradient-source) is closed
for this family; a genuinely different mechanism (task restructuring
or a short BC-pretrain phase before RL, still consistent with the
track's "demo is training data" charter) becomes the next thing to
try, not another reward coefficient.)

Previous update, 2026-08-22 (STYLE-DOSE LEVER CLOSED: both clean-lib
style arms `-style05-v2` (0.5/0.5) and `-stylew2-v2` (2.0/1.0) FAIL,
statistically indistinguishable from the -noamp control and from
each other — the AMP-style-income route is refuted at 2M discovery
scale across the whole tested dose range for the freeprog/term400
pricing family. Follow-up `cw-amp-m2-freeprog-term400-fixedcmd{,
-seed11}` launched same cycle to isolate task-command complexity
from reward shape using the SLIPWALK bank's own literal fixed
command; see M2 bullet below. Earlier 08-22 history retained below
this line.)

Previous update, 2026-08-22 (M2 freeprog DIG-IN RESOLVED: the topple was
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
  AUDIT (08-22, before spending a reward-shape rewrite): the SLIPWALK
  bank's own scripted twins, run under this exact reward stack, ALREADY
  rank a real gait far above marching-in-place (`creep` +108/ep, 0.16m
  travel, vs `stall` (scripted march-in-place) -143/ep, vs `park` -244)
  — the reward's THEORETICAL ranking is correctly aligned; the trained
  policy's own march is even worse than the bank's clean `stall` twin
  (ep_rew -1267 vs -143), i.e. a noisy, high-slip version of marching,
  not evidence the reward ranks it well. So this reads as an RL
  exploration/local-optimum problem, not a ranking defect — matching
  this codebase's OWN prior fix for the identical failure signature on
  the joystick track (`cw-dep-bcgait1-hard1-steer2-stagecurric1`:
  "full-mix exposure from step 0 is not enough", PASSED once staged).
  Neither term400 arm was ever exposed to a simple sub-task: both threw
  the FULL stress_mix envelope (resample 1.75s, yaw, 15% stops) at a
  from-scratch actor from step 0. LAUNCHED (single lever, no reward
  edit): `cw-amp-m2-freeprog-term400-stagecurric` (train-3) — the
  existing `sched.*` engine ramps `goal.walk_cmd_stage` 0->2 over the
  first 60% of the 2M budget (forward/back-only, then headings/turns,
  full family+jitter only by 1.2M), everything else byte-identical to
  `-term400-noamp`. **VERDICTED FAIL 08-22**: ramp verified working
  (sched 0->2 on schedule), survival held (ep_len 13->348, terms
  123->~10), but det fwd travel med 0.02m (WORSE than noamp's
  0.026-0.032m), gait_valid 0/6 det, sacrificed rear legs [3,5],
  video = the same sprawled statue; freeprog_pen pinned -1.5/tick all
  4 quarters (reward flat + eval flat = stuck mechanism per the 08-22
  agreement rule). Even the stage-0 forward-only sub-problem never
  left the statue basin -> staging/exploration lever class CLOSED for
  this family (joins term-penalty and std-anneal). The pre-registered
  net-displacement-floor patch is DEPRIORITIZED, not next: its premise
  (misranking) was refuted by this section's own bank audit (creep
  +108 >> stall -143 >> park -244) and `k_walk_idle_charge=20` already
  charges the statue a smoothed travel floor in full — another charge
  deepens the statue's cost but creates no gradient TOWARD stepping.
  The untried gradient SOURCE is the style channel itself: every style
  arm so far trained against the frame-corrupted teacher_v1 lib AND at
  a priced-out weight (max 0.5/tick style vs ~-1.5/tick statue
  charges). LAUNCHED 08-22: `cw-amp-m2-freeprog-term400-style05-v2`
  (teacher_v2 lib at 0.5/0.5) and `-stylew2-v2` (teacher_v2 +
  style/task 2.0/1.0). **BOTH VERDICTED FAIL 08-22, STYLE-DOSE LEVER
  CLOSED PER THE PRE-REGISTERED RULE**: style05-v2 (det fwd med
  0.03m, slip 8.09/m, gait_valid 5/6) and stylew2-v2 (0.03m,
  10.45/m, 5/6) are visually and numerically indistinguishable from
  each other AND from the plain -noamp control — the clean teacher_v2
  lib retires the "corrupted v1 lib" caveat with no behavior change,
  and 4x the style weight (2.0 vs 0.5) barely moved realized style
  income (`env/reward_amp_style` final 0.135/tick vs 0.027/tick) —
  both an order of magnitude short of freeprog_pen's flat
  -1.4 to -1.5/tick. Discriminator stayed healthy both arms (d_real
  0.78-0.79 vs d_fake -0.96, unsaturated) but style_reward_mean
  itself stayed pinned low (0.05-0.07) the entire run in both — the
  policy never learns to imitate even partially, so the pre-
  registered in-place-mimicry cheat never had a chance to fire
  either. W&B reward FELL every quarter in both (never rising) —
  genuine flat/declining FAILs, not continue-longer cases. Per the
  pre-registered decision rule: the AMP-style-income route is
  refuted at 2M discovery scale for the freeprog/term400 pricing
  family across the whole tested dose range (0.5x-2.0x).
  **TASK-COMPLEXITY HYPOTHESIS REFUTED, n=2 SEEDS (08-22)**:
  `cw-amp-m2-freeprog-term400-fixedcmd`/`-seed11` — single lever vs
  -noamp: replaced the full stress_mix command stream with the
  SLIPWALK bank's own literal fixed command (vx=0.05 m/s constant, no
  heading/resample/yaw/stop, byte-identical reward stack, no AMP) —
  the exact setup the bank already proves ranks real travel far above
  stationary behaviors, to test whether command COMPLEXITY (not
  reward shape) was ever the real barrier. BOTH seeds FAIL, and
  WORSE than the harder stress_mix arms: det gait_valid 0/6 both
  (seed7: legs 2,3,5 sacrificed every episode; seed11: legs 3,4),
  fwd med 0.03-0.05m (bar 0.10m), same freeprog_pen (~-1.37/tick) and
  walk_gait_min (~0.29) plateau as noamp, W&B reward falling every
  quarter in both (never rising). Simplifying the task did not just
  fail to help — it let MORE legs go idle/statue than the stress_mix
  arms did (0-1 sacrificed there vs 2-3 here). CONCLUSION: the
  from-scratch marching/statue basin is a genuine PPO exploration/
  optimization pathology independent of command distribution — the
  "is it reward-shape or task-complexity" question for M2 freeprog is
  now closed on the task side. CORRECTION: `goal.walk_gait_start_frac`
  RSI is NOT untried on `--impl warp` — `cw-gait-rsi1` (08-11, an
  older pre-freeprog/pre-term400 from-scratch stack) already ran it
  at frac=0.5 and was refuted for the identical freeze/statue
  signature; a concurrent cycle has read that as closing RSI for
  this family too and (same cycle as this entry) bank-tested the
  next mechanism instead — `reward.k_walk_swing` (any-direction
  lift-swing-touchdown bonus, no along-command gate, unlike
  k_step_event) on `SLIPWALK_SWING_OVERRIDES`
  (`test_task_semantics.py`, 11/11 passing, commit 1fb65603).
  **LAUNCHED 08-22 (this cycle)**: `cw-amp-m2-freeprog-term400-
  swing-{noamp,style05}` (single lever, `reward.k_walk_swing=1.0` on
  top of the verdicted -noamp and -style05-v2 configs respectively,
  2M discovery, RUNNING on train-0/train-1). Bank numbers backing the
  launch: real gait/creep income up ~11-23% (558->622, 103->126)
  while every stationary twin (stall, park, skate) AND a new
  realistic farming twin (`shuffle`: a genuine coordinated six-leg
  gait reversing direction every 0.6s, ~0 net travel despite real
  strides both ways — the direct analog of "farm the direction-
  agnostic bonus without going anywhere") stay priced BELOW park —
  the mechanism does not reopen a known stationary-cheat exploit.
  Pre-registered live-monitoring cheat (no scripted twin could
  trigger it, so NOT bank-cleared): a single-leg-farm pattern (one
  leg cycling, five sacrificed/static, `env/reward_swing` > 0 with
  det fwd travel still ~0.02-0.03m) — if seen on triage, FAIL
  regardless of return and bank+fix a `walk_gait_gate`-style MIN-
  across-six-legs gate on the swing bonus before any follow-up.
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
