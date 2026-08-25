# standwalk — mesh-model stance retrain, then distill into walking

Last updated: 2026-08-25 ~09:1x (**rung-4 product-gradient wave, first
cell read: `cw-standwalk-stance-mesh2-holdprod-f01` (floor 0.1, seed 0)
VERDICTED FAIL — the per-foot product income gradient does NOT hold the
scratch policy in the plant basin either.** Plain English: this arm
replaced the load-min gate (flat scraps the moment any foot unloads)
with a product-over-feet gate (each re-loaded foot multiplies income
back toward full pay), betting the missing piece was a graded income
path BACK to the plant. It isn't sufficient: hold panel 0/12 (DR-0 and
own-DR 0.2 both all-over_current), video shows the rear-up basin
(starts at the paid plant, rocks back, waves front legs aloft until OC
— holdload1min-seed0's shape), and the W&B trace is decisive on
mechanism: `env/hold_feet_factor` starts 0.95 at 65k (policy IS in the
paying basin) but collapses to 0.018 by 786k and never recovers past
0.13 — PPO walks OUT of the basin in the first ~0.7M steps and the
product gradient never pulls it back. Reward monotonically worse every
quarter (-102.6/-407.6/-645.6/-763.8): genuine FAIL, not an 08-21
continue case. One cell of four — the joint floor-0.1/0.3 x seed-0/1
closure belongs to the sibling reads (`f01-s1`, `f03`, `f03-s1`, ALL
just finished at 6M ~09:0x, each owned by its own triage cycle; the
watcher's SUSPECT on f03-s1 was a false alarm — clean W&B sync at its
budget). Per the pre-registered FAIL branch: no combo arm launches
until the product closure + the noise axis (-dr0 already FAIL, -ent4
concurrent cycle) are all read. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_holdprod_f01_{gate,owncfg}/`, W&B `c5i5ktyj`.)

Previous entry: 2026-08-25 ~09:0x (**`cw-standwalk-stance-mesh2-riseonly1-acq1`
VERDICTED FAIL — second independent confirmation that budget-alone
continuation of an isolated-mode 2M checkpoint makes things WORSE, not
better; closes the "just train longer" branch for rise the same way
`holdonly1-acq1` closed it for hold (and `loweronly1-acq1`, a sibling
cycle's read, closes it a third time for lower).** Plain English: this
was the +8M continuation (10M total, unrepriced, rise=1.0 isolated
diet) testing whether more budget teaches a lower-torque way to hold
the risen pose instead of fighting gravity at the current ceiling. It
doesn't: at 2M the parent's failure mode was purely `over_current`
while genuinely upright (clean rise motion on video); at 10M it is
0/6 det, 0/6 sto valid-plant, and the termination signature shifted to
mostly `tilt_pitch`/`tilt_roll` — real falls, not a stable-but-hot
hold. Training reward got monotonically WORSE every quarter
(-153.8/-300.9/-413.4/-438.8), never recovering — this is the
`holdonly1-acq1` shape (behavior itself regressing under more
on-policy pressure), not the "declining return via accumulating hot
charges on an improving behavior" shape that would have justified more
budget per the 08-21 ruling. Does not change the track's current
direction (rung-4's `hold_feet_load` structural fix is already the
active line, per the entry below) — this just retires "budget alone"
as a lever for isolated-mode rise/lower/hold continuations across all
three modes now tested. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_riseonly1_acq1_{gate,owncfgowncfg}/`, W&B
`fbdij62l`.)

Previous entry: 2026-08-25 ~08:4x (**rung-3 load-min wave: 2 of 3 arms
FAIL — a NEW failure basin (belly-flop freeze) named, the stilt-rescue
lever CLOSED, and two single-lever optimization arms launched as the
rung-4 batch.** (a) `holdload1min-s1` (scratch seed 1) FAIL: under the
load-min income gate PPO finds a BELLY-FLOP FREEZE — drops from the
planted start onto its belly by t=3s and freezes; survives 12/12 with
ZERO terminations (roll_tail 0.0, cur_p95 0.5A) but h_err=70mm,
all-feet duty <=0.27, never-planted, return -802/ep vs honest +1471.
KEY READ: the reward is ALIGNED (honest plant dominates 2.9x and the
hold episode STARTS in the paying basin) yet training reward declined
all run (-94 -> -627) — this is an EXPLORATION/OPTIMIZATION failure,
not pricing: term_cost never fires (it survives), current_hot barely
fires, and min-over-feet income has no gradient from belly to plant.
(b) `holdload1min-warm` FAIL exactly per prediction-if-false: the
holdonly1 stilt cannot be locally deformed into a plant — 12/12 tilt
terms both panels, duty still stilt-shaped, sto rides the current
ceiling (p95 2.6A). 2/2 continuations from the stilt checkpoint now
destabilize (acq1 without repricing, this with) — **holdonly1 is
RETIRED as a parent**. (c) Scratch seed 0 (`holdload1min`) is the
concurrent cycle's read, still pending; joint scratch-pair call is
theirs to complete, but s1's basin is decisive on its own terms.
(d) RUNG-4 LAUNCHED (2 arms, both single-lever off the holdload1min
scratch recipe, reward untouched so the green bank still covers them):
`cw-standwalk-stance-mesh2-holdload1min-dr0` (dr-scale 0.2 -> 0.0,
train-1 — is flat DR-from-step-0 what knocks the random-init policy
out of the plant equilibrium? rung-2 candidate (b), first time
actually tried) and `-ent4` (ent-coef 0.005 -> 0.02, train-0 —
does sustained entropy prevent the premature variance collapse into
the freeze? rung-2 candidate (c), first time tried). Both VERIFIED
RUNNING, 6M, same hold gate as holdload1min. Read jointly: dr0-pass ->
build the DR ramp; ent4-pass -> entropy retune rung; both-fail ->
hold curriculum / pose-anchor mechanism (bank work first). NOTE: the
concurrent cycle has 4 `holdprod-f01/f03[-s1]` arms queued (product-
variant income dose) — reward-side lever, complementary, do not
duplicate.)

Previous entry: 2026-08-25 ~08:2x (**ADDENDUM to the ~08:0x synthesis
below — corrections + the last wave arm verdicted + rung-3 is a
3-arm batch, not one run.** (a) CORRECTION: `holdload1min` is
FROM-SCRATCH (respec of holdonly1's recipe + the load-min gate), not
a warm-start; the warm-start question is its own arm. The full rung-3
batch launched 08-25 ~08:0x-08:2x: `cw-standwalk-stance-mesh2-
holdload1min` (scratch seed 0, 6M, train-0 — already FINISHED at
~08:15, next cycle triages), `-holdload1min-s1` (scratch seed 1, 6M,
train-1), `-holdload1min-warm` (WARM from the holdonly1 stilt
checkpoint, 4M, train-4 — tests whether repricing deforms the
already-balanced stilt into a six-foot plant, vs acq1's
no-repricing continuation which destabilized it). All gated on hold
panel DR-0 det+sto n=12: >=10/12 survive, zero OC/tilt, six-foot
stance, cur_p95<=1.0A. (b) Pricing-probe backing (this cycle,
`logs/probe_stance_pricing_holdload.json`; probe gained a `loadX[min]`
token): under the launch dose + load-min gate, honest six-foot hold
keeps EXACTLY 1471.6/ep while the actual holdonly1 stilt policy drops
+511 -> -94.5 (product variant -77.3) — stilt strictly unprofitable,
honest untouched. (c) Last diagnostic arm `cur1-reftrack10` VERDICTED
FAIL (eval run manually — watcher prestage skipped it): k=10 at 2M
buys a FLAG-LEG FREEZE basin (det hold/lower ZERO terminations both
DRs — the only full-mix arm that doesn't fall — but hold ok 0/6,
one front foot aloft, rise still 5-6/6 terms, sto collapses).
CAVEAT for future reads: lower det scored 4/6 "ok" on the height
clause while the video shows the same frozen flag stance — the lower
ok criterion is lenient to flag-leg freezes; require the plant/duty
clause too. With refgain15 (k=15, 20M) FAIL, the ref-gain lever is
closed at all doses. (d) NOTE: `riseonly1-acq1` (continuation of
riseonly1, +budget) was launched by a concurrent cycle whose read of
riseonly1's video ("clean rise, only OC terminations") disagrees with
the recorded FAIL verdict (sprawled press-up); its own 10M panel will
adjudicate — owner triages it, do not double-verdict.)

Previous entry: 2026-08-25 ~08:0x (**Full diagnostic wave (7 arms) now
CLOSED — pricing is exonerated, the multi-task goal-mix is the
blocker; rung-3 (bank-checked `hold_feet_load` income gate, min-over-
feet variant) already launched off the fresh conclusion.** Plain
English synthesis of everything triaged this cycle + the concurrent
cycle (all verdicts committed): the rung-2 total collapse
(cur1/seed1/seed2, 0/3 healthy seeds) sent out 6 single-lever
diagnostic arms plus the already-in-flight `refgain15`, all now read
together —
(1) **pricing is not the lever**: all 4 corners of the current_hot x
term_cost grid were tried (`rr1`=neither -> profitable grind, `cur1`
=both -> total collapse, `curonly`=hot-only -> total collapse,
`termonly`=term-only -> grind returns) — current_hot is NECESSARY to
kill the grind (termonly proves it) but NOT SUFFICIENT to make the
full-mix recipe learnable (curonly proves it), and dosing the
ref-tracking gain 7.5x (`refgain15`) didn't help either. (2) **balance
IS learnable in isolation** — `holdonly1` (hold=1.0 diet, same
pricing) survived 6/6 DR-0 det hold episodes, refuting "physics is
broken on mesh." But the learned hold is a hot THREE-LEG STILT (other
3 feet held 15-20mm aloft, current riding just under the priced 2.0A
threshold) that the bank shows earns ~3x less than honest six-foot
quiet standing (504 vs 1472/ep) — a local basin, not the optimum. (3)
**Budget alone does not anneal the stilt**: `holdonly1-acq1` (+8M
continuation, 10M total) made it WORSE, not better (hold 0/24 vs
parent 6/6, reward declining in lockstep) — this rules out "just
train longer" and points at a reward-shape fix. (4) **rise and lower
each fail from-scratch even at 100% diet share**: `riseonly1` (11/12
DR-0 over_current/sprawl) and `loweronly1` (6/6 DR-0 over_current
splayed grind) both fail identically to their share in the full mix —
neither mode is diet-starved, both need a hold-capable PRIOR
(warm-start), not more curriculum share or ref-tracking gain.
**Conclusion or rung-3**: curriculum/goal-mix STRUCTURE (not reward
magnitude, not pricing) is the lever, and the first sub-problem to
solve is turning the tripod-stilt hold basin into an honest six-foot
stance via a load-gated income term. That arm is already running:
`cw-standwalk-stance-mesh2-holdload1min` (bank-checked
`reward.hold_feet_load=1` + `hold_feet_load_min=1`, min-over-feet
variant per the primitive-era one-foot-shedding lesson;
`test_task_semantics.py` HOLD-feet-load + MIN-over-feet banks both
green pre-launch; 6M budget, warm-start from `holdonly1`, gated on
six-foot valid_plant + zero OC + cur_p95<=1.0A at 6M) — VERIFIED
RUNNING train-0, owned by whichever cycle reads its 6M checkpoint.
Once an honest six-foot hold checkpoint exists, the pre-registered
next step is warm-starting rise-only and lower-only continuations
from IT (not from-scratch) to test whether the hold-first prior is
what rise/lower were actually missing. Evidence for every arm above:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_{holdonly1,holdonly1_acq1,
refgain15,curonly,termonly,riseonly1,loweronly1}_{gate,owncfg}/`,
RL_LOG 08-25 07:1x-07:5x.

Previous entry (2026-08-25 ~07:3x (**`cw-standwalk-stance-mesh2-refgain15`
VERDICTED FAIL: 7.5x the rise-ref-tracking gain does NOT fix the
rung-2 total collapse — ref-tracking weight is exonerated, goal-mix
implicated instead.** Plain English: this arm kept cur1's exact
pricing (current_hot+term_cost) and goal-mix (hold=.1/rise=.45/
lower=.45) but raised `k_rise_ref_track` 2.0->15.0 to test whether
weak guidance toward the known-good rise trajectory was why PPO never
found a stance basin. It didn't help: 0/36 stance episodes ok on
BOTH DR-0 gate and own-DR(0.2) (every mode, det+sto), training reward
ended WORSE in Q4 (-373) than Q3 (-134) — not a "still rising" case.
Video (hold/rise/lower det_0) shows the SAME distinctive pathology in
every mode: the robot rears up off its plant pose into a near-vertical
splay with 2-3 legs jutting out, then falls or trips current — this
is a DIFFERENT shape from the tripod-STILT the concurrent
`holdonly1` arm survived (6/6, see banner below), even though both
use the identical current_hot/term_cost pricing. **The one variable
that differs between the two is the goal-mix itself**: holdonly1's
isolated hold=1.0 diet stays alive under this pricing; the full
hold/rise/lower mix does not. This is the cleanest lead yet for the
rung-3 recipe: stop iterating on reward MAGNITUDE (ref-track gain,
now tested at 2.0 and 15.0, both dead) and test the goal-mix/
curriculum STRUCTURE instead (e.g. per-mode-isolated training passes
a la holdonly1/riseonly1/loweronly1, or a staged curriculum that
starts hold-only and phases in rise/lower once hold is solid).
Pending: the curonly/termonly pricing-isolation pair (concurrent
cycle, still triaging) will say whether pricing ALONE (independent of
goal-mix) is sufficient to reproduce the collapse, or whether it's a
goal-mix x pricing interaction. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_refgain15_{gate,owncfg}/`, W&B `75dqyaxt`.
Also this cycle: ran `ops.sh podeval` by hand (unclaimed, pods idle,
no eval report existed) for three sibling discovery arms that
finished earlier without a prestaged read —
`cw-standwalk-stance-mesh2-loweronly1`, `-riseonly1`,
`-cur1-reftrack10` — all three now VERDICTED FAIL at their 2M canary
gate, but with a genuinely useful pattern: **isolated single-mode
training (rise alone, lower alone) produces CLEAN, non-pathological
motion** (rise: smooth push-up to a level plant; lower: an upright
stand, just not low enough) — a completely different failure shape
from the full goal-mix's rearing/splay collapse (cur1/seed1/seed2/
refgain15, all 0/36). What kills both isolated arms is the SAME thing
in both cases: every det episode pins at the actuator current ceiling
(cur_p95 2.5-2.65A) holding the near-target pose and trips
over_current — the MOTION is right, the HOLD is too hot. Reward was
still declining (not flat) at 2M in both, matching the
already-validated holdonly1->holdonly1-acq1 continuation shape, so
**launched `cw-standwalk-stance-mesh2-riseonly1-acq1` and
`-loweronly1-acq1`** (+8M each, 10M total, single lever = budget,
same recipe/pricing), both VERIFIED RUNNING (train-2, train-0), gated
on >=4/6 det+sto valid-plant with cur_p95<=1.5A at 10M; pre-registered
fallback if still current-pinned at 10M is a torque/effort shaping
term or goal.lower_height_mm mesh recalibration (lower only).
`cur1-reftrack10` (the 2M full-goal-mix sibling of refgain15, dose
10.0 instead of 15.0) closes the ref-track-weight axis for good (0/6
rise, matching refgain15's 15.0 dose and cur1's own 2.0 — three doses,
zero effect) but also surfaced a side lead worth flagging for rung-3:
at just 2M, LOWER alone scored 4/6 clean success even embedded in the
full competing goal-mix — far better than cur1's own 0/36-at-20M
collapse — while reward quarters START positive (32.1) and decline
thereafter (unlike cur1's flat-negative-from-Q1 shape). Reads as the
full-mix recipe finding a partially-good policy early and drifting
AWAY from it with more training under this exact pricing (a
misalignment story, not "needs more budget") — worth an intermediate-
checkpoint pull + panel read before trusting a 20M full-mix
continuation. Not acted on this cycle (single unreplicated 2M read);
flagged here for whichever cycle designs the rung-3 full-mix recipe.
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_{loweronly1,
riseonly1,cur1_reftrack10}_{gate,owncfg}/`, W&B `1dg40jin`/`myzj5aoo`/
`fhk6ir7p`.)

Previous entry (2026-08-25 ~07:2x (**FIRST isolation result:
`cw-standwalk-stance-mesh2-holdonly1` VERDICTED PASS on its canary
gate — balance IS learnable on mesh/100Hz; physics/gains-audit branch
is DEAD.** Plain English: with hold=1.0 diet the robot survives all
6/6 DR-0 det hold episodes for the full 15 s (roll tail 1.0°, zero
terminations), refuting "balance itself is broken on mesh" — cur1's
hold collapse was starvation/recipe, not physics. BUT the learned
hold is a hot TRIPOD STILT: three feet held 15-20 mm aloft (duty
[.99,.04,.99,.02,1.0,.02]), valid_plant=false, current riding just
UNDER the 2.0 A priced-hot threshold (p95 1.8 A, ceiling touches
2.64 A, 14.1 s above soft), and 5/6 over_current trips on BOTH sto
panels (own-DR det 4/6 ok, 0 terms). NOTE the return-scale trap for
other isolation reads: training reward DECLINED all run
(5.6/-51/-162/-218) while eval survival IMPROVED 0.5@1M → 1.0@2M det
— under this pricing, longer survival accumulates per-tick hot
charges, so a declining reward curve does NOT mean nothing is
learning (riseonly1's "declining, bad early sign" below should be
re-read with this in mind). The probed bank prices honest six-foot
quiet hold at ~1472/ep vs this policy's 504 (honest draws 0.15 A
mean), so the tripod is a LOCAL basin, not the reward optimum —
08-21 continuation case. Launched:
`cw-standwalk-stance-mesh2-holdonly1-acq1` (+8M continuation from
the holdonly1 checkpoint, VERIFIED RUNNING train-0) gated on
six-foot valid plant + zero OC det+sto + cur_p95<=1.0 A;
pre-registered fallback if it still stilts at 10M total:
`reward.hold_feet_load` income gate (existing default-OFF cfg key,
sim_env.py ~3094), bank-check REQUIRED before that arm. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_holdonly1_{gate,owncfg}/`,
W&B `khyece06`.)

Previous entry: 2026-08-25 ~06:5x (**Per-mode isolation diagnostic BATCH
launched (4 arms) to root-cause the rung-2 total-collapse FAIL below.**
Plain English: rung-2 (mesh2-cur1/-seed1/-seed2) failed EVERY mode
including plain `hold` from an already-planted start, with flat reward
the whole 20M run — a genuine stuck-mechanism FAIL, not a misalignment,
per this cycle's own triage of `cur1`/`-seed1` (0/6 all modes both DR-0
and own-DR; term signature MIXED per mode — hold tilt_roll, rise
tilt_pitch, lower over_current — three distinct failure modes, not one
uniform grind). Before spending a 3rd full 3-seed 20M batch, this
cycle isolates WHICH mode(s) are actually unlearnable vs. starved by
the multi-task goal-mix (hold is only 10% of it) or by a too-weak
rise-ref tracking weight (`k_rise_ref_track=2.0`), via 4 cheap 2M
discovery arms off the same cur1 pricing recipe (single-lever each):
`cw-standwalk-stance-mesh2-holdonly1` (goal-mix hold=1.0, isolates
whether quiet standing alone is learnable), `-riseonly1` (goal-mix
rise=1.0, isolates whether rise alone tracks the reference — FINISHED
already, reward quarters 7.3/-36.8/-147/-232.8, DECLINING not
recovering, a bad early sign pending its stance-panel read),
`-loweronly1` (goal-mix lower=1.0, isolates the named
lower-height/current fallback fork), and `-cur1-reftrack10`
(full mix unchanged, `k_rise_ref_track` 2.0->10.0, tests the
alternative "weight too weak" fix instead of curriculum share). All 4
VERIFIED RUNNING/FINISHED (train-0/1/2/3). Read jointly with the
concurrent cycle's own parallel lever `cw-standwalk-stance-mesh2-
refgain15` (same base recipe, a different `k_rise_ref_track`-style
gain choice, independently launched — not a duplicate, a second dose
point) — next triage cycle should read all 5 together before deciding
rung-3. None of these are the pricing-OFF isolation / DR-ramp / log-
std retune candidates named below (still unbuilt; queue those next if
this batch doesn't name a clear fix).

**ADDENDUM (this cycle, same wave):** added the pricing-OFF isolation
as a proper ablation PAIR instead of waiting: `cw-standwalk-stance-
mesh2-curonly` (`term_cost_per_remaining_s` forced to 0.0, `current_hot`
pricing unchanged from cur1) and `-termonly` (`k_current_hot` forced to
0.0, term_cost unchanged) — single-lever complements, both 20M, both
VERIFIED RUNNING (train-6/train-4). Rationale: mesh1-rr1 (NEITHER
charge) found a rising-reward grind; mesh2-cur1 (BOTH charges) found
nothing (flat reward); these two arms name which charge alone is
sufficient to block all learning vs. which is safe alone. Read all 6
diagnostic arms (holdonly1/riseonly1/loweronly1, refgain15/reftrack10,
curonly/termonly) together for the rung-3 recipe decision — do not
launch more same-lever variants until this wave reports.

Previous entry (2026-08-25 ~06:4x (**RUNG-2 (mesh2-cur1) FAIL, 3/3 seeds
CONFIRMED, TOTAL collapse -- worse than a misalignment, a genuine
stuck-mechanism FAIL. Rung closed; root-cause dig-in needed before
rung 3.** Plain English: the realigned-pricing rung
(current_hot=1.0@2.0A + term_cost, meant to fix the mesh1-rr1 grind
exploit) does not merely fail to improve -- it fails to find ANY
stable stance at all. `cw-standwalk-stance-mesh2-cur1` and `-seed1`
both verdicted FAIL this cycle: 35-36/36 episodes terminated (DR-0 gate
+ own-DR 0.2) via tilt_pitch/tilt_roll/over_current on EVERY mode,
including `hold` (just standing still at the plant pose) -- video
(hold_det_0) shows the robot visibly tipping over during a plain hold.
Training reward is FLAT the whole 20M run on all 3 seeds (quarters
cur1 -293/-339/-276/-285, seed1 -318/-533/-311/-297, seed2 -315/-516
/-298/-364 -- same shape, dip-then-partial-recovery, never net
positive) -- reward AND eval both flat/bad = a genuine FAIL per the
08-21 ruling, not a misalignment to realign-and-continue. Distinct from
the mesh1-rr1/seed2-rr1 failure mode (which at least found a
profitable-but-wrong grind basin) -- this rung's pricing (or the
from-scratch mesh/100Hz recipe itself: log-std-init=0, ent-coef=0.005
unchanged from the primitive-era defaults, DR=0.2 from step 0 with no
ramp) is not finding a stance basin at all. `-seed2` CONFIRMED the same
(35/36 terminated, identical shape) -- 3/3 seeds agree exactly, this is
a recipe-level failure, not a seed-lottery result. **Next rung
candidates (not yet built)**:
(a) a pricing-OFF isolation probe (does hold alone stabilize with
current_hot/term_cost removed -- tests whether the NEW pricing itself
is destabilizing vs. the base recipe just being too hard on mesh);
(b) a DR curriculum ramp (0 -> 0.2 over the run, instead of flat 0.2
from step 0 -- first real mesh contact might need an easier start);
(c) re-examine log-std-init/ent-coef for a from-scratch mesh/100Hz
task (both values are untouched primitive-era defaults, never
re-tuned for this recipe). Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_cur1{,_seed1}_{gate,owncfg}/`, W&B
`9jgempi8`/`h8nqxuk2`.)

Previous entry (2026-08-25 ~05:5x (**dig-in DONE, both stage-1 seeds
VERDICTED FAIL-misaligned, realigned pricing rung launched.**)

## Dig-in resolution (08-25 deep cycle — supersedes the open flags)

- **Root cause found and measured.** The "exact 2.64 A" is the actuator
  torque ceiling: cur = |torque| x 1.2 A/Nm, ceiling 2.2 Nm -> 2.64 A;
  trip = 2.5 A sustained 0.8 s (`rl_move/safety.py`). own-DR spread
  2.52-2.66 = DR-scaled limits. Both seeds learned a **torque-saturation
  ground-grind**: servos parked at the ceiling fighting contacts
  (seed2 video overlay: i_max migrates across servos, MEAN current
  0.37 -> 2.23 A — whole body, not one hot joint). Learning curves:
  both seeds survived ALL stance modes at 1M, collapsed to survived=0
  by 8-10M and never recovered while reward kept rising — PPO walked
  up an exploit gradient the gate kills. rr1's 19M bg-eval hold=1.0
  was an n=2 flicker (deterministic harness kills hold 6/6 both DR).
- **Not a sim/trip defect, and the cross-track 2.6-2.7 A correlation is
  closed:** that band is just the saturation signature (limit x 1.2),
  reachable on the 3.5 kg body by any reward stack with unpriced
  current. Honest hold draws 0.15 A mean / 0.41 A max; the honest rise
  replay TOUCHES 2.64 A transiently (no trip) — margin is thinner on
  mesh but the task is feasible. Per-track pricing fixes; no shared
  defect. (Relayed to joystick via this STATUS; their k_walk_current
  machinery already prices it on walk.)
- **Pricing probe (`rl_move/sim/probe_stance_pricing.py`, results in
  `logs/probe_stance_pricing_rr1*.json`)** rolled the ACTUAL rr1
  checkpoint vs honest scripted behaviors under the launch-exact mesh
  stack: at base pricing the grind is a PROFITABLE local optimum
  (hold +66, lower +187/ep) even though honest dominates (hold 1472,
  rise replay 2412) — misalignment = profitable cheat basin, not
  honest<cheat. Flat hot pricing @1.0 A REPRODUCES the kick cycle's
  bank breakage (honest mid-crouch runs one servo ~2.2-2.6 A
  legitimately: partial -649 < freeze). **Chosen dose: k_current_hot=1.0
  @ current_hot_a=2.0 + term_cost_per_remaining_s=3 (cap 60)** — grind
  negative in all modes (hold -39, lower -27, rise -773), honest rise
  keeps 90% (2160), hold untouched (1472), rise orderings preserved
  (replay > partial 412 > flagleg 58 > freeze -637 ~ stilt -771).
  All pre-existing default-OFF cfg keys; no env code changed; the
  committed primitive bank is bit-exact untouched.
- **Known residual risk (named fallback fork):** the LOWER task's
  25-55 mm crouch is intrinsically hot on mesh (~2.2+ A sustained on
  one knee even for the honest descent; honest -44 vs grind -27 at the
  chosen dose — profit erased but honest not yet dominant). If the
  realigned rung learns rise/hold but still fails lower, the next fork
  is recalibrating goal.lower_height_mm for mesh (likely belly-rest
  supported, servos unloaded — freeze draws 0.15 A) rather than more
  pricing. Bank debt: fold the grind rows into test_task_semantics
  when the first mesh stance pass records the reference band.
- Verdicts: `cw-standwalk-stance-mesh1-rr1` FAIL (0/36 ok DR-0,
  33/36 term; reward +90 Q4), `cw-standwalk-stance-mesh1-seed2-rr1`
  FAIL (36/36 term both panels; 2/3-height rise then lateral tip).
  0/2 completed seeds = recipe gap per the pre-registered rule
  (seed1-rr1 is another cycle's read and cannot change that joint
  conclusion). No continuation of these checkpoints — realign+relaunch:
  **`cw-standwalk-stance-mesh2-cur1` / `-seed1` / `-seed2`** (same
  recipe + the probed pricing, 20M, 3 seeds, launched this cycle).

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

**seed1-rr1 VERDICTED FAIL (08-25 ~05:3x cycle):** 0/60 stance
episodes (rise/hold/lower, det+sto, DR-0 gate AND own-DR 0.2), every
episode an over_current trip with Imax pinned ~2.64 A. Video: rise =
sprawled press-up with outrigger legs/skating feet, no valid plant;
hold DEGRADES from a planted start into a sliding sprawl (1352 mm
drag); lower ends belly-down but splayed, plant 0/6. Reward rose in
the final quarter (-95 vs -487) but task success was flat ZERO at
every in-training eval for all 20M => second-clause genuine FAIL for
this rung (bank-aligned reward + rung budget, gate unmoved), NOT a
continuation candidate. Identical signature to seed0's gate report —
2/2 seeds so far point at the pre-registered "recipe gap" branch;
JOINT READ (and rung-2 batch design) belongs to the cycle holding
rr1 + seed2-rr1. Rung-2 lever candidates from this triage: rise-ref
tracking k 10-20 or ref-clocked anchoring (the 25 Hz ref replays to
a valid plant 3/3 on mesh, so it IS followable — k=2.0 just loses to
the shaping terms PPO games), a bank-proven stance-tick current-dwell
charge (movecur analog; honest mesh hold draws 0.41 A max vs the
2.64 A the policies pin), and/or a hold-only-first rung split.

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
