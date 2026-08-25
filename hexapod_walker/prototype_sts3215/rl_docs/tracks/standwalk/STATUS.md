# standwalk — mesh-model stance retrain, then distill into walking

Last updated: 2026-08-25 ~18:1x (**FLAT-START ROOT CAUSE FOUND — the
BC anchor's height-floor pursuit SKIPS the mesh ref's entire tuck
segment from flat states; exposure refuted (flatmix70-s1 CANARY
FAIL-MECH); tuckfloor0 canary pair launched as the fix.** Seed-1 read:
flat-pinned probe 0/12 valid, all 12 over_current at exactly 2.64A,
same never-tucks radial press-up (h_err 13-62mm); standard DR-0 gate
det 0/6 — the 0.70-flat mix sampled 4 flat + 2 rsi det starts and BOTH
rsi det starts regressed into the press-up basin (vs meshref det 5/6);
sto 4/6 (rsi/bridge clean, flat 0/2). Seed-0's standard gate
replicates episode-for-episode (det 0/6 all oc incl. both rsi, sto
4/6; its verdict is its own cycle's). MECHANISM (measured):
`rise_ref_mesh_scripted.npz` tucks for ticks 0-245 (~4.9 s) at exactly
h=0.0 mm, then presses 0→83 mm over ticks 245-400; the state-aligned
anchor's `train.bc_anchor_min_h_ahead_mm=8` floor requires the target
tick to command >=8 mm above current height, so from ANY h≈0 state the
first qualifying tick is ~258 — the anchor jumps the whole tuck and
supervises flat starts straight toward press-phase legs-under-body
poses while the legs are still splayed = the 2.64A pin. Non-flat
starts match at/past the tuck (why only flat fails); the floor is
~no-op in the press segment (climbs ~0.5 mm/tick, floor 8 mm ≈ 0.32 s
≈ the 0.25 s lookahead) — it was anti-freeze for the LEGACY ref's 5 s
0→25 mm crawl and is segment-blind to a zero-height tuck. This also
retro-explains why every exposure/budget/pace/dose lever failed: the
supervision itself is wrong at flat states. FUNDED: `tuckfloor0` /
`tuckfloor0-s1` 2M canary pair — exact meshref recipe, single lever
`min_h_ahead_mm` 8→0 (the key's own bit-exact default), restoring tuck
supervision from flat while leaving press supervision effectively
unchanged; judged jointly on the flat-pinned probe + standard gate
(PASS → 8M + port to stancemix; PARTIAL/stall-return → code the
tuck-exempt floor, i.e. floor active only at/after `ramp_i0`; FAIL
with anchor verified pointing into the tuck → tuck-segment start
curriculum next). Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
riseonly_bcchain3_meshref_flatmix70_s1_{gate,owncfg,flatprobe}/`, W&B
xz8urts2.)

Prior entry: 2026-08-25 ~17:5x (**8M GRID CLOSES with seed-2 — budget
is NOT the lever on the mesh-native ref; PACE AXIS ALSO CLOSED —
half-pace stays the default. Flat-start tuck is the sole residual, and
the running flatmix70 exposure pair is the funded attack on it.** This
cycle's verdicts complete the reads the entry below was waiting on:
(1) `-8m-s2` **FAIL per the grid's per-seed budget-refuted branch** —
eval-identical to the 2M canary pair (DR-0 rise det 5/6 + sto 4/6
valid_plant, oc 3/12 = det flat + 2 sto rsi all pinned 2.64A, valid
bridge episodes at cur_p95 2.24-2.33A over the 1.5A clause; video =
the same splayed-front press-up, h_err 20-25mm; reward rose to 1134
throughout). Grid tally: s0 PARTIAL (oc 2/12, within the same-seed
noise band the dup-twin defined), s1 PARTIAL (exact canary plateau),
s2 FAIL — the >=2/3-seed strict-PASS is impossible, the 3-seed 8M
grid is CLOSED, and 2M/5M/8M all land at canary level: budget bought
nothing on this ref. (2) `-fullpace2` **CANARY PASS (PARTIAL
branch)** — full pace trains cleanly to its 2M budget (its watcher
SUSPECT was a false alarm: clean budget-complete 2,031,616-step exit,
W&B 3kgxqusz) but regresses det to 4/6 with a NEW bridge 2.64A
press-up pin absent in both half-pace seeds (sto 5/6, oc 3/12 equal,
one valid ep at 2.26A over the 2.25A band); its seed twin
`-fullpace2-s1` was verdicted CANARY FAIL - MECHANISM (det 2/6) by
its own cycle — joint pace read: full pace at best ties, usually
regresses; **keep half-pace (`bc_anchor_lookahead_s=0.25` /
`min_h_ahead_mm=8`) as the default chain pacing for the mesh ref.**
Net frontier: every generic lever (pace up/down, anchor dose, budget,
floor/lookahead) is now refuted on the mesh-native ref; the
flat-start never-tucks press-up is the only failing subclass, and the
running `flatmix70`/`-s1` exposure pair is the pre-registered
mechanism test — its FAIL branch pre-registers ref-content/tuck-phase
treatment next. Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
riseonly_bcchain3_meshref_{8m_s2,fullpace2}_gate/`, W&B 6pwzsqht /
3kgxqusz.)

Prior entry: 2026-08-25 ~17:4x (**RUNG-9 8M GRID, 2 OF 3 SEEDS READ:
neither beats the strict PASS bar; seed-0 nudged, seed-1 exactly
plateaued.** `meshref-8m-s1` (this cycle) PARTIAL — DR-0 gate det 5/6
+ sto 4/6 valid_plant, IDENTICAL counts to its own 2M canary, with
over_current 3/12 landing on the SAME start-kind set the canary named
(1 det:flat + 2 sto:rsi) — 4x the budget moved nothing for this seed.
Worse, 2 of the 5 valid det episodes (both bridge starts) run
cur_p95 2.27A/1.97A, over the gate's <=1.5A-on-every-valid-episode
clause, so this seed clears neither PASS nor even PARTIAL's own
"beats canary" bar in isolation. own-DR(0.2) det 3/6 + sto 6/6.
Video: flat/rsi deep starts genuinely tuck and lift toward a
splayed-but-planted stand before tripping current mid-hold — the
same qualitative story as every rung-8/9 sibling, not a freeze.
Training reward is strongly rising (quarters -18/-167/+340/+1041,
ep_rew_mean 1515) — per the 08-21 ruling this doesn't disqualify the
run, but paired with a flat-vs-canary eval it does NOT read as
"needs more budget" either; the residue looks like a hard floor for
this seed's init. `meshref-acq8m` (seed 0, concurrent-cycle read,
RL_LOG 17:34) instead nudged: det 5/6 + sto 5/6, oc 2/12 (<canary's
3/12) — not yet reflected in this file's own entries, folded in here
for the joint read. `meshref-8m-s2` (seed 2) still training — the
grid's own ">=2/3 seeds at the strict bar" call needs it; do not
close the grid on 2/3 reads, especially with the two available reads
disagreeing on direction (nudge vs. plateau). If s2 also
plateaus/regresses, the grid's own FAIL branch fires: budget is
refuted for the flat-start tuck residue, next lever is a targeted
tuck mechanism (start-mix weighting toward flat, or tuck-phase anchor
dose), not more budget. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_8m_s1_{gate,owncfg}/`,
W&B `q5fttoat`.)

Prior entry: 2026-08-25 ~17:3x (LEDGER CORRECTION, no science change:
the cycle that launched `-acq8m` briefly mis-marked it DEAD -
INFRASTRUCTURE (misread its ~8-min budget-complete exit as a launch-
race kill) and relaunched a verbatim duplicate `-acq8m-r1` on
train-4. Both errors reverted within minutes of the entry below
landing: `-acq8m` restored to RUNNING/unverdicted (it FINISHED clean
at 8,060,928 steps, W&B 08k9lmkm synced 17:15 — it IS the 8M grid's
seed-0, triage belongs to its fan-out cycle), `-acq8m-r1` killed ~5
min in and verdicted KILLED - DUPLICATE (no information lost; same
spec+seed as the completed acq8m). Net fleet state is exactly the
entry below: 8M grid = acq8m(s0, done)/-8m-s1/-8m-s2 + fullpace2
pair, nothing else standwalk-running.)

Prior entry: 2026-08-25 ~17:2x (**pace-redose canary pair recovered
and running.** The `-fullpace` launch was an infra dud (respec omitted
the two pace overrides → exact meshref-s0 duplicate, killed ~2 min in,
verdicted CANARY FAIL-INFRA by the launching cycle) and its claimed
relaunch `-fullpace2` NEVER actually landed — no ledger entry, W&B
run, or process existed, only the pre-launch snapshot commit. This
cycle relaunched it for real: `-fullpace2` (train-0, W&B 3kgxqusz) +
seed twin `-fullpace2-s1` (train-1), both VERIFIED RUNNING with
`bc_anchor_lookahead_s=0.5` / `min_h_ahead_mm=15` confirmed present
in the ledger commands; 2M canaries judged jointly as a pass-rate vs
the meshref s0/s1 pair (PASS = det>=5/6 + sto>=4/6, no episode
>2.25A, oc terms <=3/12 → full pace preferred, one less knob).
Also: watcher SUSPECT on `-acq8m` was a FALSE ALARM — clean
budget-complete exit at 8,060,928 steps, W&B synced 17:15
(08k9lmkm); `-8m` likewise finished clean at 8.06M @17:14 (its 17:19
duplicate-kill verdict predates the finish taking effect — treat
acq8m as the grid's seed-0). Triage of both belongs to their fan-out
cycles.)

Prior entry: 2026-08-25 ~17:0x (**RUNG-9 CANARY PAIR: CANARY PASS,
PARTIAL-strong — the mesh-native scripted rise reference IS the
lever the whole rung-8 dose grid was missing.** Both seeds
(`meshref`/`meshref-s1`, 2M, exact slowchain recipe with only
`reward.rise_ref_path` swapped to `rise_ref_mesh_scripted.npz`)
replicate each other almost exactly on the DR-0 rise gate: det 5/6 +
sto 4/6 valid_plant vs parent slowchain's 3/6+2/6; valid-episode
cur_p95 median 1.36A (s0) / 1.19A (s1) vs slowchain's 1.85A
bridge-press; deep starts (bridge/rsi) rise cleanly on video
(sprawl → six-feet plant → full stand, tilt<2°, h_err_end 3-9mm).
Missed the strict PASS bar only on the zero-over_current clause:
3/12 terms per seed — the flat-prone det/0 start plus 2 rsi-sto
episodes, where a splayed FRONT leg never tucks under the body and
the press against the extended lever arm pins 2.64A MID-RISE
(h_err_end 16-38mm — qualitatively different from slowchain's
freezes at 76-79mm/0.3A). Reward-quarter swings (12→-382) match
slowchain's own shape; recipe-normal, not collapse. FUNDED per the
canary's own promotion clause: 3-seed 8M acquisition grid — seed 0 =
`meshref-acq8m` (launched 17:01 by the concurrent cycle in a
triage-overlap race; both cycles independently reached the same
CANARY PASS read and funded the same promotion, and this cycle's own
seed-0 copy `meshref-8m` was KILLED as an exact duplicate) + seeds
1/2 = `meshref-8m-s1`/`-8m-s2` (this cycle) — exact recipe, only
budget changes; the 2M canary had already annealed std to 0.018, so
the 8M schedule's slower anneal is the exploration that could
convert the flat-start tuck. RUNNING train-0/train-3/train-2, judged
jointly as a pass-rate (grid gate on `meshref-8m-s1`) with a
pre-registered FAIL route to a targeted tuck mechanism (start-mix
weighting toward flat, or tuck-phase anchor dose). SKILLS.md deliberately NOT updated: the canary gate forbids
skill-acquisition claims; the 8m grid's full bar owns that. Once a
rise recipe passes in isolation, re-run the stancemix mix with the
mesh ref (per the stancemix FAIL verdict below). Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_
{gate,s1_gate}/`, W&B 5wdood22 / axy001yj.)

Prior entry: 2026-08-25 ~16:5x (2 verdicts, both reinforcing rung-9:
**anchordose10 CANARY FAIL - MECHANISM** — anchor coef 3.0→10.0 on
the slowchain recipe is monotonically DOWN (DR-0 valid_plant 2/12 vs
parent 5/12, over_current 9/12 vs 3/12, all pinned 2.64A;
bc_anchor_loss_rise ends 0.100 vs the 0.05 plateau — harder
supervision toward the infeasible flat posture just pins current
harder; ep_rew quarters 8→-419 as the anchor penalty dominates).
Dose10 can never be promoted regardless of anchordose6 (other cycle);
if dose6 also fails to beat 5/12, the anchor-dose axis is CLOSED
alongside pace/budget. **stancemix-bcchain3-slowchain FAIL** per its
pre-registered bar — the half-pace lever does NOT fix rise in the
3-mode mix, but hold/lower transfer perfectly: DR-0 hold det+sto
12/12 zero terms (cur_p95<=0.87A), lower 12/12 honest descents
(herr_end<=0.8mm), rise 2/12 with ALL 9 deep starts pinned 2.64A
(own-DR: hold 11/12, lower 12/12, rise 0/12). KEY DIAGNOSTIC: in-mix
bc_anchor_loss_rise fell to 0.027-0.035 — BELOW the ~0.05 riseonly
plateau — and env/rise_score hit 0.56 (riseonly peak ~0.43), yet
SCORE/raise_success stayed 0.0 for all 8M steps. Tracking capacity is
not the bottleneck; the borrowed primitive-extracted reference is —
the closed-loop twin of the rung-9 open-loop command-lag measurement
below. No further mix funding until a rise recipe passes in isolation
(meshref pair in flight); then re-run this exact mix. Evidence:
logs/ckpt_eval/cw_standwalk_stance_mesh2_{riseonly_bcchain3_slowchain
_anchordose10_gate,stancemix_bcchain3_slowchain_gate,stancemix_
bcchain3_slowchain_owncfg}/, W&B yei41azm / kza9ep2s.)

Prior entry: 2026-08-25 ~16:5x (**DECOUPLE GRID FULLY CLOSED —
`-decouple-b` (0.125s lookahead + floor 15mm) landed almost exactly
on the prior entry's pre-registered prediction, confirming the
floor-vs-lookahead synthesis with all three arms now read.** DR-0
gate det 2/6 [bridge1/2 crouch1/1 flat0/1 rsi0/2] + sto 3/6
[crouch2/2 rsi1/4] valid_plant (5/12 — beats quarterchain's paired
4/12) with ZERO freeze episodes (quarterchain froze 6/12) — the same
anti-freeze rescue decouple-c showed at 0.0625s, this time at the
middle lookahead. Own-DR(0.2) same shape: 5/12 valid (beats
quarterchain's 3/12), zero freeze (quarterchain 4/12). Cost of the
rescue, also matching the a/c pattern: over_current terms rise to
6/12 gate + 7/12 owncfg (quarterchain 1/12 + 5/12) — video confirms
freeze converts to a genuine-but-hot attempt, not a clean pass (det_0
flat start presses up to a raised splayed stance before tripping;
det_5 bridge — usually a clean starter — instead sinks/splays into
the SAME wall). **Joint read across all three (a FAIL 6/12 @0.25s,
b PARTIAL 5/12 @0.125s, c PARTIAL 6/12 @0.0625s): floor>=15mm gives a
flat 5-6/12 valid_plant ceiling REGARDLESS of lookahead length, with
the identical over_current-at-2.64A wall on rsi/bridge deep starts
every time — the 2D floor x lookahead pursuit-shaping grid is now
fully bracketed and CLOSED, no further arm on this axis is worth
funding.** This independently corroborates rung-9's own open-loop
measurement (next entry): the blocker is the legacy reference's
pacing/content, not the anchor's pursuit mechanics. Every named lever
on rise pursuit-shaping (pace, budget, anchor-dose, floor x lookahead)
is now exhausted; rung-9's mesh-native scripted reference (already
built, validated open-loop at 0.53A max, canary pair
`meshref`/`meshref-s1` training) is the sole remaining live lever.
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_
decouple_b_{gate,owncfg}/`, W&B `z36jhugx`.)

Prior entry: 2026-08-25 ~16:5x (**RUNG-9 BUILT, PROVEN, AND FUNDED:
a mesh-native scripted rise reference now exists, is torque-feasible
on the mesh model, and a 2-seed canary pair is training.** New tool
`rl_move/sim/make_rise_ref_scripted.py` (snapshot 25941468) mints a
belly→plant reference from geometry alone — NO checkpoint extraction:
tuck feet to the plant footprint while the belly carries the mass,
then a symmetric quasi-static Cartesian press via the trusted
`tripod_gait` FK/IK, then hold; the candidate must survive an
open-loop replay on the training model with a hard current bar
(p95<=1.5A) plus held-out-seed robustness before it may be written.
TWO KEY MEASUREMENTS from its probes: (1) the legacy
primitive-extracted ref's pacing sits far above the ~31 deg/s servo
velocity limit — at that pace even a clean symmetric scripted press
reproduces the 2.64A pin OPEN-LOOP as a pure command-lag convergence
transient (no policy involved); (2) at quasi-static pacing (tuck 3s /
press 5s / hold 2.5s) the full belly→plant rise runs at **0.53A max**,
ends +83mm and 0.8° RMS from plant — so a <=1.5A mesh rise IS
physically feasible; the task was never torque-blocked, the reference
content/pacing was the defect. Shipped
`rl_move/sim/refs/rise_ref_mesh_scripted.npz` (T=526, dt=0.02s,
ramp_i0=245, validation PASS). FUNDED: 2M canary pair
`riseonly-bcchain3-meshref` (seed 0) / `-meshref-s1` (seed 1), exact
slowchain recipe with ONLY `reward.rise_ref_path` swapped, judged
jointly as a pass-rate — VERIFIED RUNNING train-0/train-1. If both
FAIL with anchor loss converged, reference content is refuted
alongside pace/budget/dose and the next suspect is the
reward/termination pricing itself. Evidence: generator output in this
entry (reproducible via the command above), snapshot tag
`exp/cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref`.)

Prior entry: 2026-08-25 ~16:4x (**DECOUPLE GRID READ (a FAIL, c
PARTIAL): floor-vs-lookahead answered in one wave — floor strength IS
the independent anti-freeze lever, lookahead is flat once floor
>=15mm, the contemplated 2D grid is CLOSED as not worth funding, and
the deep-start over_current wall stands, exactly as the rung-9 entry
above measured open-loop.** `-decouple-c` (eighthchain's 0.0625s
lookahead + floor reverted 2->15mm) rescues eighthchain's total
collapse from 0/6+0/6-with-freezes to det 3/6 + sto 3/6 valid_plant,
ZERO freeze episodes, including a video-clean bridge-start rise
(det_1, hend 0.2mm) — decisive PARTIAL per its pre-registered
criteria. `-decouple-a` (slowchain's 0.25s lookahead + floor 8->15mm)
is FAIL: 6/12 valid vs slowchain's 5/12 (+1, inside noise) with
over_current terms DOUBLED (6/12 vs 3/12) — at an already-working
lookahead more floor only pushes deep starts hotter, mirroring the
anchor-dose overshoot. Synthesis: floor>=15mm at ANY lookahead lands
on the same ~6/12 ceiling (a 6/12 @0.25s, c 6/12 @0.0625s); BOTH
pursuit axes are now bracketed and no further pursuit-shaping arms
are justified. Every residual failure has the identical signature —
rsi/bridge deep starts tripping over_current at 2.64A MID-RISE while
making genuine progress (c det_3 video: legs tuck, body lifts, then
hot) — invariant across pace x floor x anchor-dose x budget, which
independently corroborates rung-9's open-loop finding that the legacy
ref's pacing (not the pursuit shaping) is the defect. `-decouple-b`
(0.125s + 15mm) still unread at write time; prediction: ~6/12, zero
freezes — a deviation (esp. >6/12 or freezes) would reopen the axis.
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_
bcchain3_decouple_{a,c}_gate/`, W&B `ahz9mrwq`/`yz8pe501`.)

Prior entry: 2026-08-25 ~16:2x (**ANCHOR-DOSE-UP AXIS CLOSED for rise:
`-slowchain-anchordose6`/`-anchordose10` BOTH CANARY FAIL - MECHANISM,
dose-insensitive between 6.0 and 10.0.** Doubling/3.3x-ing the BC-anchor
coef (3.0->6.0/10.0) off slowchain's own working half-pace does NOT
extend hold/lower's "more dose helps" precedent to rise — it overshoots
into a DIFFERENT, worse failure mode than the pace-dose siblings' cold
stalling: DR-0 gate det 1/6 valid_plant + sto 2/6 + 9/12 over_current
BOTH doses (slowchain baseline: 3/6 det + 2/6 sto + 3/12 terms); video
(dose6 rise_det_0) shows a genuinely MORE aggressive rise attempt
(body visibly lifting/tucking) that then trips hot mid-motion, not a
frozen splay — raising supervision strength pushes the policy to chase
the reference harder than the current budget allows, the opposite
direction from what helped hold/lower at low dose. Both arms' reward
trajectories overlay almost exactly (quarters ~8/-23/-220/-420) and
`bc_anchor_loss_rise` ends WORSE (0.098) than the ~0.05-0.06 plateau
every coef=3.0 sibling reaches — this is a real regime change at the
3.0->6.0 boundary, not a continued dose-response, so no intermediate
dose is worth probing. Combined with the pace-dose grid's own
exhaustion (quarterchain/eighthchain/slowchain-cont8, consolidated
below) and now this axis, BOTH named levers in every rise-rung gate
text to date are closed — rung-9 (mesh-native IK rise ref) and the
decouple-a/b/c floor-vs-lookahead split (launched this same window,
see entry below) are what's left in flight; a direct posture-reward
redesign on the flat/bridge/rsi segment remains unbuilt/unfunded.
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_
slowchain_anchordose{6,10}_{gate,owncfg}/`, W&B `r0htou5e`/`yei41azm`.)

Prior entry: 2026-08-25 ~16:2x (**Root-cause decoupling of the pace
scalar — LAUNCHED, a third parallel lever alongside the anchor-dose
grid and rung-9.** This cycle's own triage of `eighthchain` (verdicted
FAIL, matches the consolidated read below) found the mechanism behind
its total collapse: `sim_env`'s BC-anchor pursuit conflates TWO
distinct knobs into one "pace" scalar every prior arm moved together
— `train.bc_anchor_lookahead_s` (how far ahead the target sits once
IN a genuinely-climbing region = torque aggressiveness) and
`train.bc_anchor_min_h_ahead_mm` (the height-floor search that jumps
the target forward until it clears the reference's ~5s flat dead
zone = anti-freeze strength; see `bc_anchor.py`'s
"HEIGHT-FLOOR pursuit" comment, 08-12 origin). eighthchain paired the
SMALLEST floor (2mm) with the smallest lookahead and produced BOTH the
worst current pin AND a brand-new pathology never seen at any other
dose: several deep starts (`flat`/`rsi`) show ALL-SIX-LEG
`duty_cycle=0.0` for the full episode (video-confirmed total
belly-down freeze, height_err 79-86mm) — a 2mm floor is too weak to
force the target meaningfully past the flat zone regardless of
lookahead. Launched 3 arms isolating floor from lookahead (all
respec `--from slowchain`, from-scratch, unchanged budget/recipe,
VERIFIED RUNNING train-2/3/4): `-decouple-a` (lookahead 0.25s =
slowchain's, floor reverted to the ORIGINAL 15mm instead of
slowchain's paired 8mm), `-decouple-b` (lookahead 0.125s =
quarterchain's, floor 15mm — does a strong floor rescue quarterchain's
new freeze failure while keeping its shorter/gentler jump?),
`-decouple-c` (lookahead 0.0625s = eighthchain's, floor 15mm — does a
strong floor rescue even eighthchain's total collapse?). Gate:
same DR-0 rise det+sto n=6+6 harness; PASS clears
4/6+4/6+<=1.5A+zero-over_current+zero-freeze-episodes; PARTIAL beats
the SAME-lookahead paired-dose sibling (b vs quarterchain, c vs
eighthchain) on valid_plant or freeze-episode count even short of
PASS (confirms floor as an independent lever, motivating a proper 2D
grid); FAIL = flat/worse than the paired sibling, closing this
direction and leaving rung-9 (mesh-native ref / flat-segment edit) as
the sole remaining lever. Runs IN PARALLEL with the anchor-dose grid
and rung-9 — not a substitute for either, cheap enough (8M each, same
budget as the pace arms) to read before committing to rung-9's bigger
build. Evidence: ledger verdicts for `eighthchain`/`slowchain-cont8`
(FAIL, this cycle), W&B `l9tcutsx`/`5bx2x0n3`; decouple arms not yet
evidenced (just launched).)

Prior entry: 2026-08-25 ~16:1x (NOTE, same window, independent parallel
triage of `quarterchain`: this cycle's own read of `quarterchain`
landed the identical FAIL conclusion as the consolidated entry below
(det 2/6 worse than slowchain's 3/6, deep-start current/term proxy
metrics look great but `height_err_end_mm` 46-86mm on flat/rsi
exposes stalling-not-cooling; `env/rise_score` also peaks ~0.43 around
65% of training then declines to ~0.32 by the end, and
`bc_anchor_loss_rise` ends at 0.061 after briefly dipping to 0.049 —
both worth a second look if this rung's chain-loss plateau ever gets
revisited) — a genuine parallel-cycle duplication (ledger raced,
re-recorded, see RL_LOG 16:08/16:10), not a contradiction. Distinct
value-add before spotting the duplicate: launched a 2-arm anchor-DOSE
grid off `slowchain`'s own already-working half-pace checkpoint's
recipe (`train.bc_anchor_coef` 3.0->6.0 and ->10.0, 2M canaries,
VERIFIED RUNNING train-2/train-1) — rise has never been dosed above
the hold/lower-inherited default of 3.0, and this is the OTHER named
lever in every one of this rung's own gate texts ("attack the
flat-segment posture directly" / supervision strength, as opposed to
chain PACE which is now fully bracketed per the entry below). Runs
IN PARALLEL with rung-9's mesh-native-IK-ref proposal, not a
substitute for it — cheap enough (2M each) to read before committing
to the bigger rung-9 build. Evidence: ledger verdict + W&B `75d9j4tg`;
new arms not yet evidenced (just launched).)

Prior entry: 2026-08-25 ~16:1x (**rung-8 rise PACE DOSE GRID FULLY
BRACKETED — non-monotonic, peak at slowchain's 1/2-pace, both further
dosing AND more budget FAIL; rung-9 (mint a mesh-native rise ref from
scripted IK) is now the live lever.** Following the pace-confirmed
3-arm triage below, the batch it funded all came back FAIL:
`quarterchain` (pace 1/4, lookahead 0.125s/min_h 4mm) — det 2/6 WORSE
than slowchain's 3/6; the deep-start current/term proxy metrics look
great (median 0.2A, 1/12 terms) but `height_err_end_mm` exposes why:
most flat/rsi episodes drew near-zero current while sitting at
76-86mm height error, UNCHANGED from the belly start — the policy
FROZE rather than pressed up. A cheap way to score well on the
current/term proxy while doing worse on the actual goal — do not
grade this rung on current/terms alone, always cross-check
`height_err_end_mm`. `eighthchain` (pace 1/8) — total collapse, det
0/6 + sto 0/6, even crouch (6/6 robust at every other dose in the
whole rung) breaks; mixes both bad modes (3 over_current + 3 frozen).
`slowchain-cont8` (8M more budget on slowchain's own working
half-pace, std pinned, no re-anneal) — NET WORSE than slowchain
itself (det 3/6→2/6, terms 3/12→7/12): continued low-noise refinement
pushed some deep starts MORE aggressive/hot, not uniformly cooler.
Combined with cont8/reanneal's identical null result on the
unchanged full pace, budget is now settled as NOT a lever on this
rung at ANY pace once `bc_anchor_loss_rise` hits its ~0.05 plateau.
**Consolidated dose-response (DR-0 gate, det+sto valid_plant / 12,
over_current terms /12):** full pace (stdanneal parent) 4/12 valid,
8/12 oc; full pace +8M (cont8) 5/12, 7/12; full pace +reanneal 5/12,
7/12; **1/2 pace (slowchain) 5/12, 3/12 — the peak**; 1/2 pace +8M
(slowchain-cont8) 4/12, 7/12; 1/4 pace (quarterchain) 4/12, 1/12 (but
new freeze failure, not genuine improvement); 1/8 pace (eighthchain)
0/12, 5/12 (collapse). Pace dosing and budget are BOTH exhausted
levers now — do not refund either without a structural change first.
`ppo_goal_cw_standwalk_stance_mesh2_riseonly_bcchain3_slowchain.zip`
is the best rise-in-progress asset from this whole campaign (still
short of PASS). NEXT (rung-9, unfunded — real code, not a config
dose): mint a mesh-native rise reference from scripted/analytic IK
(sit→plant joint trajectory computed directly on the 3.5kg mesh
geometry, not borrowed from the 2.1kg/25Hz primitive extraction) so
the anchor stops supervising a torque-infeasible flat-segment
posture; alternatively, inspect/edit the existing
`rise_ref_belly2plant.npz` flat segment directly (shorten it, or
re-shape the splayed-leg posture the 08-25 dig-in flagged) before
building a full new generator. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_riseonly_bcchain3_{quarterchain,eighthchain,
slowchain_cont8}_gate/`, W&B `75d9j4tg` / `l9tcutsx` / `5bx2x0n3`.)

Prior entry: 2026-08-25 ~15:1x (**rung-8 rise stdanneal 3-arm triage:
PACE IS THE LEVER, not budget or noise — the "mint a mesh-native IK
ref" branch is NOT triggered.** All three arms shared the same 8M
budget off the `riseonly-bcchain3-stdanneal` parent (DR-0 gate 2/6 det
+ 2/6 sto valid_plant, deep-start [flat/bridge/rsi] cur_p95 median
2.64A pinned, 8/12 over_current terms, bc_anchor_loss_rise plateaued
~0.05-0.07): `-cont8` (8M more steps, same chain pace, std pinned -4)
FAIL — det 2/6 sto 3/6, deep-start median STILL 2.64A, anchor loss
0.0515 (at the plateau, no further tracking headroom); `-reanneal`
(fresh 0→-4 noise re-injection, same pace) FAIL — det 1/6 (worse) sto
4/6, deep-start median STILL 2.64A, anchor loss 0.0512 (same
plateau) — noise dose was never the deep-start blocker on this rung
(unlike hold/lower, where std-anneal alone cooled everything).
`-slowchain` (chain pace HALVED from-scratch — `bc_anchor_lookahead_s`
0.5→0.25s, `min_h_ahead_mm` 15→8) PARTIAL, real progress: deep-start
cur_p95 median falls 2.64A→**1.85A**, gate over_current terms fall
8/12→**3/12**, DR-0 det valid_plant 2/6→3/6 — the pre-registered
prediction-if-true landed and prediction-if-false ("equally pinned at
half pace") is refuted. Still short of the 4/6+4/6+≤1.5A PASS bar:
the deep-start plants that DO land are still hot (bridge det
1.85-2.09A), so "unpinned" means below the hard 2.64A ceiling, not yet
cool. Because slowchain (the pace arm) moved the blocked metric while
cont8/reanneal (budget/noise arms, unchanged pace) both independently
did not, the joint-FAIL condition that would force rung-9 (mint a
mesh-native rise ref from scripted IK) is NOT met — pace is confirmed
as a real, working lever, so we dose it further before reaching for a
new reference. FUNDED (batch, from-scratch clones of slowchain's
recipe unless noted): `cw-standwalk-stance-mesh2-riseonly-bcchain3-
quarterchain` (lookahead 0.125s / min_h 4mm — halve again),
`-eighthchain` (lookahead 0.0625s / min_h 2mm — bracket further),
`-slowchain-cont8` (init-from the slowchain checkpoint itself, 8M
more steps at the SAME already-working half-pace, std pinned -4 — is
budget useful GIVEN a working pace, unlike cont8's null result on the
non-working pace). Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_riseonly_bcchain3_{stdanneal_cont8,
stdanneal_reanneal,slowchain}_{gate,owncfg}/`, W&B `i29a19wo` /
`lqav84km` / `8abary62`.)

Prior entry: 2026-08-25 ~15:1x (**Lower seed hedge resolved:
`loweronly-bcchain3-s1` (seed 1, 2M mechanism canary) CANARY PASS.**
DR-0 det 6/6 valid_plant, herr_end 1.2–3.1mm (full commanded drop
band), zero over_current, roll clean; det strips show the same level
six-foot planted descent as seed 0. cur_max 2.25–2.45A ≈ seed-0's
2.17–2.26A hot-crouch band; sto tilt falls at un-annealed std ~1.0
are expected/excluded by this gate and already proven fixable by
stdanneal. Conclusion: the IK-descent BC-anchor-chain lower mechanism
is seed-robust — seed 0 was not a fluke — and the rung stays CLOSED
on the seed-0 stdanneal champion; no further lower funding needed.
Stage-1 tally: HOLD PASS, LOWER PASS (+seed-robust canary), RISE
PARTIAL (pace-dose batch in flight), STANCEMIX PARTIAL (stdanneal +
warmmix arms in flight). Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_loweronly_bcchain3_s1_{gate,owncfg}/`, W&B
`7iczyeb4`.)

Prior entry: 2026-08-25 ~14:5x (**LOWER RUNG CLOSED — `loweronly-bcchain3-
stdanneal` is a FULL PASS, mirroring the hold rung's stdanneal close.**
8M acquisition off the `loweronly-bcchain3` IK-descent BC-anchor-chain
canary (log-std anneal 0→-4.0, final std 0.018): DR-0 det 6/6 + sto
6/6 AND own-DR(0.2) det 6/6 + sto 6/6 valid_plant — ZERO terminations
across all 24 harness episodes, height_err_end_mm 0.0–0.5 (full
commanded 25–55mm drop tracked to sub-mm error, env-native metric),
roll peak ≤0.8°. Current cooled exactly as the canary triage flagged
as the open risk: cur_max 0.66–1.24 A / cur_p95 0.37–0.62 A vs the
canary's hot 2.17–2.26 A crouch (>2x drop) — std-annealing fixed both
the sto-mode fall gap AND the hot-crouch current in one shot, same
mechanism as hold. Gate bar was sto≥4/6 det≥5/6 + zero over_current +
current-vs-canary report — exceeded on every clause (6/6 everywhere).
Honest caveat: the settled crouch reads visually shallower/less
knee-bent on video than the hot canary's version; resolved via the
env-native height-error channel (same code that grades the reward,
already trusted from the canary's independently-video-verified
descent) — read as the cooler policy reaching the same target through
less mechanical strain, not a shallower descent; not escalated as a
gate/video conflict. **New mesh lower champion / stage-2 lower
teacher: `ppo_goal_cw_standwalk_stance_mesh2_loweronly_bcchain3_
stdanneal.zip`** (SKILLS.md row added). Seed-robustness hedge
launched same cycle: `cw-standwalk-stance-mesh2-loweronly-bcchain3-s1`
(seed 1, 2M canary, mirrors the hold rung's `bcanchor3-s1` precedent,
VERIFIED RUNNING train-4). Stage-1 tally: HOLD PASS, LOWER PASS, RISE
PARTIAL (stdanneal cont8/reanneal/slowchain in flight), STANCEMIX
PARTIAL (warmmix1 FINISHED / warmmix2-lowstd RUNNING, triage pending —
both concurrent-cycle-owned). Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_loweronly_bcchain3_stdanneal_{gate,owncfg}/`,
W&B `da1srvqe`.)

Prior entry: 2026-08-25 ~14:5x (**rung-8 `stancemix-bcchain3` composition
read: CANARY PASS (partial) — dilution confirmed, not collapse, and the
dilution is UNEVEN across modes.** Plain English: the full
hold=.1/rise=.45/lower=.45 goal-mix (same per-mode anchor bundle that
PASSED each mode in isolation) does NOT reproduce the old rungs'
rearing/splay mix-collapse, but it does measurably cost quality on two
of three modes vs both the isolated siblings AND this run's own
immediate parent checkpoint: HOLD keeps a real plant (valid_plant=True
6/6 DR-0 det, height_err 8.7mm, roll clean) but runs 3.4x hotter than
its bcanchor3 parent (cur_p95 1.79A vs 0.53A) and now trips
`hold_min_load` in all 6/6 episodes (parent: 0/6) — degraded, not
collapsed. RISE is essentially UNDILUTED: 0/6 valid_plant / 2/6
success, current-pinned on 4/6 — same magnitude AND the same
`bc_anchor_loss_rise` plateau (~0.21-0.27) as the isolated
riseonly-bcchain3 canary at the identical 2M budget. LOWER is badly
diluted: 0/6 success at 17.9-32.4mm height error vs the isolated
loweronly-bcchain3 canary's 6/6 success at 0.1-3.7mm, DESPITE
`bc_anchor_loss_lower` converging just as cleanly in both runs
(0.16->0.011-0.02) — proof that a converged supervised anchor loss
does not guarantee the behavior lands under the competing full-mix RL
objective. Training reward fell every quarter (7.5/-31.8/-88.7/-169.0)
while max_current climbed 0.3A->1.5-1.8A — the 08-21 ruling's
"accumulating hot charges on an active behavior" shape, not a
stuck-flat mechanism. Per the gate's own PARTIAL branch ("compare the
isolated siblings before funding") and the operator's directive to
keep funding the full-mix curriculum, FUNDED the pre-registered 8M
stdanneal continuation (`cw-standwalk-stance-mesh2-stancemix-bcchain3-
stdanneal`, log-std 0->-4.0 anneal-frac 0.5, identical lever to the
hold/rise/lower stdanneal recipes, warm-started from this checkpoint,
VERIFIED RUNNING train-0) — decisive read: anneal cools hold + closes
lower's gap => full-mix stays viable, single-network stage-1 per the
operator's canonical-recipe directive; hold/lower stay diluted/hot
noise-free => the interference is structural (shared capacity fighting
across modes) and stage-1 should fork to stage-2 distillation of the
three isolated champions instead. NOTE: distinct from the operator's
separately-directed raw-18 `footlow2raw18-mesh2-hz100-warmmix1/2`
lineage below (same canonical-recipe family, different lineage/owner —
do not conflate the two "stancemix"-flavored threads). Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_bcchain3_{gate,owncfg}/`,
W&B `tj2k8oxo`.)

Prior entry: 2026-08-25 ~14:4x (**operator kick fb_20260825T140238_d43b35
(footlow2 raw-18 curriculum = binding priority) + rung-8 mix canary
read: `stancemix-bcchain3` CANARY PASS (partial) — the footlow2
anchor bundle prevents the rung-1-6 mix collapse for the first time,
but from-scratch mix buys three HOT half-skills.** DR-0 det: hold
planted (valid_plant 6/6, herr 8.7 mm) yet 6/6 `hold_min_load` trips
~8 s in — load-shedding shuffle (slip 1.6 m) at cur_max 2.62 A vs the
hold champion's 0.44 A; rise first in-mix det successes 2/6 (herr
3–8 mm, 2 over_current); lower det 0/6 (stalls 18–32 mm high) but sto
4/6 — noise dithers it down. Every mode strictly worse than its
isolated sibling at matched 2M (loweronly det was 6/6 clean).
KEY STRUCTURAL FACT vs the footlow2 evidence the operator named: the
primitive footlow2 mix PASSes (`hard1`/`-s1`/`-stable1`) were NEVER
from-scratch — all warm-started an already-competent stance policy;
mix was always HARDENING. So the honest mesh2/100 Hz footlow2 analog
is warm-mix. FUNDED (2M canary pair, VERIFIED RUNNING):
`cw-standwalk-footlow2raw18-mesh2-hz100-warmmix1` (exact mix recipe
init-from the hold stdanneal champion, `--warm-log-std-override=-1.0`,
train-3) and `-warmmix2-lowstd` (same, champion's annealed std ~0.018
kept — separates noise-dose from warm-init; train-2). Gates in the
ledger; mode-isolated rise/lower acquisitions remain the primary
stage-1 path. Canonical-recipe ruling encoded in CURRENT_TRUTHS
"STANDWALK CANONICAL STANCE RECIPE" + Next item 0. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_bcchain3_{gate,owncfg}/`,
W&B `tj2k8oxo`.)

Prior entry: 2026-08-25 ~14:2x (**rung-8 rise stdanneal read:
`riseonly-bcchain3-stdanneal` PARTIAL — std-anneal delivers the FIRST
valid mesh rises but deep starts stay current-pinned.** DR-0 det 2/6 +
sto 2/6 valid_plant, own-DR(0.2) det 1/6 + sto 4/6 (parent: 0/6
everywhere). Structure is by start kind: crouch starts 6/6 valid
across both passes at cur_p95 0.90–0.97 A; the bridge/flat/rsi wins
run 1.92–1.99 A; ALL 13 failures are over_current with current pinned
~2.64 A (one tilt_pitch), mostly with height already reached
(herr<30 mm in 8/13). Video: bridge det = clean level six-foot plant
held to truncation; flat det = hot SPLAYED PRESS-UP (legs radially
extended, sliding) until the trip. FAIL branch dead:
bc_anchor_loss_rise 0.22 -> 0.05–0.07 (converged), slip collapsed
(0.26–1.00 m vs parent 0.66–1.98). Over_current term rate fell
68.7->15.8/chunk and final-quarter reward still rising (-264 -> -48)
=> 08-21 ruling continuation. FUNDED 3-arm batch (all queued/draining):
`-stdanneal-cont8` (8M more from ckpt, std pinned -4 — pure
refinement), `-stdanneal-reanneal` (fresh 0->-4 anneal from ckpt —
noise re-injection to escape the press-up basin, hold precedent),
`-slowchain` (from-scratch stdanneal recipe with
bc_anchor_lookahead_s 0.5->0.25 + min_h_ahead 15->8 mm — is chain
pace the torque driver?). Joint FAIL of cont8+slowchain = flat
segment of the 25 Hz primitive rise ref is torque-infeasible on
3.5 kg -> rung-9 lever is minting a mesh-native rise ref from
scripted IK. Also this cycle: watcher SUSPECT on
`loweronly-bcchain3-stdanneal` was a FALSE ALARM — budget-complete
clean exit at 8,060,928 steps, W&B synced 14:13; triage on its
pre-staged evals belongs to the next cycle. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_stdanneal_{gate,owncfg}/`,
W&B `0z7x5jk2`.)

Prior entry: 2026-08-25 ~13:5x (**rung-8 lower canary read:
`loweronly-bcchain3` CANARY PASS — the IK-descent BC-anchor chain
(`bc_anchor_lower=1`, coef 3.0) produces the FIRST honest sit-down on
the 3.5 kg mesh model.** DR-0 det 6/6 honest descents: full commanded
drop (height_err_end 0.1–3.7 mm, >>60 % bar), zero terminations, zero
over_current, roll clean (peak <=1.0 deg); det video = level six-foot
descent from plant to crouch, feet grounded throughout, held to
truncation. Own-DR(0.2) det also 6/6. Honest caveats: (1) sto 0/6 at
DR-0 AND own-DR, all fell (tilt/over_current) — the exact un-annealed
policy_std~1.0 signature the hold rung had before stdanneal took sto
0/6->6/6 by anneal alone; (2) the crouch is HOT as the 08-25 dig-in
predicted (det cur_max 2.17–2.26 A, cur_s_above_soft up to 10.2 s, no
over_current term) — watch whether anneal cools it like hold
(0.53->0.44 A); `goal.lower_height_mm` belly-rest recalibration stays
the fallback if acquisition can't. FUNDED the pre-registered
follow-up: `cw-standwalk-stance-mesh2-loweronly-bcchain3-stdanneal`
(8M acquisition, warm-start from the canary ckpt, log-std 0->-4.0
anneal-frac 0.5, recipe otherwise unchanged, VERIFIED RUNNING
train-0), mirroring the hold/rise stdanneal recipe — all three
mode-isolated stdanneal acquisitions now in flight (hold PASSED,
rise + lower running). `stancemix-bcchain3` (goal-mix composition
read) still training — its triage decides whether stage-1 unifies via
goal-mix or via stage-2 distillation of the isolated champions.
Watcher SUSPECT on loweronly-bcchain3 was a false alarm
(budget-complete clean exit at 2.03M, W&B synced). Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_loweronly_bcchain3_{gate,owncfg}/`,
W&B `kysdqcu1`.)

Prior entry: 2026-08-25 ~13:5x (**rung-8 rise canary read:
`riseonly-bcchain3` CANARY PASS (partial) — the state-aligned BC-anchor
chain DRIVES THE RISE on the 3.5 kg mesh model, first belly->stand in
eight rungs, but the terminal plant fails on quality.** DR-0 det: 0/6
valid_plant, yet 5/6 episodes visibly rise to a level six-foot standing
posture (height_err_end 4.5–18.9 mm vs the 40 mm hover / 79 mm
belly-freeze of rungs 1–7); failures are current pinned at the 2.64 A
actuator max (p95 1.3–2.6 A vs the 1.5 A bar), footprint creep (slip
0.66–1.98 m/15 s), 1/6 press-up freeze, 1 over_current term per pass.
Mechanism diagnosis from W&B: `bc_anchor_loss_hold` converged
(0.16->0.013, matching the hold passer) but `bc_anchor_loss_rise`
PLATEAUED at ~0.22 (hold reached 0.003) — under un-annealed
policy_std~1.0 the policy tracks the moving chain only loosely, the
exact imprecision signature log-std annealing alone fixed on the hold
rung (sto 0/6->6/6, 2.64 A->0.5 A). NOT the refuted branch (no
belly-freeze majority). FUNDED the pre-registered follow-up:
`cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal` (8M
acquisition, log-std 0->-4.0 anneal-frac 0.5, recipe otherwise
unchanged, VERIFIED RUNNING train-1). Its FAIL branch names the next
lever: mesh-native rise ref minted from scripted IK, not dose.
`loweronly-bcchain3` (lower rung mirror) still training — triage on
finish. Watcher SUSPECT on riseonly-bcchain3 was a false alarm
(budget-complete clean exit at 2.03M, W&B synced). Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_{gate,owncfg}/`,
W&B `lnqbzq5g`.)

Prior entry: 2026-08-25 ~13:1x (**HOLD RUNG CLOSED — `bcanchor3-stdanneal`
(8M acquisition, dose 3.0 + log-std anneal 0->-4.0, final std 0.018)
is a FULL PASS: DR-0 det 6/6 + sto 6/6 AND own-DR(0.2) det 6/6 + sto
6/6 valid_plant, ZERO terminations in all 24 episodes, cur_p95
0.44-0.69A, height_err_end 0.1-0.4mm; det+sto videos = level six-foot
plant, motionless to truncation.** Gate bar was sto>=4/6 with det>=5/6
— exceeded everywhere. This settles the canaries' det/sto gap:
un-annealed policy_std~1.0 was the entire sto blocker (annealing alone
took sto 0/6 -> 6/6 with no other change). Seed hedge also landed:
`bcanchor3-s1` (seed 1, 2M canary) CANARY PASS — DR-0 det 6/6
valid_plant @0.46A + own-DR det 6/6, same clean plant on video, same
benign un-annealed-sto signature; the BC-anchor mechanism is
seed-robust AND dose-robust. **New mesh hold champion / stage-2 hold
teacher: `ppo_goal_cw_standwalk_stance_mesh2_holdminload40_bcanchor3_stdanneal.zip`**
(SKILLS.md row added). NEXT RUNG = rise/lower on mesh via the same
proven lever (BC-anchor chains toward the rise ref / plant pose),
warm-started from the hold champion (mesh-family, so warm-start is
legal and default). SPECIFICATION DEBT first: `test_bc_anchor.py`'s 3
rise/lower chain tests (`test_state_aligned_chain_climbs_to_the_plant`,
`test_lower_anchor_chain_descends_with_feet_planted`,
`test_min_h_ahead_unpins_the_plateau_traversal`) are RED — pinned to
primitive-era heights (primitive plant h_rel 131.94mm vs mesh 82.96mm)
— and per RESEARCH_RULES the bank must be green before the mechanism
launch. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_holdminload40_bcanchor3_stdanneal_{gate,owncfg}/`,
`..._bcanchor3_s1_{gate,owncfg}/`, W&B `bmoh247p`/`q1xfw1ik`.)

Prior entry: 2026-08-25 ~12:5x (**rung-7 dose read COMPLETE (3/3),
FINAL: `bcanchor0p5` (dose 0.5) is a CANARY FAIL - MECHANISM (dose
too low), not a third passer — settled after two independent reads
converged (this cycle's own first pass wrongly called it PASS on a
"held to truncation" misreading of the video; self-corrected; a
concurrent cycle independently reached FAIL from the same evidence).**
DR-0 det: `valid_plant`=True and `cur_p95`<=1.5A on 6/6 episodes
(clears the gate's literal numeric bar) BUT every one of those 6
episodes TERMINATES EARLY via `hold_min_load` at t~3.5s (return
~57.8, not the full 15s) — i.e. even under pure deterministic policy
execution with zero action noise, one foot's load fraction drops
below the safety floor within seconds; `valid_plant` reads true only
because it grades geometry/height/current at the termination
instant, not sustained load balance. Contrast with `bcanchor1`
(dose 1.0) / `bcanchor3` (dose 3.0): both hold the FULL 15s with
ZERO terminations in DR-0 det — a materially different, genuinely
sustained result. Own-DR(0.2) det: 5/6 valid_plant but only 1 of 6
episodes runs to truncation without any termination; sto 0/6 at
every dose (same un-annealed-`policy_std` signature). Reading: the
BC-anchor mechanism itself is confirmed real (a genuine six-foot
plant is reachable, unlike any of the six prior pure-pricing rungs)
but 0.5x sits below the dose floor needed to sustain it even without
noise — 1.0x and 3.0x are the true passers, dose-response has a
floor around 0.5-1.0x. **No dose-axis action needed regardless**:
the concurrent cycle already launched the pre-registered ONE 8M
acquisition arm off dose 3.0 (the cleanest of the three,
`cw-standwalk-stance-mesh2-holdminload40-bcanchor3-stdanneal`,
log-std 0->-4.0, running on train-1) before either read landed —
still the right pick. NEXT: triage `bcanchor3-stdanneal` when it
finishes (gate: DR-0 det AND sto >=5/6 valid_plant, cur_p95<=1.5A,
zero hold_min_load in det) — that answers whether std-annealing
closes the sto robustness gap and stage-1 hold can move to
rise/lower. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_holdminload40_bcanchor0p5_{gate,owncfg}/`,
W&B `vggrigsq`.)

Prior entry: 2026-08-25 ~12:4x (**rung-7 second read: `bcanchor1`
(dose 1.0) ALSO CANARY PASS — the BC-anchor result replicates across
doses.** `cw-standwalk-stance-mesh2-holdminload40-bcanchor1` DR-0 det
gate 6/6 valid_plant, cur_p95 0.75A, zero terminations, det video =
level six-foot plant held motionless; height_err 12.5->4.8mm over 2M,
bc_anchor_loss 0.164->0.005. Same honest caveat as bcanchor3: DR-0
sto 0/6, all `hold_min_load` — under std~1.0 action noise the stance
creeps (~167mm drag) until a foot unloads; robustness is the 8M
acquisition question, not a canary fault. Dose read so far: 3.0 PASS,
1.0 PASS (3.0 slightly cleaner det: cur_p95 0.53A vs 0.75A,
height_err_end 0.7mm vs ~4.8mm, plus own-DR det 6/6); 0.5 pending —
the bcanchor0p5 cycle owns the final pick + ONE 8M acquisition
launch. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_holdminload40_bcanchor1_gate/`,
W&B `0q4hurrk`.)

Prior entry: 2026-08-25 ~12:3x (**rung-7 first read: BC pose-anchor
BREAKS the 40mm hover basin — `bcanchor3` (dose 3.0) CANARY PASS, the
first honest six-foot plant on the mesh model in seven rungs.**
Plain English: adding a supervised pull toward the episode's own
settled plant pose did what six rungs of reward/termination shaping
could not — the robot now stands level on all six loaded feet.
`cw-standwalk-stance-mesh2-holdminload40-bcanchor3`
(`train.bc_anchor_coef=3.0`, 2M canary): DR-0 det gate **6/6
valid_plant** (height_err_end 0.7mm vs the parents' pinned 40mm,
cur_p95 0.53A, cur_max 1.1A vs the pinned 2.64A, roll clean, all six
end_clear ~0mm, plant_margin 149.5mm); own-DR(0.2) det ALSO 6/6 with
zero terminations; det video = level chassis, six loaded feet, held to
truncation. `bc_anchor_loss` 0.08->0.003. Honest caveat: DR-0 sto 0/6,
all `hold_min_load` — but `policy_std` is still 1.019 at 2M (never
annealed), so sto eval draws ~full-init noise on a clean mean; the
gate's "rising hold_feet_factor" sub-criterion (0.27->0.20, not
risen) reads on those same std~1.0 training rollouts and is
superseded by the det harness it proxies. Training reward still
declines (-2.9/-57.9/-91.2/-86.2; anchor is a supervised loss outside
the reward — expected at canary scope).
UPDATE (~12:5x, same cycle): the joint dose read completed while this
entry was being written — ALL THREE doses CANARY PASS (`bcanchor1`
6/6 det @0.75A, `bcanchor0p5` 6/6 det + own-DR det 5/6, `bcanchor3`
6/6 det; dose-INSENSITIVE across the 6x range 0.5-3.0), and the
concurrent cycle funded the 8M acquisition
`cw-standwalk-stance-mesh2-holdminload40-bcanchor3-stdanneal`
(dose 3.0 + log-std anneal to -4.0 per the joystick stotight45
precedent; gate = sto >=4/6 valid_plant at DR-0 with det >=5/6
preserved) — that arm answers the det/sto gap question directly.
CAVEAT this cycle hedged: all three dose arms used the DEFAULT seed
(identical network init), so dose-insensitivity is proven but
seed-robustness is NOT — launched `...-bcanchor3-s1` (seed 1, 2M
canary, VERIFIED RUNNING train-2) as the cheap hedge read; if
stdanneal disappoints, its verdict tells us whether the anchor
mechanism generalizes across inits or rode one lucky seed.
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_holdminload40_bcanchor3_{gate,owncfg}/`,
W&B `80jrhio3`.)

Previous entry: 2026-08-25 ~12:1x (**rung-6 (`holdminload40`) CLOSED,
2/2 seeds FAIL — min-load termination alone still can't break the
chassis-hover-at-40mm basin; rung-7 (BC-anchor pose imitation) LAUNCHED
as a 2M mechanism-health canary.** Plain English: `cw-standwalk-
stance-mesh2-holdminload40` (seed 0) and `-s1` (seed 1) both finished
6M with the identical failure signature rung-5 (`holdterm40`) already
showed, now with the min-load lever also firing: DR-0 det gate 0/6
valid_plant both seeds, every episode `TERM hold_low_height` at
`height_err_end_mm` pinned 40.0-40.4mm (right at the drop line) with
`cur_max_a` pinned ~2.64A (height AND current constraints both
binding at once); own-DR(0.2) sto shows the new `hold_min_load` reason
firing 3/6 times each seed (mechanism verified LIVE — it correctly
catches the previously-invisible unloaded-foot cheat) plus the
remainder split tilt_roll/hold_low_height. Training reward WORSENED
every quarter both seeds (seed0 -97.8/-223.6/-347.6/-355.1; seed1
-95.9/-237.6/-309.0/-264.8) — genuine FAIL per the 08-21 ruling
(bank-proven aligned mechanism, task metric flat-zero the whole 6M).
Video: crouched crab-like stance, legs bent under a lowered chassis,
never a level six-foot plant. CONCLUSION: pure income/termination
shaping is now closed as a lever across SIX rungs (pricing grid,
income-gradient variants x2, height-drop term, min-load term) — the
missing ingredient is not a better penalty/gate, it's a signal for
WHAT the honest target posture actually looks like.
RUNG-7 (launched, single lever, 2M canary): `train.bc_anchor_coef=1.0`
on the holdminload40 recipe unchanged — the already-built HOLD/TRACK
BC-anchor machinery (`sim_env.py`, `bc_anchor.py`, built 08-11 for the
primitive-era `footlow2` champion, hold-scoped bank tests already
green: `test_bc_anchor.py -k hold` 3/3) supervises the policy's mean
action toward `self._q_nom` — the settled six-foot plant pose captured
at episode reset (these are `start_kind=plant` hold episodes, so
`_q_nom` literally IS the honest target, not a proxy) — every tick,
independent of the RL reward. This is the same lever `footlow2` used
successfully, previously assumed off-limits on mesh only because that
prior recipe ALSO warm-started a (now cross-family-incompatible)
primitive checkpoint; the anchor target itself (`_q_nom`) needs no
warm start, so it is usable from-scratch. `cw-standwalk-stance-mesh2-
holdminload40-bcanchor1` VERIFIED launching on train-0, 2M steps,
MECHANISM-HEALTH CANARY gate only (>=4/6 det valid_plant + rising
hold_feet_factor/hold_load_factor = fund an 8M continuation; 0/6 with
the same 40mm/2.64A pinned signature = bc_anchor itself refuted for
this task, escalate to operator re: accepting the primitive-family
footlow2 champion as an interim teacher). Second seed and/or longer
budget deliberately withheld until this canary reads — six of six
prior rungs converged to the identical basin, so a single cheap read
is the right amount of commitment for an unproven 7th lever.
Side note: `test_bc_anchor.py` has 3 PRE-EXISTING RED tests unrelated
to hold (`test_state_aligned_chain_climbs_to_the_plant`,
`test_lower_anchor_chain_descends_with_feet_planted`,
`test_min_h_ahead_unpins_the_plateau_traversal` — all rise/lower
floor-climb assertions pinned to primitive-era height numbers, likely
stale against the mesh mass/height defaults) — not touched by or
relevant to this diff, flagged for whichever cycle next works rise/
lower floor-climb BC chains.
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_holdminload40{,_s1}_{gate,owncfg}/`,
W&B `jbnbyrdm`/`cvacvokp`.)

Previous entry: 2026-08-25 ~10:2x (**rung-5 (`holdterm40`) VERDICTED
FAIL; rung-6 (min-load termination) landed, bank-proven, and LAUNCHED
as a 2-seed batch.** `cw-standwalk-stance-mesh2-holdterm40`'s gate
report confirms rung-5's own pre-registered "alternative cheat" bit
for bit: every det+sto episode (0/12 valid_plant) ends with
`height_err_end_mm` pinned at **exactly 40.0-40.3 mm** (right at/on
the drop line, not a collapse), `cur_max` ~2.6 A, leg imbalance
1.8-1.9, one foot `end_clear` up to 26 mm while the rest sit ~0 —
the policy learned to hover its CHASSIS at the boundary rather than
re-plant. W&B: `env/hold_feet_factor` collapsed 0.96->~0.11 by 200k
and never recovered above ~0.17 across the full 6M run (flat, not
rising); reward quarters -107/-335/-538/-466 (Q3 worse than Q2, a
late partial recomposition from a `hold_low_height` termination spike
in the last ~600k steps that still ends 0/6 valid) — a genuine FAIL
per the 08-21 ruling (task metric flat the whole run), not a continue
case. `-s1` (seed twin) finished around the same time but belongs to
another cycle's read — not incorporated here.
ROOT CAUSE: `hold_low_height` only sees CHASSIS height; a foot that
stays functionally unloaded (or is forced to carry an unfair share)
while the body sits inside the 40 mm line is invisible to it — the
exact hover/hover1 crouch-park class this file already had scripted
twins for (`HOLD_LOAD_OVERRIDES` section), now shown to survive to
truncation even with the height-drop lever ON.
RUNG-6 LEVER (landed, default-off, bit-exact when off; snapshot
`17554cea`): `safety.hold_min_load_terminate_{s,n,grace_s,tau_s}` —
terminates a hold episode when the WORST (min-over-feet) touch force,
EMA-smoothed, stays below `hold_min_load_terminate_n` for
`hold_min_load_terminate_s` seconds (reason `hold_min_load`),
independent of body height. Universal per-episode state
(`_hold_minload_ema`/`_hold_minload_low_s`) lives in the SHARED base
class (`sim_env._reset_finalize` + `mjx_host.SNAP_ATTRS`), not the
walk-only subclass, because hold mode is available on every task
class (the walk-only `_walk_qvel_ema`/`_walk_idle_low_s` pattern this
mirrors is walk_task-scoped and would silently AttributeError for
plain `joint_goal` hold-mode training — verified this was the fix
needed after the first draft crashed exactly that way).
Bank (`test_hold_minload_*`, `test_task_semantics.py`, 4/4 green):
(a) default-off bit-exact, (b) documents the loophole — TODAY'S LIVE
STACK (height-drop lever already on) really is blind to the scripted
hover/hover1 poses (they survive to truncation, un-terminated), (c)
the new lever terminates both hover classes with reason
`hold_min_load` inside a few seconds, (d) the honest quiet stand is
byte-identical untaxed.
Launched: `cw-standwalk-stance-mesh2-holdminload40` (seed 0) +
`-s1` (seed 1), 6M each, holdterm40's exact recipe + the new lever
(`hold_min_load_terminate_n=0.3`, `_s=1.0`, `_grace_s=1.0`), VERIFIED
RUNNING train-0/1. Gate: same hold panel as rung-5, now also requiring
zero `hold_min_load` terminations.
Full regression note: ran `test_task_semantics.py -k hold` (39 tests,
10 pre-existing red — same named set STATUS already flagged, unrelated
to this diff) and `-k "not hold and not walkcurr"` (162 tests, 15 red,
confirmed pre-existing via git-stash A/B on 3 samples — the
control.hz=100 tick-count regression already tracked elsewhere in
OPERATOR_QUESTIONS) — zero NEW failures from this cycle's diff.

Previous entry: 2026-08-25 ~09:5x (**rung-5 LAUNCHED: basin-exit
termination landed, bank-proven, and running as a 2-seed batch.**
The mechanism the closure below calls for executed same-cycle:
`safety.hold_max_height_drop_mm` in sim_env (`hold_low_height` —
walk_low_height's hold-mode twin, default-off bit-exact, snapshot
`9b69378c`) + the HOLD_BASIN_TERM semantics bank, 5/5 green on
primitive AND mesh (scripted all-hips-up belly flop: the launch stack
is provably blind without the lever; with it the flop terminates <4 s
with reason hold_low_height, the honest quiet stand is byte-identical,
and return is strictly MONOTONE in time-in-basin — the slope
min/product income provably lack). `cw-standwalk-stance-mesh2-
holdterm40` + `-s1` (seed twin) VERIFIED RUNNING train-0/1:
holdload1min scratch recipe + ONLY the two safety keys (40 mm drop,
1 s grace), 6M each; hold-panel gate now also requires zero
hold_low_height terms. Named watch axis: crouch-hover just above the
40 mm line (h_err 30-39 mm, no valid plant) = FAIL; pre-registered
follow-up for that or a pinned-termination stall is a MIN-LOAD exit
trigger (terminate when min foot load stays floor-pinned), bank
first. SIDE NOTE: 10 pre-existing RED tests in test_task_semantics on
pristine main (2 hold-bank + 8 walkcurr ranking banks; verified via
git-stash A/B, NOT from this cycle's diffs — OPERATOR_QUESTIONS 08-25
~09:5x): any launch citing those specific banks must re-run them
first.)

Previous entry: 2026-08-25 ~09:3x (**PRODUCT CLOSURE COMPLETE: the
remaining three product-gradient cells `holdprod-f01-s1`/`-f03`/
`-f03-s1` all VERDICTED FAIL — the 2x2 grid (floor 0.1/0.3 x seed 0/1)
is 4/4 dead, and with load-min 5/5 dead plus the noise pair (-dr0,
-ent4) dead, EVERY income-shaping and optimizer-side lever for the
hold rung is now closed. Rung-5 (mechanism branch) is unblocked.**
Plain English: paying each returning foot its own graded income
(product gate) doesn't work at either floor or seed — PPO still walks
out of the six-foot plant it starts in and settles into a quiet freeze
it never leaves. The three new reads: f01-s1 = belly-flop freeze
(h_err 70 mm, all feet ~40 mm clear, cur_p95 0.53 A, ZERO
terminations, 0/24 valid both DRs); f03-s1 = same belly freeze; f03 =
a flag-leg belly-freeze variant (one leg pinned duty 1.0 /
end_clear -0.3 mm, one waved aloft 150-200 mm, cur_p95 1.2-2.0 A,
sto rides above soft-current for up to 12 s — and still never
terminates). MECHANISM TRACE, identical in all four cells:
`env/hold_feet_factor` starts 0.96-0.97 at 65k (the policy IS in the
paying basin) and collapses below 0.15 by ~200k (f01: by 786k);
post-1M max never exceeds 0.23; reward monotone-worse all run — not
08-21 continue cases. JOINT READ (pre-registered): income gradient
shape (min vs product, floor 0.1 vs 0.3) and exploration noise (DR-0,
4x entropy) are ALL insufficient because every failure basin is
QUIET — zero terminations, low current — so out-of-basin data
dominates after ~0.2-0.8M steps and nothing ever prices or resets
the excursion. Rung-5 per the ent4/f01 pre-registrations: hold-mode
EARLY TERMINATION on sustained basin exit (h_err/plant loss sustained
-> priced termination, episodes reset back into the paying basin),
semantics-bank proof BEFORE launch. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_holdprod_{f01_s1,f03,f03_s1}_{gate,owncfg}/`,
W&B `n02jq4xp`/`4qcs74q2`/`0oy291ud`.)

Previous entry: 2026-08-25 ~09:2x (**noise axis CLOSED: `holdload1min-ent4`
VERDICTED FAIL — 4x entropy does not rescue the load-min recipe, and
with the dr0 sibling's FAIL the rung-4 optimization pair is jointly
dead. Load-min income gate now 5/5 arms dead across 4 distinct
basins.** Plain English: ent4 bet that the belly-flop freeze was
premature variance collapse; the lever mechanically fired (train/std
held at 1.68 the whole run vs ~0.4 typical collapse) yet the policy
still dropped from its planted start onto its belly by t~2s and froze
— 0/24 valid hold (DR-0 + own-DR 0.2, det+sto), zero terminations,
h_err ~70 mm every episode, hold_load_factor pinned at the 0.1 min
floor, reward declining -99 -> -708 (not an 08-21 continue case).
KEY READ: sustained random action noise never re-plants six feet
simultaneously, so min-over-feet income sits at its floor with no
gradient home — same lesson as the product-f01 trace (in-basin at
65k, out by 786k, never returns). Joint pre-registered read
(dr0 FAIL tilt-topple + ent4 FAIL belly-freeze): optimizer-side
levers (noise up, noise down) are BOTH closed; rung-5 is the
hold-curriculum / pose-anchor MECHANISM branch, bank work first.
The strongest bank-work candidate named by this evidence: hold-mode
early termination on basin exit (h_err/plant loss sustained -> priced
termination + reset back into the paying basin), because every failure
basin SURVIVES quietly today (belly-freeze: 0 terms, cur 0.5 A) and
episodes spend ~all data out-of-basin after the first 0.7M steps.
No rung-5 launch until the product closure (f01-s1/f03/f03-s1, other
cycles' reads) is in, per the f01 pre-registration. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_holdload1min_ent4_{gate,owncfg}/`,
W&B `39kxul3i`.)

Previous entry: 2026-08-25 ~09:1x (**rung-4 product-gradient wave, first
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

**RISE, 08-25 ~17:4x — rung-9 8M seed-0 TWINS read: budget lever is
likely NULL (within same-seed noise of the 2M canary); targeted
flat-mix canary LAUNCHED.** `meshref-acq8m` PARTIAL by its gate
letter (DR-0 det 5/6 + sto 5/6 valid_plant, oc 2/12, valid-ep
cur_p95 median 0.81A vs canary 5/6+4/6, 3/12) — BUT the dup-killed
`meshref-8m` turns out to have FINISHED its full 8M budget
(max global_step 8,060,928 @~17:14, before the 17:19 kill landed;
earlier "~5M" reads were an out-of-order W&B video row) and its
pre-staged eval scored EXACTLY canary level (5/6+4/6, 3/12 oc).
Same seed + same config + same budget on a different pod => the
twin delta (±1 episode, ±1 oc term at n=12) IS this harness's
run-to-run noise band, and acq8m's nominal edge over the 2M canary
sits inside it. Robust across both 8M replicates: flat det/0 is the
identical never-tucks splayed press-up (2.64A pin, h_err_end
24-28mm, video-confirmed) at 2M/8M/8M — budget-invariant,
structural; successes stay clean (bridge sprawl→level six-foot
plant by ~3s, tilt 0.3°). Grid still judged jointly when s1/s2
triage lands (both finished training ~17:4x, next cycle owns them),
but with two seed-0 8M replicates at/within-noise-of canary levels
expect the pre-registered FAIL route. ACTED on acq8m's own PARTIAL
clause ("fund one targeted arm at the named residual subclass"):
`cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-flatmix70`
VERIFIED RUNNING train-0 — 2M canary, exact meshref recipe, single
config lever `goal.rise_flat_frac` 0.35→0.70 / `rise_partial_frac`
0.40→0.20 (flat exposure ~17.5%→~35%; mechanism already existed in
config, no code). Its gate's primary evidence is a FLAT-PINNED pod
probe (eval `--cfg-set goal.rise_flat_frac=1.0 rise_partial_frac=0
rise_rsi_frac=0`, det+sto 6+6) since the standard gate draws only
1-2 flat episodes — read deltas against the twin noise band. On
flatmix FAIL: exposure refuted → ref-content/phase treatment
(tuck-phase anchor dose or tuck-segment curriculum) is the next
rung. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_{acq8m,8m}_gate/`,
W&B 08k9lmkm/ul88m0v3.

Prior entry (**RISE, 08-25 ~17:0x:** rung-9 mesh-native ref canary
pair = CANARY PASS (PARTIAL-strong, see Last-updated entry); 3-seed
8M acquisition grid `meshref-acq8m`(s0)/`meshref-8m-s1`/`-8m-s2`
RUNNING. On grid PASS (>=2/3 seeds at the full bar): promote the
best seed as the mesh rise recipe and re-run the stancemix mix with
`reward.rise_ref_path=rise_ref_mesh_scripted.npz`. On grid FAIL at
canary levels: budget is refuted for the flat-start tuck residue;
next rung is a targeted tuck mechanism (start-mix weighting toward
flat starts, or tuck-phase anchor dose) — never more undirected
budget.)

**STANCEMIX-BCCHAIN3-STDANNEAL PASSED its pre-registered DR-0 gate
(08-25 ~15:5x):** the full hold=.1/rise=.45/lower=.45 mix + log-std
anneal (0→-4 over 8M, warm-started off the 2M stancemix-bcchain3
canary) hits all 3 pre-registered clauses — hold det 6/6 + sto 6/6
valid_plant, ZERO terms, cur_p95 0.78A det / 0.81-0.92A sto (cooled
from the canary's hot 2.62A hold_min_load trips); lower det 6/6 +
sto 6/6, height_err_end 0.0-1.4mm (up from the canary's 0/6 success
at 18-32mm — the mode called "severely diluted" is now AT the
isolated lower champion's own band); rise det 2/6 valid_plant (meets
the >=2/6 bar; the 4 failures are the SAME universal deep-start
over_current/tilt_pitch press-up pinned at 2.33-2.64A the rung-8
pace-dose grid (below) is independently attacking, not a new
mix-specific failure). No mode regressed vs the 2M parent canary —
**the operator's full-mix-curriculum directive is vindicated for
hold+lower; rise remains capped by the campaign-wide deep-start
blocker.** Own-DR(0.2) is weaker (hold sto 4/6, rise 0/6) — flagged,
not disqualifying, same universal-blocker signature.
`logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_bcchain3_stdanneal_{gate,owncfg}/`.
Refill: since rung-8 (below) found `slowchain`'s 1/2-pace is the
PEAK of the whole dose-response (both further halving AND more
budget FAIL), ported that exact validated dose into the mix —
`cw-standwalk-stance-mesh2-stancemix-bcchain3-slowchain` (8M,
`bc_anchor_lookahead_s` 0.5→0.25s + `min_h_ahead_mm` 15→8, log-std
pinned -4 no re-anneal, warm-started from THIS pass checkpoint so
hold/lower keep their solved weights) VERIFIED RUNNING on train-0 —
tests whether the isolated-proven pace fix transfers into the
shared-capacity mix without disturbing hold/lower.

RUNG-8 tally (08-25 ~16:1x, concurrent cycle): HOLD PASS, LOWER PASS
(both stdanneal), RISE dose-response now FULLY BRACKETED and
non-monotonic — peak at `slowchain`'s 1/2-pace (5/12 valid, 3/12 oc);
further halving (quarterchain 1/4, eighthchain 1/8) and more budget
(cont8, slowchain-cont8) all FAIL/regress. Pace and budget are BOTH
exhausted levers on this rung now; rung-9 (mint a mesh-native rise
ref from scripted IK, since the borrowed 25Hz/2.1kg primitive ref's
flat segment may be torque-infeasible on the 3.5kg mesh) is the next
live lever if the mix-ported slowchain arm above also caps out.

Stage-1 HOLD is SOLVED (08-25 ~13:1x): mesh hold champion
`ppo_goal_cw_standwalk_stance_mesh2_holdminload40_bcanchor3_stdanneal.zip`
(24/24 valid_plant across DR-0+own-DR det+sto, zero terms — see Last
updated entry). RUNG-8 (rise/lower anchor chains) RUNNING 08-25
~13:3x: the 3 red `test_bc_anchor.py` chain tests are FIXED and green
(root cause was NOT stale primitive heights — the 08-24 config
`control.hz` 25->100 flip quartered the wall-time of their fixed
tick-count loops while chain progress is slew-limited at a
rate-invariant 37.5 deg/s; loops now expressed in seconds via
`env.dt`, bit-identical at 25 Hz, 56/56 green at the 100 Hz default;
bisected to commit 9a2b644c, fix tag
`exp/bcanchor-chain-tests-rate-fix` = 41419aef). Batch (2M mechanism
canaries, footlow2-PASS anchor bundle — state_aligned/lower/foot_z/
stratified/lookahead 0.5s/min_h_ahead 15mm — on the mesh bcanchor3
recipe, coef 3.0, from-scratch; NO warm start from the hold champion:
its std is annealed to 0.018 and the anchor needs no init, proven by
rung-7): `cw-standwalk-stance-mesh2-riseonly-bcchain3` (train-1),
`-loweronly-bcchain3` (train-0), `-stancemix-bcchain3` (train-2, the
hold=.1/rise=.45/lower=.45 composition read vs the rung-1-6 mix
collapse). Gates in the ledger; isolated-vs-mix comparison is the
designed read.

Historical (superseded) entry — stage-1 batch 08-25 ~04:4x (operator kick):
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

0. OPERATOR DIRECTIVE (08-25, fb_20260825T140238_d43b35 — binding
   priority): the raw-18 `joint_goal` **footlow2** curriculum
   (hold=.1,rise=.45,lower=.45 + per-mode anchors + rise-ref) is THE
   canonical rise/lower recipe to get working on mesh2 @ 100 Hz
   before any AMP/transformer/walk-style drift. `stance_dr10` is NOT
   the template (6-ch body-pose). The rung-8 `*-bcchain3` lineage IS
   the mesh2/100 Hz footlow2 analog (same task/mix/anchor bundle,
   mesh-recalibrated heights); triage its arms against the footlow2
   evidence (`hard1`/`hard1-s1`/`stable1`/`plant150-3-rsifix`) and
   keep funding the full-mix curriculum until stage-1 passes the old
   hard1-style gates + current/DR/session gates. Encoded in
   CURRENT_TRUTHS "STANDWALK CANONICAL STANCE RECIPE".
0.5 RUNG-9 (08-25 ~16:1x, unfunded, real code): rung-8's rise
   BC-anchor-chain pace/budget dose grid is now FULLY EXHAUSTED (7
   arms: stdanneal/cont8/reanneal/slowchain/quarterchain/eighthchain/
   slowchain-cont8 — see Last-updated entry for the full table). The
   deep-start (flat/bridge/rsi) rise still tops out around slowchain's
   5/12 valid_plant, 3/12 over_current. Next lever is the REFERENCE
   itself, not more dosing: mint a mesh-native `rise_ref` from
   scripted/analytic IK on the 3.5kg mesh geometry (the current
   `rise_ref_belly2plant.npz` was extracted from a 2.1kg/25Hz
   primitive-family policy — `extract_rise_ref.py`'s own docstring
   flags this as a stopgap), OR inspect+edit the flat segment of the
   existing ref directly (shorten the slow 0-25mm prep crawl / fix the
   splayed-leg posture the 08-25 dig-in named) before building a full
   new generator — cheaper first probe. Whoever picks this up: read
   the full pace-dose table in the Last-updated entry before touching
   `train.bc_anchor_*` again; that lever is spent for this rung.
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
