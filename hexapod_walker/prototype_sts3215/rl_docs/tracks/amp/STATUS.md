# amp - AMP locomotion from scratch

Last updated: 2026-08-23 ~22:0x (**M5 GATE GREEN — 4/4 SEED
REPLICATION COMPLETE; TRACK GOAL MET (sim scope).** `-phasehz11-s23`
and `-s29` both VERDICTED full m5 PASS this cycle (own-cfg suite,
walk wpm24 n_translating=28): s29 walk det_prog_med **1.058** /
det_slip_med **2.883**, yaw tips **0.1109/0.1421** — the strongest
M5 read in program history; s23 prog 0.959 / slip 3.294, tips
0.1398/0.1383; both 0 falls/terms in every section, gait 48/48 walk
+ 12/12 push + >=11/12 fault, walk det strips watched (upright,
level, six legs cycling, no flag leg). With seed-7 (base) and s17
(concurrent cycle) the pre-registered 4-seed pass-rate gate reads
**4/4** — the 1.1 Hz cadence sweet spot is recipe-level, not seed
luck. **M5 champion: `ppo_goal_..._phasehz11_s29.zip`** (best prog
AND tips of the four passers; champions append-only; SKILLS.md row
added). Per `rl_docs/AMP_LOCOMOTION.md` the track is DONE at M5; M6
(hardware transfer) is operator-owned.

## WAITING-ON

- `[operator]` M6 hardware transfer of the M5 champion
  (`..._phasehz11_s29.zip`, dialect: walk_phase_obs 1.1 Hz,
  obs_body_vel=2, fault_health obs) — physical-robot step, out of
  agent scope. Sim-side amp work is COMPLETE unless the operator
  re-opens (e.g. orders an M5 margin-hardening pass or a
  cross-engine robustness sweep). Secondary open read (not blocking,
  other cycle): `-phasehz05-cont1`/`cont2` 0.5 Hz continuation —
  only relevant if it someday jointly beats 1.1's prog+slip.

Previous banner (~21:4x (**SEED 2/4 CONFIRMS — `-phasehz11-s17`
VERDICTED PASS, full m5 green on an independent seed with BETTER
margins than the base: walk det_prog_med 0.983 / det_slip_med 2.661
(wpm24, n_translating=28, bars 0.75/3.5), yaw tips 0.1141/0.1655
(bar 0.20), 0 falls/terms anywhere, gait 48/48 walk + 12/12 push +
11/12 fault, strips clean (level body, six-leg cycling, fault leg
held out limp while five cycle). 1.1 Hz pass-rate now 2/4 (seed7
base + s17); `-s23`/`-s29` (still training) decide the >=3/4
sweet-spot branch that flips the track M5 gate. Note for future
triage: the off-distribution DR-0 quick-gate report looks bad on
this lineage (s17: det slip 3.95 there vs 2.66 on m5; base: 7.08 vs
3.13) — judge on the m5 suite only. Evidence:
`logs/ckpt_eval/..._phasehz11_s17_m5/`. Prior banner below.)

Previous entry (~21:2x (**FIRST FULL amp-m5-v1 PASS —
1.1 Hz is the cadence sweet spot; 3-seed replication batch RUNNING
before the track M5 gate flips.** The dose curve is complete and
NON-monotone: `-phasehz11` (goal.walk_phase_hz 1.33->1.1, single
lever) VERDICTED **PASS** — all four m5 sections green for the first
time in program history: yaw tips **0.1451/0.1453** (bar 0.20,
parent 0.2157/0.2351, 0 falls), walk (wpm24, n_translating=28)
det_prog_med **0.893** / det_slip_med **3.131** (bars 0.75/3.5),
0 terms, gait 48/48 walk + 12/12 push + 11/12 fault, strips watched
(walk/push/fault: upright, level, six legs cycling, no flag leg).
`-phasehz09` VERDICTED PARTIAL: tips 0.1313/0.1801 pass but walk
prog 0.642 / slip 4.739 (wpm24) — with the concurrent 0.7 read
(prog 0.674/slip 4.48) the 0.7-0.9 band is a WALK TROUGH (worst of
both worlds) while tips recover at every dose below 1.333; the
feasible region for BOTH skills opens at ~1.1 Hz, i.e. the parent's
tip saturation was a mild cadence ceiling needing only a 17%
slowdown, not the 0.5 Hz sacrifice. Checkpoint
`ppo_goal_..._phasehz11.zip` is the **M5 candidate champion**
(SKILLS.md row added). LAUNCHED (this cycle, all VERIFIED RUNNING,
2M discovery each): `-phasehz11-s17/-s23/-s29` — exact-recipe seed
twins; joint 4-seed pass-rate gate (with seed-7 original): >=3/4
PASS = recipe-level confirmation, M5 flips; 2/4 fragile ->
continuations; <=1/4 seed luck. Still training elsewhere:
`-phasehz05-cont1` (does 0.5 Hz recover prog with +4M — now a
secondary question unless it beats 1.1's slip/prog jointly).
Assumption recorded (OPERATOR_QUESTIONS.md q_20260823T2110Z): M5
declaration requires the replication read, not just one seed.
Evidence: `logs/ckpt_eval/..._phasehz{09,11}_m5{,_wpm24}/`. Prior
banner below.)

**UPDATE ~21:2x-3x (concurrent cycle): `-phasehz05-cont1` VERDICTED
PARTIAL** — +4M at 0.5 Hz grows walk det_prog_med 0.52 -> **0.669**
(wpm24 n=28) while IMPROVING tips to **0.1146/0.1048** (family best;
erosion alternative refuted at one dose) and cutting slip to a
lineage-best **2.31**; push PASS, fault first-ever spotless (0
terms, gv 12/12); m5_pass=false only on walk prog; reward still
rising at cutoff (272.6 last quarter). Against phasehz11's PASS
(prog 0.893/slip 3.13/tips 0.145): 0.5 Hz is the WIDE-MARGIN corner
(tips/slip) that is still prog-limited, 1.1 Hz is the prog corner
with thinner slip margin (3.13 vs 3.5 bar). `-phasehz05-cont2`
LAUNCHED (+4M, acquisition): if prog reaches >=0.75 at 10M the 0.5 Hz
recipe becomes a second full-PASS candidate with strictly wider
tip/slip margins than 1.1 Hz — a champion-selection A/B, not a
rescue; FAIL branch (<0.72, gain-per-4M collapsed) hands the
operating point permanently to 1.1 Hz (or conditional-clock code
work). Read jointly with the phasehz11 seed twins.

Previous entry (~20:5x (**0.7 Hz dose point read — PARTIAL,
and it sharpens the curve: TIPS ARE NON-MONOTONE IN CADENCE with the
best point INTERIOR.** `-phasehz07` (clock 0.7 Hz, teacher_v2 demos,
otherwise unmutated phasehz05 recipe) m5: yaw PASS with tips
**0.1013/0.1363** — better than BOTH endpoints (0.149/0.156 @0.5,
0.216/0.235 @1.333; family-best left side), so slow-clock turn
authority does NOT require the full 0.5 Hz walk sacrifice. Walk
wpm24 (n_translating=28): det_prog_med **0.674** — recovers from
0.52 @0.5 toward 0.944 @1.333 but under the 0.75 bar (pre-registered
PARTIAL branch); det_slip_med **4.476** — above the 3.5 bar and the
family 3.2-4.0 band AT SOLID n, a real regression (slip dose curve
2.44 -> 4.48 -> 3.55 is non-monotone with the WORST point at 0.7).
Safety spotless (0 falls/terms all sections, gait 12/12 + 48/48
wpm24, no sacrificed legs). Consequence: `-phasehz09` (still
training) is now the PIVOTAL read — prog needs +0.08 with 0.06-0.10
tip margin available; its slip decides whether 0.7's 4.48 is a
mid-cadence valley-wall or a fluke. Read 0.9/1.1 jointly with fixed
points 0.5/0.7/1.333. Evidence: `logs/ckpt_eval/..._phasehz07_m5/`,
`..._phasehz07_m5_wpm24/`. Prior banner below.)

Previous entry (~20:0x (**CADENCE IS THE LEVER — the 0.5 Hz
phase-clock batch produced the FIRST yaw-section PASS in this
lineage's history AND the best walk slip ever measured, at the cost
of halved walk progress; 4-arm follow-up batch RUNNING.** Both arms
PARTIAL (verdicted): `-phasehz05` (clock 0.5 Hz, teacher_v2 demos
unchanged) m5 tips **0.1488/0.1557** vs parent 0.2157/0.2351 — both
sides clear the strict 0.20 M5 bar, improvements 0.067/0.079, 4-8x
the ±0.02 floor that eight prior mechanism classes (pricing,
demo-ceiling, style-ablation, densification, cpg-full-swap, disc
masking, turn-clip splice, envelope/exposure) never crossed; walk
det_slip_med **2.443** at n_translating=28 (`--walk-per-mode 24`
re-read; family band 3.47-3.83, bar 3.5) — best ever on the lineage;
0 terms, gait 48/48 walk, push 12/12 gv (1 det term, bar<=2), fault
11/12 (leg 5). `-phasehz05-cpglib` (coherent package: 0.5 Hz + cpg_v1
demos) matches: tips 0.1458/0.1367, slip 2.676 @ n=28, same safety.
READINGS: (1) the CLOCK is the mechanism, demos are second-order —
the teacher_v2-demo sibling won just as big, so cpgdemo1's old slip
regression was indeed the clock/demo cadence conflict; (2) the
cpglib gate's BONUS branch FIRES — the closed slip fork REOPENS with
CADENCE (not demo source) as the mechanism; (3) the COST: walk
det_prog_med 0.52/0.465 vs family ~0.94 (m5 bar 0.75) — achieved
~0.04 m/s vs 0.08 commanded; 0.5 Hz turns and grips but hasn't
learned big-stride translation in 2M. NOT clean PASSes: the gates
never priced prog, so both verdicted PARTIAL. LAUNCHED (this cycle,
all VERIFIED RUNNING): cadence dose sweep `-phasehz07/-phasehz09/
-phasehz11` (single lever each on the phasehz05 recipe, 2M discovery
— with the 0.5/1.333 endpoints this is a 5-point dose curve: find a
cadence with tips<=0.20 AND prog>=0.75, or declare the trade
monotone) + `-phasehz05-cont1` (+4M acquisition continuation from
the phasehz05 checkpoint per the 08-21 ruling — reward was still
climbing 43.5->216.5 by quarters; tests whether stride growth
recovers prog at 0.5 Hz, since cpg_v1 proves 0.08 m/s at 0.5 Hz is
plant-reachable; tips re-gated, never assumed). If no cadence point
is feasible and cont1 plateaus, next is command-conditional clock
code work (slow clock only under turn commands — mechanism, so
semantics-bank first). Tooling: `m5_pod_eval.py` now passes
`--walk-per-mode=` through (committed). Evidence:
`logs/ckpt_eval/..._{phasehz05,phasehz05_cpglib}_m5{,_wpm24}/
m5_verdict.json`, ledger verdicts on both. Prior banner below.)

Previous entry (~19:2x) (**RECONCILIATION + EXPOSURE CLASS
CLOSED + CADENCE MEASUREMENT FLIPS THE PHASE-CLOCK ARM'S SIGN +
2-arm capability batch RUNNING.** Two cycles raced on the exposure
batch (ops note: yawenv045/yawexpo02 were THIS cycle's "just
finished" assignment; a concurrent cycle triaged them anyway —
second double-triage today after wzmask2). Final recorded state:
BOTH runs are VERDICTED FAIL on their pre-registered branches —
yawenv045 tips unmoved (0.2238/0.2179 vs parent 0.2157/0.2351,
±0.02 floor; envelope-overshoot lever closed), yawexpo02 tips WORSE
(0.2443/0.2793, right +0.044 beyond the floor; exposure-dose lever
closed). The concurrent cycle's DIG-IN fork question (incentive-
stable optimum vs joint-space capability) is materially pre-answered
on the incentive side: `-noamp1` ALREADY ablated the style reward to
0.0 and tips stayed unmoved/worse — "style reward suppresses fast
turning" was directly refuted by that run's own pre-registered gate.
The still-open piece of the dig-in (per-leg cadence/amplitude of the
POLICY during tip scenarios) is noted below as an optional analysis;
the behavioral test is already training. NEW MEASUREMENT (this
cycle, FFT on motion-library joint tracks): teacher_v2 = 1.33 Hz
cadence, ~9 deg median p2p stride, turn clips wz~0.085; cpg_v1 (the
controller measured at ~0.29 rad/s on this plant) = **0.5 Hz — 2.7x
SLOWER — with 13-15 deg strides**. The plant-proven high-yaw gait is
slow-cadence/big-stride, so the "raise phase_hz" idea has the WRONG
SIGN; the capability lever is SLOWING the clock so yaw-per-stride
can grow. It also reinterprets cpgdemo1's slip regression as a
clock/demo cadence CONFLICT (0.5 Hz demos under a 1.33 Hz clock) —
the coherent package was never tested. LAUNCHED 2-arm batch
(VERIFIED RUNNING, 2M discovery each, family tip gate):
`-phasehz05` (train-2: goal.walk_phase_hz 1.333->0.5, single lever)
and `-phasehz05-cpglib` (train-0: 0.5 Hz clock + cpg_v1.npz demos,
coupled bundle — clock and style agree; BONUS branch: if its walk
det_slip_med <=3.5 with clean safety, the closed slip fork REOPENS
via cadence-coherent demos). If both land tips-unmoved the
phase-clock lever closes and the joint-space fork narrows to
stance-geometry code work / the bar-amendment question. Evidence:
`logs/ckpt_eval/..._{yawenv045,yawexpo02}_m5/m5_verdict.json`,
ledger gates on both new arms. Prior banner below.)

Previous entry (~19:1x, concurrent cycle, superseded where it
conflicts with the reconciliation above) (**exposure batch back in
minutes and both arms hit FAIL branches — envelope-overshoot
VERDICTED FAIL; yawexpo02 left UNVERDICTED with a DIG-IN flag
because its tips got WORSE with more practice, which decides the
fork.** `-yawenv045`
(walk_yaw_max_rad_s 0.3->0.45, single lever): tips 0.2238/0.2179 vs
parent 0.2157/0.2351 — unmoved inside the ±0.02 floor, arcs marginal
(0.1705 vs 0.1866), safety spotless (0 falls, 12/12 gait all
sections, push+fault PASS) — envelope lever refuted, verdicted FAIL.
`-yawexpo02` (walk_yaw_zero_frac 0.5->0.2, ~2.7x more nonzero-yaw
practice): tips 0.2443/0.2793 — RIGHT SIDE +0.044 WORSE than parent,
OUTSIDE the noise band, with walk actually healthy (det_slip 3.56 at
n_transl=6, prog 0.944, all sections' gait 12/12, push+fault PASS).
More turn practice made turn-in-place WORSE while everything else
held: that is anomalous-beyond-noise and it decides the yaw fork's
next mechanism class — if the slow turn is the INCENTIVE-STABLE
optimum (style reward punishing large/fast turning steps harder than
the yaw kernel rewards them, so extra practice entrenches it), the
fix is reward realignment (08-21 semantics-bank route, never yet
aimed at the yaw axis specifically), NOT capability; if per-leg gait
analysis instead shows the policy stepping at teacher cadence and
amplitude-saturated during tips, it's the phase-clock/stance-geometry
capability class (delta-clamp half already REFUTED eval-only this
cycle: dq10 probe left tips unmoved). DIG-IN NEEDED on yawexpo02:
per-leg step cadence/amplitude during tip-in-place vs the CPG
controller's turning gait + yaw-axis reward-flow accounting (style
vs yaw-kernel per-step income during fast vs slow turning) before
ANY next training arm. Evidence:
`logs/ckpt_eval/..._yawenv045_m5/`, `..._yawexpo02_m5/`. Prior
banner below.)

Previous entry (~18:4x (**turnclip1 FAIL — the SEVENTH and
LAST demo-side yaw mechanism is closed; the yaw fork now lives
entirely on the curriculum/exposure side, and a 2-arm batch is
launched there.** `-turnclip1` (teacher_v4 = teacher_v2 with ONLY
turn_ccw/turn_cw spliced from the CPG controller's real ~0.29 rad/s
clips, single lever on the unmutated `pushcal518` recipe) landed on
its pre-registered FAIL branch: m5 yaw tips 0.2375/0.2457 vs parent
seed7 0.2157/0.2351 — both sides slightly WORSE (+0.022/+0.011,
at/inside the wzmask2-calibrated ±0.02 floor), no side <=0.22. The
decisive detail: achieved wz on the m5 arc scenarios is 0.125-0.165
rad/s — STILL pinned at the scripted teacher's 0.15-0.16 saturation
even though the exact demo family the bar scores now demonstrates
0.29 rad/s. The demo library does NOT set the policy's turn
authority: pricing, demo-ceiling (turnlib3), style-ablation,
densification, cpg-full-swap (cpgdemo1), discriminator-obs masking
(x2 widths, n=5 grid), and now the surgical turn-clip splice are ALL
refuted. Secondary: 0 falls, push PASS; walk det_slip_med 4.56 above
the 3.2-4.0 band but on n_translating=3 (thin-sample noise per
q_20260823T0700Z); fault gait_valid 8/12 (3 sacrificed legs, worse
edge of the 10-12 family range) — watch fault on the next arms.
REMAINING LIVE FORK (per the turn-authority probe: 0.30 rad/s IS
plant-reachable, bar amendment answered NO): curriculum/exposure and
joint-space capability. LAUNCHED 2-arm single-lever batch on the
unmutated `pushcal518` recipe (teacher_v2, seed 7, 2M each):
`-yawenv045` (goal.walk_yaw_max_rad_s 0.3->0.45: make the bar's 0.30
an INTERIOR training command, not the distribution edge — the policy
has literally never been commanded past the rate it is graded at) and
`-yawexpo02` (goal.walk_yaw_zero_frac 0.5->0.2: ~2.7x more nonzero-
yaw practice per rollout, everything else identical). Family gate
convention (tips -0.03 / one side <=0.22 / slip band / safety) on
both. If BOTH land tips-unmoved, the exposure class closes too and
the fork escalates to joint-space capability (action-delta clamp /
stance geometry / phase-clock constraint — e.g. an eval-only
max_delta_q_deg probe on an existing checkpoint, and whether the
1.333 Hz phase clock forbids the CPG's turning cadence). Evidence:
`logs/ckpt_eval/..._turnclip1_m5/` (yaw.json scenarios),
verdict on run l80xbg8f.

SAME-CYCLE ADDENDUM (~19:0x, eval-only delta-clamp probe — the
joint-space-capability fork's first sub-hypothesis is already
REFUTED, for free): added `--cfg=` passthrough to
`rl_move/orchestrator/m5_pod_eval.py` (appended last, last-wins;
always pair with `--suffix`) and re-ran the yaw-only m5 section on
the frozen `pushcal518` checkpoint with `safety.max_delta_q_deg`
relaxed 5.0->10.0 (`..._m5_dq10probe`): tip errs 0.2305/0.2328 vs
default-clamp 0.2157/0.2351 — UNMOVED (+0.015/-0.002, inside the
±0.02 floor), arcs turn_wz_err_med 0.1608 vs 0.1866 (marginal). The
per-tick action-delta clamp is NOT what caps turn rate; the policy
simply does not command a faster turn. Remaining capability
candidates if the exposure batch fails: the fixed 1.333 Hz phase
clock (CPG turning cadence may differ) and stance geometry.
METRIC-SEMANTICS CORRECTION to the turnclip1 verdict text: the m5
yaw scenario `wz_med` values are median |wz ERROR| (eval_amp_m5
docstring line 26), not achieved rates — so turnclip1's achieved
rates are ~0.135 on max arcs and ~0.065 on tips (parent ~0.065-0.084
tips), NOT "achieved 0.15-0.16"; the verdict's conclusion (tips
unmoved/worse, demo-side closed) is unaffected. Prior banner below.)

Previous entry (~18:1x (**yaw fork: SEVENTH mechanism class
launched (surgical turn-clip splice), and the slip-axis "thin
sampling" amendment is independently CONFIRMED at n=28 translating
episodes.** Two pieces of work this cycle, unblocked by (and
downstream of) the wzmask2 n=5 grid closure + the concurrent cycle's
turn-authority probe (both above): (1) built `eval_amp_m5.py
--walk-per-mode` (default None = old `--per-mode`, bit-exact, 2 new
unit tests) implementing the `q_20260823T0700Z` amendment, then ran
an eval-only (no GPU/training spend) walk-only re-read of the
lineage's real M5-candidate base (`...pushcal518`) at
`--walk-per-mode 24`: n_translating jumps from ~3/12 to 13/24 det +
15/24 sto, det_slip_med **3.553** — lands exactly on the earlier
per-mode-12 read (3.553) and squarely in the family's 3.47-3.83 band,
confirming (independently, 2nd invocation) that the walk-slip bar
miss is noise-floor, not a live mechanism; 0 falls/terms, gait_valid
24/24. (2) Built `merge_motion_library.py` (11/11 new unit tests) to
splice NAMED clip families from one motion library into another,
tick-range-exact — the tool neither `teacher_v3` (rescaled the
SCRIPTED teacher's own turn clips, dose too small, `-turnlib3` FAIL)
nor `cpgdemo1` (swapped the WHOLE library, slip regressed,
`-cpgdemo1` FAIL) provided: isolate ONLY the turn_ccw/turn_cw demo
family (the exact clips the M5 tip-left/right bar scores) at the
CPG-search controller's own achieved rate (~0.29 rad/s at commanded
0.30, just confirmed plant-reachable by the concurrent cycle's
turn-authority probe), leaving every other family byte-identical to
`teacher_v2` so a `cpgdemo1`-style slip trade is far less likely.
Built `teacher_v4.npz` (39/45 clips from `teacher_v2`, 6/45 turn
clips from `cpg_v1`; loads clean under `MotionLibrary`'s per-clip-
neutral hard-fail check) and launched `-turnclip1` (single lever
`--amp-motion-lib` v2->v4 on the unmutated `pushcal518` recipe, 2M
discovery, VERIFIED RUNNING train-2) — the seventh yaw mechanism
class and the first genuinely un-refuted one (pricing/demo-ceiling/
style-ablation/densification/cpg-full-swap/discriminator-obs-masking
are all closed FAIL). Pre-registered gate: PASS both tips improve
>=0.03 with >=1 side <=0.22 and slip stays 3.2-4.0 and 0 falls;
PARTIAL one tip clears or slip regresses >0.3 (cpgdemo1 trade
reproduced); FAIL tips unmoved (+-0.02, wzmask2-grid-calibrated
floor) -> closes ALL demo-side yaw mechanisms, leaving only
curriculum/exposure (non-demo) mechanisms or the bar-amendment
(already answered NO this cycle, q_20260823T0130Z/q_20260823T1750Z:
0.30 rad/s is plant-reachable) as open forks. Evidence:
`logs/ckpt_eval/diag_pushcal518_walkpermode24/`,
`rl_move/orchestrator/OPERATOR_QUESTIONS.md` q_20260823T1750Z.)

Previous entry (~17:4x: **the wzmask2-s23/s13/s17 decider
grid is CLOSED at full n=5 — the gyro-channel yaw hypothesis is DEAD
FOR REAL, and the close was predicted exactly by order-statistics
math before the last arm even finished.** All five draws of the
identical wzmask2 recipe (mask dims 36-38, full gyro triad): `wzmask2`
0.239/0.2371, `gyroxyz` 0.174/0.202, `s23` 0.2092/0.252, `s13`
0.215/0.2234, `s17` 0.2082/0.2442. Pooled n=5 medians: tip_left
**0.2092** (parent pooled 0.2168, delta -0.0076), tip_right **0.2371**
(parent 0.2351, delta +0.0020) — both inside the grid's own ±0.02 FAIL
band, and only 1/5 draws (gyroxyz) has any side <=0.20 (bar needs
>=2/5). Grid gate: FAIL on both the median and the count condition.
(The n=4 read at s23's own triage cycle had already mathematically
LOCKED this outcome — a 5-draw median is bounded by the sorted
4-draw's 2nd/3rd order statistics, so no possible s17 value could
move it outside the FAIL band; s17 landed at 0.2082/0.2442, inside the
predicted [0.2092,0.215]/[0.2234,0.2371] ranges exactly.) Reading: the
two earlier straddling n=2 draws (`wzmask2` FAIL-looking, `gyroxyz`
PASS-looking) were both samples from one noisy-but-flat distribution
centered near the parent, not a real lever — masking the
discriminator's rotation-sensing dims (gyro-z alone in `wzmask1`, then
the full gyro triad here) moves nothing. This closes the FIFTH+SIXTH
yaw mechanism classes (discriminator-obs, both widths) on top of
pricing/demos/style-ablation/reset-densification — **the yaw fork has
no live discriminator-obs or demo/pricing/densification lever left;
every named mechanism class is refuted.** Remaining paths are the
0.20-bar amendment (q_20260823T0130Z, still the operator's open call)
or stance-geometry/turn-authority (joint-space capability, not
incentive/exposure) — no further discriminator-obs mask arms are
warranted. Safety/behavior across the grid: 0 falls/terms every
section every draw; walk det_slip_med clusters 3.47-3.83 (at the
family's known ~3.5 sampling-noise floor); fault gait_valid ranges
10-12/12 (s17 sacrificed 2 legs, others clean) — normal seed-to-seed
fault-section noise, not a mask effect. Evidence:
`logs/ckpt_eval/..._wzmask2_{s23,s13,s17}_{gate,m5}/`.)

**SAME-CYCLE ADDENDUM (~18:1x, turn-authority probe — the yaw fork's
"is 0.30 rad/s even plant-reachable?" question is now MEASURED, not
open):** ran the recommended eval-only probe (OPERATOR_QUESTIONS
q_20260823T0130Z dig-in note): `eval_cpg_gate.py` on the exported CPG
controller artifact (`cpg_controller_robust120_yawtrim.json`, same
plant, same servo contract write 1500/acc 80) with the turn segments
commanded at **wz=0.30** (the m5 yaw envelope) instead of the gate's
default 0.20. Result: PASS — turn yaw_along_frac **0.898/0.977**
(~0.27-0.29 rad/s achieved), prog_mean 0.894, slip/m 0.70, no falls,
no sacrificed legs (`logs/ckpt_eval/cpg_turnauthority_wz030/`).
Consequence: 0.30 rad/s turning IS physically achievable at this
plant with a scripted controller, so the m5 yaw bar (tip_err<=0.20 =
achieved>=0.10) is NOT plant-limited and the bar-amendment branch
loses its "never physically achievable" justification — the miss is
the POLICY's (and plausibly the demo library's: teacher_v2 clips
saturate ~0.15-0.16 rad/s, q_20260823T1240Z, while the CPG controller
demonstrably reaches ~0.29). Live levers for the next amp yaw arm, in
order: (1) turn-curriculum/capability training (policy has never been
made to practice max-rate turns against a reference that can actually
do them), (2) a CPG-sourced TURN-clip library at wz 0.25-0.30 (raises
the demo ceiling with clips from the one controller measured to turn
at rate; distinct from cpgdemo1's refuted WALK-slip swap — that arm
swapped the walk anchor for slip, this one adds turn-rate coverage).
No mask/pricing/densification arms — those six classes are closed.)

Previous entry (~17:3x: **wzmask2 widen-mask read is
INCONCLUSIVE in the most instructive way possible: two
identical-config, identical-seed draws of the SAME recipe read
opposite verdicts, exposing that the tip metric's replicate noise
floor (Δleft 0.065) is 3x the ±0.02 band every yaw arm has been
gated on; 3-seed decider grid launched.** The launch-race duplicate
`-wzmask2-gyroxyz` was ledger-KILLED at 16:37 but the kill never
took effect — it trained to its full 2M (W&B finished, 2,031,616
steps) and GPU non-determinism diverged it from `-wzmask2` (ep_rew
262.4 vs 233.7, NOT byte-identical as the kill verdict assumed).
m5 reads: `-wzmask2` tips 0.239/0.2371 (the pre-registered FAIL
signature: right unmoved, left wrong-way +0.023) vs `-gyroxyz`
tips **0.174/0.202** (the PASS signature: both sides −0.033/−0.042,
family-best-ever left, below the 11-read family floor 0.198; also
the closest any family member has come to the v1 yaw bar). Both
draws safety-clean: 0 falls, 0 terms, walk gv 12/12, det slip
3.81/3.83 (unmoved under the <0.3 caveat), fault gv 10/12 with
video-clean parked carried-fault legs, six-leg strips clean.
Consequence: neither gyro-channel closure nor lever promotion is
claimable at n=2 straddling, and every single-read ±0.02-0.03 tip
verdict this campaign (wzmask1, cmdcond1, cpgdemo1's tip_left,
noamp1's gain) is retroactively weakened — the tip analog of the
slip x12 sampling finding. Per the tipspawn1b replicate-grid
precedent, LAUNCHED the 3-seed decider grid `-wzmask2-s23/-s13/-s17`
(exact recipe, 2M each; s23/s13 pair with the parent's own matched
seeds 0.2493/0.2393 and 0.2168/0.2269): grid gate = pooled mask n=5
medians vs parent pooled n=3 (0.2168/0.2351) — PASS both improve
>=0.02 with >=2/5 draws at a side <=0.20; FAIL within ±0.02 (gyro
closes for real, gyroxyz = tail draw); either way the grid delivers
the first measured tip replicate-noise calibration, which the
0.20-bar question (q_20260823T0130Z) needs regardless. gyroxyz's
checkpoint retained append-only as the family yaw-best artifact /
promotion candidate if the grid confirms. OPS: (1) killrun was a
silent no-op on zero matches and never re-verified — FIXED in
ops.sh (match count, re-scan, kill -9 escalation, 'verified dead'
gate before any KILLED ledger mark); (2) one snapshot pull-rebase
failed with 'Cannot rebase onto multiple branches' (FETCH_HEAD race
with a concurrent cycle's fetch) and a STALE ~03:48 autostash was
found+dropped — if you hit this, check `git stash list` age before
popping. Evidence:
`logs/ckpt_eval/..._wzmask2_{gate,m5}/`, `..._wzmask2_gyroxyz_{gate,m5}/`.)

Previous entry (~16:3x: **slip axis CLOSED by measurement;
gyro-mask yaw lever refuted at dim 38, final widen arm launched.**
Three reads this cycle. (1) `-cpgdemo1` FAIL (double-triaged by two
cycles, verdicts agree): swapping the demo anchor to the CPG-search
library (cpg_v1, −39% slip in its own clips) made everything WORSE —
m5 walk slip 4.37 (parent 3.67), probe 15.84mm (control 14.03),
fault gait_valid 10/12 with 2 sacrificed legs; only tip_left 0.1936
improved (first family member past 0.20, the predicted ccw clip
asymmetry). Demo-anchor fork closed; obs_style does not transmit
clip ground-contact slip. (2) The q_20260823T0700Z sampling
amendment was EXECUTED, not just proposed: per-mode-12 parent
resample (`_m5_x12`) reads det slip med **3.553** (eps 3.5/3.55/3.8)
— the family sits AT the 3.5 bar, the campaign's ±0.1-0.2 "misses"
were n=2-episode sampling noise, and only Δ>=0.4 regressions
(stdanneal50, cpgdemo1) were real. NO more slip-mechanism training
arms on this lineage; bar-value ruling is the operator's
(q_20260823T0700Z). (3) `-wzmask1` FAIL: masking gyro-z alone leaves
tips unmoved (0.202/0.2371 vs 0.2157/0.2351, band ±0.02) with slip
in-family (3.85) — the discriminator reads rotation off
joint_vel/foot dims, not the yaw-rate channel; pre-registered single
widen arm `-wzmask2` (mask 36-38, full gyro) VERIFIED RUNNING
train-0 — if unmoved, gyro-channel hypothesis closes and yaw rests
on stance-geometry/turn-curriculum or the 0.20-bar amendment
(q_20260823T0130Z). (4, ~16:3x cycle) `-cmdcond1` FAIL: command-
conditioning the discriminator (63-dim obs_style + matched
teacher_v2_cmdcond library) moved tips the WRONG way (0.249/0.2669
vs parent 0.2157/0.2351, +0.033/+0.032), safety/slip clean (0 falls,
walk gv 12/12, slip 3.5725), weights moved 13/13, disc healthy —
the FIFTH yaw mechanism class refuted (pricing, demo ceiling,
noamp1, reset densification, cmd-conditioning); discriminator-obs
structural fixes now rest entirely on `-wzmask2`. Ops: cmdcond1's
cycle independently launched the same widen arm as
`-wzmask2-gyroxyz` (identical seed/args, 2-min race with the
concurrent cycle's `-wzmask2`); the duplicate was killed at launch
and ledger-verdicted — same-branch follow-ups from double-triaged
parents are a known collision mode. Tooling:
`m5_pod_eval.py --per-mode/--suffix` (default bit-exact). Evidence:
`..._cpgdemo1_{m5,gate}/`, `cpgdemo1_slipdist.json`, `..._m5_x12/`,
`..._wzmask1_m5/`, `..._cmdcond1_m5/`.)

Previous entry (~15:4x: **tipspawn1b seed-replicate grid
0/3 PASS — the state-visitation slip fork is CLOSED; the noise-floor
anneal family is the ONLY live slip mechanism.** Exact tipspawn1b
recipe re-run at seeds 11/13/17 (2M each, trained normally, reward
~41→250 all three): m5 walk det_slip_med **s11 3.55, s13 3.73, s17
3.765** vs bar 3.5 — all three land in the 3.55–3.77 family baseline
band (parent 3.67, startonly 3.60, wzonly 3.59), exactly the
pre-registered prediction-if-false. The original tipspawn1b 3.1855
was a one-seed fluke; per the grid's own ruling the combined
RSI-start_frac × spawn_wz interaction is refuted and NOT promoted.
Safety clean all seeds: 0 falls all sections, walk gait_valid 12/12,
healthy six-leg det strips. Cross-fork comparison (answering the
~15:2x note): state-visitation never moved the drag probe or slip;
the train-noise anneal is the only mechanism with a real measured
effect (probe −32%) — the running `-stdanneal50` rung (other cycle)
carries the slip question alone, including its pre-registered
PARTIAL→bar/metric-amendment branch for the probe-vs-m5
dissociation. Ops note: s11's watcher prestage was missed; its
checkpoint pull + gate/m5 evals were run manually this cycle.
Evidence: `logs/ckpt_eval/cw_amp_m4_..._tipspawn1b_s{11,13,17}_{gate,m5}/`.
SAME CYCLE, yaw axis re-armed: the yaw fork had NO live lever
(pricing dose-refuted, turnlib3 demo-ceiling refuted at dose,
tipspawn densification refuted, noamp1 disqualified by its slip
trade) — built `--amp-style-mask-dims` (zero named obs_style dims on
BOTH real+fake discriminator inputs; default '' bit-exact,
`test_amp_style_mask.py` 6/6 + amp banks 21/21, snapshot
`exp/...-wzmask1`) and LAUNCHED `-wzmask1` (mask dim 38 = gyro-z/yaw
rate, single lever on pushcal518, 2M, train-0, VERIFIED RUNNING):
tests whether noamp1's tip gain (−0.038/−0.020) can be had through
the rotation channel alone WITHOUT its +0.25 slip cost. Gate: PASS =
both tips improve ≥0.02 (≥1 side ≤0.20) with slip within ±0.15 of
3.67; PARTIAL = tips improve but slip regresses (noamp1 trade
reproduced → lever closed); FAIL = tips unmoved → one widen-mask
follow-up (36-38), then the gyro-channel hypothesis closes and yaw
goes to stance-geometry/turn-curriculum.)

Previous entry (~15:2x: **`-stdanneal45-r2` PARTIAL — the
train-noise floor is the FIRST mechanism to genuinely cut the
family's loaded-foot drag; `-swinganneal45-r2` PARTIAL but the swing
composition is NOT adopted.** Both r2 arms pass the weight-movement
precheck (12/12 non-log_std tensors moved vs turnfault_seq1 AND
pushcal518; log_std −4.46 — the r1 freeze fix works). Probe, matched
same-cycle conditions (hazard-free own-cfg, seed 0, 6 eps, ~900
stances; SAME-conditions parent control rerun reads **14.03mm**
median, not the 11.49 recorded under the earlier invocation — use
matched controls for this probe from now on): anneal-alone
**9.55mm** (−32%, tail p90 25.8 vs 35.2) > anneal+swing 10.43 >
parent 14.03. Every reward-side lever left this median at/above
parent; the noise floor moved it. BUT m5 walk det slip did NOT
follow: anneal 3.71 (unmoved vs 3.67, bar 3.5), anneal+swing 3.515
(family-best, misses by 0.015) — a probe-vs-m5 DISSOCIATION (stance
travel −32%, slip/m flat) suggesting the m5 walk slip metric is
dominated by something other than loaded-stance drag (likely the
stress_mix turn-in-place phases); noted on q_20260823T0700Z, no bar
changed. Composition answered: swing income redistributes drag (tail
p90 33.3 vs anneal's 25.8), regresses tip_left past the 0.25 band
(0.2512) and costs fault sto gait_valid (11/12; swing1 alone 9/12) —
the anneal is the active ingredient, swing is dropped. Safety clean
both arms: 0 falls all sections, walk gait 12/12, videos clean.
LAUNCHED the gate-prescribed dose rung `-stdanneal50`
(--log-std-final=-5.0, single lever, train-0, VERIFIED RUNNING):
PASS = probe <=8mm AND slip <=3.5; PARTIAL = probe <=8 with slip
unmoved -> escalate the dissociation to a bar/metric amendment
instead of more rungs; FAIL = probe >9 or robustness cost -> fork to
teacher-v4 demo-anchor (CPG contextual winner as motion library).
Cross-fork note: tipspawn1b seed replicates (concurrent cycle) probe
the OTHER live slip mechanism (RSI×spawn_wz interaction, m5 slip
3.1855); when both read, compare noise-floor vs state-visitation
before any adoption. Evidence:
`logs/ckpt_eval/{stdanneal45_r2,swinganneal45_r2,pushcal518_ctrl}_slipdist.json`,
`logs/ckpt_eval/cw_amp_m4_..._{stdanneal45,swinganneal45}_r2_m5/`.)

Previous entry (~15:0x: **`-tipspawn3-wzonly` FAIL — the
2×2 slip-mechanism isolation is COMPLETE: neither lever alone works,
only the combination; a 3-seed replicate batch of the combined dose
is launched.** m5 walk det_slip_med 3.59 vs parent(pushcal518) 3.67 —
Δ0.08, inside the pre-registered ±0.15 "unmoved" band (bar 3.5;
tipspawn1b combined hit 3.1855). Full 2×2 on pushcal518: start_frac
alone 3.6015 (unmoved), spawn_wz alone 3.59 (unmoved), both 3.1855
(win) — the slip effect, if real, is an INTERACTION of mid-walk RSI
× live spawn omega, not either half. Safety clean (0 falls all
sections, walk gv 12/12, clean six-leg det strip; push PASS 3.2405,
fault PASS gv 11/12). Yaw side-read: tip_left 0.1945 clears 0.20 but
tip_right 0.2223 regressed — same asymmetric trade as the whole turn
grid, no new signal. LAUNCHED per the run's own pre-registered FAIL
branch: exact tipspawn1b replicates at seeds 11/13/17 (2M discovery,
train-0/1/2; grid ruling ≥2/3 at ≤3.5 = effect REAL → promote both
levers to an acquisition arm; 0/3 = tipspawn1b was noise → close the
state-visitation fork, leaving the train-noise anneal family
(stdanneal45-r2, concurrent cycle) as the last live slip mechanism;
1/3 = fragile, dig in first). Evidence:
`logs/ckpt_eval/cw_amp_m4_..._tipspawn3_wzonly_{gate,m5}/`.

Previous entry (~14:4x: **`-stdanneal45` + `-swinganneal45`
INVALID — NEITHER RUN EVER TRAINED; the noise-floor question is
still OPEN, and a silent trainer defect is found+fixed.** Forensics:
both checkpoints' 12 network tensors (actor AND critic) are
byte-identical to the warm-start ancestor `turnfault_seq1` — only
log_std moved. Root cause: the `--log-std-final` callback set
log_std at `on_rollout_end`, i.e. between collection and PPO's
`train()`; the log_prob shift alone put first-minibatch approx_kl at
~0.13 > 1.5×target_kl(0.02), and SB3's early-stop fires BEFORE
`optimizer.step()` — pod logs show `Early stopping at step 0` on
31/31 updates. The twins' "rising reward" (43→268 / 48→294) was pure
shrinking-noise artifact on the frozen policy; their W&B/eval deltas
are k_walk_swing re-PRICING byte-identical trajectories (same seed
7). Blast radius: joystick stotight ladder AUDITED CLEAN (all
champions show real non-log_std weight deltas; their anneal rate per
rollout was ~6× smaller so KL stayed under the stop). FIXED
(`exp/logstd-anneal-rollout-start-fix`): anneal now sets log_std at
`_on_rollout_start` so collection/buffer/train share one log_std;
freeze-provoking smoke `smoke-logstd-anneal-v2-freeze-fix` gates on
weights actually moving; both arms relaunched as `-r2` with
unchanged hypotheses/gates (stdanneal45-r2 train-1, swinganneal45-r2
train-2). Triage rule added to RUN_INTERPRETATION_RULES §1: rising
reward on any noise-schedule run proves nothing until non-log_std
weights are confirmed to differ from the parent.)

Previous entry (~14:2x: **`-swing1` (gait-income slip fork)
FAIL — the gait-income lever is CLOSED alongside pricing, and the
slipdist probe explains the near-miss.** m5 walk det slip med 3.5035
vs bar 3.5 / parent 3.67 — family-best among reward-side arms, but
the 0.167 improvement lands BETWEEN the pre-registered bands
(>±0.15 unmoved, <0.3 PARTIAL), so the gate-mandated slipdist rerun
decides: per-stance loaded travel median got WORSE (11.49→13.54mm;
champion 5.5) while only the heavy tail compressed (p90 40.3→33.6,
p95 48.8→39.4) — swing income REDISTRIBUTED drag (trimmed worst-case
drags = the eval delta) rather than producing cleaner plants: the
pre-registered "foot-lift theater" alternative, half-confirmed. Plus
a fault-section cost: sto gait_valid 9/12 (bar 10, parent 12/12;
three DIFFERENT legs [5],[2],[1], det 6/6 — no farm/flag pattern).
Safety intact (0/12 falls, walk gv 12/12, push PASS, tips unchanged
0.2141/0.2352). LAUNCHED the strongest remaining stance-level
mechanism as a 2-arm batch: this family trains at FIXED std 0.135
(no anneal) while the 5.5mm-stance joystick champion stotight45s13
annealed to log-std −4.5, and the joystick track measured (08-22 pd8
dig-in) that the train-noise floor is what makes honest stances
drag — `-stdanneal45` (--log-std-final=-4.5, single lever, train-1)
and `-swinganneal45` (anneal+swing composition: does the tail win
survive and does the fault regression follow swing; train-2). NOTE
cross-fork: tipspawn1b (concurrent) hit walk slip 3.1855 via mid-walk
RSI (`walk_gait_start_frac=0.5`) — state-visitation is a THIRD live
slip mechanism (tipspawn2-startonly isolating it, train-0); when
these read, compare all three before any adoption. Bar ruling
unchanged: 5.5mm stances attainable → keep 3.5. Evidence:
`logs/ckpt_eval/cw_amp_m4_..._swing1_{gate,m5}/`,
`logs/ckpt_eval/swing1_slipdist.json`.

Previous entry (~14:0x: **`-tipspawn1b` (reset densification,
turn-tracking fork) FAIL on its pre-registered branch, but with a
notable side win on the OTHER open fork (slip pricing).** m5 yaw
tip_left/right_err 0.2448/0.2100 vs parent(pushcal518) 0.2157/0.2351:
left got WORSE by 0.029 (outside the ±0.02 noise band), right
improved by only 0.025 (short of the PARTIAL bar's required ≥0.03 on
BOTH sides) — neither clears 0.20, so it's a genuine FAIL (mixed
asymmetric trade, the same one-side-better-one-side-worse signature
already seen in the `k_yaw_prog` pricing grid), not a clean miss and
not PARTIAL. Zero raw falls anywhere, gait_valid 12/12 every section
— safety unaffected. **Turn-tracking is now refuted against FOUR
independent mechanism classes**: pricing (`k_yaw_prog` 1-3x),
demos (`teacher_v3`, +30% wz ceiling), style-ablation (`noamp1`,
confounded with slip), and now reset densification (this run).
Remaining levers are stance-geometry/turn-authority (joint-space
capability, not incentive/exposure) or the 0.20 bar amendment ruling
(`q_20260823T0130Z`, still open) — do NOT re-dose
`walk_gait_spawn_wz`. **Side finding, matches the launch notes' own
pre-registered "strongest alternative" prediction:** walk
det_slip_med improved sharply to 3.1855 (parent 3.67, bar 3.5) — the
FIRST arm in the entire pricing/demos/densification dose-grid history
to clear the walk-slip bar, right after additive/income-gate pricing
on that exact axis was measured CLOSED (`-loadgate45` FAIL, same
cycle). Push section slip 3.4955 also now under 3.5. This points at
the mid-walk RSI half of the dose (`walk_gait_start_frac=0.5`, not
the turn-spawn/`spawn_wz` half) as the actual mechanism — slip looks
like a state-visitation/gait-phase-distribution problem, not a
pricing problem. Next: isolate `walk_gait_start_frac` alone (no
`spawn_wz`) to confirm which half of the dose earns the slip win, and
compare against `-swing1`'s (running concurrently) different slip
mechanism once it reads. Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_tipspawn1b_{gate,m5}/`.

Previous entry (~13:3x: **BOTH FORK ARMS FAIL ON THEIR
PRE-REGISTERED BRANCHES; both escalations executed same cycle.**
(1) `-turnlib3` (teacher_v3 turn demos, +30% demo wz) FAIL: m5 tips
0.1965/0.248 vs parent 0.2157/0.2351 — both inside the ±0.02 unmoved
band (walk clean: slip 3.731 ≤3.8 allowance, 0/12 falls, 12/12
gait_valid). Demo ceiling refuted at this dose; with pricing (n=4),
demos, and style-ablation (noamp1) all measured unable to move tips,
the residual cause is exposure: the policy never VISITS fast-turn
states. BUILT+SNAPSHOTTED (exp/amp-gait-spawn-wz): `goal.
walk_gait_spawn_wz` — omega passthrough into the mid-stride gait
spawn (sim_env now feeds traj.wz into TripodGait.set_velocity;
walk_task makes the yaw command live from the 0.3 s fast ramp; also
fixed the latent tip×gait-spawn interaction that resurrected the
discarded linear command; default off, bit-exact, test_sim_env 45/45,
semantics bank 174 pass/1 pre-existing fastprof red). LAUNCHED
`-tipspawn1b` (gait_start_frac=0.5 + spawn_wz=1.0 on pushcal518, 2M
discovery). NOTE: first attempt `-tipspawn1` LAUNCH-FAILED — respec's
`--arg` channel mangles `--cfg-set k=v` into `--cfg-set k v`; always
use `--cfg K=V` for cfg pairs.
(2) `-loadgate45` (full-strength g=1.0 loadslip income gate) FAIL:
m5 walk det slip med 4.072 vs parent 3.67 / bar 3.5 — WORSE with
reward rising 37→179; **slip-axis PRICING CLOSED per its own gate
text** (additive 6x/12x + income-gate all refuted). ROOT-CAUSE PROBE
run same cycle (probe_stance_slip_dist, hazard-free own-cfg, 6 eps):
pushcal518 per-stance loaded travel median 11.5 mm / p90 40 mm /
mean 16.6 vs the joystick champion stotight45-seed13's 5.5/10.8/5.8
at the SAME 0.08 m/s command and plant geometry
(`logs/ckpt_eval/{pushcal518,stotight45s13}_slipdist.json`). The
slip excess is DIFFUSE (every stance ~2x) plus a heavy tail (p90
3.7x) — a real gait-quality gap, NOT a metric artifact and NOT an
unattainable bar: 5.5 mm stances are demonstrably achievable, which
argues AGAINST loosening the 3.5 bar. LAUNCHED the gate's named
gait-level escalation `-swing1` (reward.k_walk_swing=1.0 on
pushcal518, 2M discovery; single-leg-farm live cheat pre-registered).
If swing1's slip is unmoved, the fork is stance-geometry mechanisms
vs the bar ruling — rerun the slipdist probe on it either way.
Prior banner below.)

Previous entry (~12:4x: **DIG-IN RESOLVED: the joint
pricing-grid FAIL's structural cause is MEASURED, and the fork is
launched.** (1) YAW ROOT CAUSE: the untrimmed scripted tripod
teacher ACHIEVES only 0.134-0.144 rad/s at commanded 0.25-0.30
(direct probe this cycle; saturates ~0.15-0.16 at ANY commanded
omega — foot sweep per stride is geometry-capped), so teacher_v2's
turn demos EMBODY ~0.134 rad/s while labeled 0.25, and the AMP
discriminator (obs_style contains base_angular_velocity,
UNCONDITIONED on command) punishes rotating faster than the demos —
that is why every arm sits at the price-invariant achieved-ratio
~0.5 (yaw_prog val 0.48 at k=1 and 0.49 at k=3: pure income
inflation, not an economic equilibrium). `-noamp1` (AMP-style-weight
0 diagnostic, verdicted FAIL/this cycle) confirms direction: tips
improved to 0.1778/0.2151 (left CLEARS the 0.20 bar) but walk slip
REGRESSED 3.67->3.92 — the style term is the turn CAP and the slip
FLOOR-HOLDER at once, so the fix is demos, not ablation. BUILT+
SNAPSHOTTED: `teacher_v3.npz` (turn clips at stride_scale 1.4/
period_scale 1.2 = measured 0.174 rad/s, +30% demo wz ceiling,
slip/m 1.26; all non-turn clips bit-exact vs v2; builder now records
measured_wz in manifests). LAUNCHED `-turnlib3` (single lever:
--amp-motion-lib v2->v3, train-0). (2) SLIP: dose-honesty audit
found the pre-registered PARTIAL income gate (g=0.5, band 1.5/4.5)
exerts ~0.5/tick marginal — the SAME dose slipexcess12 already
refuted — so the arm launched at FULL strength (g=1.0: ~1.0/tick
marginal = 2x refuted, 70% of walk income withheld at operating
ratio 3.6, floor 0.9/tick, bank 5/5 green incl. the pinned
default-band zero-gradient no-information-tax finding): LAUNCHED
`-loadgate45` (train-1). If loadgate45's slip is unmoved too,
pricing on the slip axis is CLOSED (gate text pre-registers this) —
next is a gait-level mechanism (k_walk_swing-style) or the
q_20260823T0130Z bar ruling. Both arms' triage belongs to the
watcher's next cycles; do NOT re-launch this pair. Prior banner
below.)

Previous entry (~12:0x: **DOSE-GRID ARM 1 OF 4 READ:
`-slipexcess6` FAIL on its pre-registered slip bar — the per-tick
slip charge ENGAGED but slip is PRICE-INELASTIC at dose 6.** m5 walk
det slip med 3.629 vs bar 3.5 and parent's ~3.67 (delta 0.04 <<
0.15 PARTIAL threshold = unmoved); the mechanism worked as built
(env/reward_loadslip_excess ramped to -0.48/tick, the designed ~0.5/
tick pressure; training loadslip_ratio 3.45-3.63 vs parent
3.57-3.70) — the policy just pays the fine. Safety fully intact:
0/12 raw falls own-cfg DR-0 (roll_peak max 13.4), m5 walk terms 0 +
gait_valid 12/12 + det prog 1.01. Watch item: yaw tips 0.253/0.261,
marginally past the 0.25 band but inside the family span
(0.198-0.317). **Charge-lever ruling now hangs on `-slipexcess12`
(2x dose, still training): slip also unmoved there = per-tick charge
REFUTED as the anti-slip lever -> next lever is the loadslip INCOME
gate (walk_loadslip_gate) at partial strength, not more charge; do
NOT launch that gate arm before slipexcess12 is read.** yawprice2/3
are other cycles' triage. Prior banner below.)

Previous entry (~11:4x: **DIG-IN RESOLVED: the "push
recalibration systematically costs walk-slip/yaw-tip margin" claim is
REFUTED — it was a BASELINE ARTIFACT. The 2-seed comparison anchored
on parent `tipfrac05`(seed7)'s tips 0.162/0.184 + slip 3.36, which the
full family table exposes as a 1-in-11 outlier: across ALL 11
old-push-range (10-25N) tipfrac05-family m5 reads (s2/s3/seed31/
seed41 seed twins, 5 kernelema arms, acq1) tips span 0.198-0.317
(median ~0.23) and walk det slip 2.97-4.33 (median ~3.8); the 3
recalibrated (5-18N) seeds — tips 0.216-0.249, slip 3.62-3.82 — sit
INSIDE that distribution at-or-better than the median, while
dominating on safety (0/12 falls x3 vs widespread) and fault
gait_valid (12/11/11). Clincher: `acq1` (+6M at the OLD range, reward
rising) fails BOTH bars too (slip 3.52, tips 0.204/0.269). ROOT
CAUSE: family-wide reward<->M5-bar misalignment (08-21 ruling) — the
reward optimum sits at tip-err ~0.21-0.25 (turn-in-place at ~25% of
the commanded 0.3 rad/s; training env/walk_yaw_err ~0.25) and slip
~3.6 (env/walk_loadslip_ratio 3.55-3.73, priced at ZERO:
walk_loadslip_gate=0, k_loadslip_excess=0 across the entire family).
Recalibration didn't cause the miss; it removed the fall-noise hiding
it. FORK DECIDED: (a) pricing nudge, bars UNCHANGED (0.25 tip err =
17% of commanded rate; eval_yaw's own gate is 0.1 — loosening would
bless real sluggishness). Launched a 4-arm single-lever dose grid on
the pushcal518 base (semantics bank 23/23 green this cycle):
`-yawprice2/-yawprice3` (k_yaw_prog 2.0/3.0 — safe now that
overshoot-decay+EMA price out the over-spin farm the pre-decay
yawprice3x died on) and `-slipexcess6/-slipexcess12`
(k_loadslip_excess 6/12 at bank-calibrated loadslip_ok=1.5, the never
-yet-trained V5 anti-skate charge; ~0.5-1.0/tick pressure vs ~3/tick
walk income). Each arm's gate: 0/12 raw falls (safety must hold) +
its own m5 section bar + no cross-regression; joint FAIL of a pair
refutes pricing as that axis's lever and escalates to mechanism, not
dose. Do NOT re-launch this grid; triage the 4 results next. Prior
banner below.)

Previous entry (~11:1x: **SEED-ROBUSTNESS GRID CLOSED AT
n=3: 3/3 RECALIBRATED SEEDS FALL-FREE ON THE FULL TURN+FAULT+PUSH
COMPOSITION — `pushcal518` (dr.ext_push_n 5-18N) IS NOW THE LINEAGE'S
SAFETY BASE; the 10-25N range is RETIRED on turnfault-seq1
descendants.** `cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-
pushcal518-seed13` (third and final pre-registered twin) VERDICTED
PASS: raw `terminated` False on all 12/12 own-cfg DR-0 episodes
(0/12 real falls, roll_peak max 14.2deg); the SAME seed at the old
range fell at `walk/det/3` (TERM tilt_roll 36.3deg) — here det/3 is
clean at 6.0deg, video-confirmed. Grid tally: seed7 0/12, seed23
0/12, seed13 0/12, vs 4-of-6 seeds unsafe uncalibrated. Same benign
caveats as its twins (one sto sacrificed-leg episode, 2
near-stationary sto episodes). SKILLS.md row added. **SAME CYCLE,
M5-SUITE DISCRIMINATOR ANSWERED (seed13 `eval_amp_m5` walk+yaw
sections, pod train-1): the marginal walk/yaw misses REPLICATE on the
second seed — this is a SYSTEMATIC recalibration trade, not basin
noise (option b ELIMINATED).** seed13: walk translating det slip med
~3.82 vs bar 3.5 (seed7 3.67, un-recalibrated parent 3.36 clean);
yaw tip_left/right 0.2168/0.2269 vs bar 0.20 (seed7 0.2157/0.2351,
parent 0.162/0.184) — same direction, same magnitude, on BOTH axes,
zero falls/terms in either section (walk roll_peak max 5.7deg,
gait_valid 12/12). The 5-18N push range buys real safety (0 falls,
fault-valid gait) at a small but real cost in walk-slip and yaw-tip
margin against the two strictest bars in the suite. Remaining fork —
(a) a walk/yaw-side pricing nudge at 5-18N to recover the margin vs
(c) a tolerance-band ruling on the never-operator-specified 0.20/3.5
bars (q_20260823T0130Z) — is a reward/gate change and belongs to a
dig-in cycle (DIG-IN flagged). seed13's push/fault M5 sections
completed on a re-run (first pass was killed mid-push by this cycle's
own process handling, rc=-15; walk/yaw artifacts complete and valid):
push PASS (0 terms, gait_valid 12/12, det slip 3.17), fault PASS
(0 terms, gait_valid 11/12, sacrificed leg {2}, det fwd 0.584) —
same profile as seed7. CAUTION: the re-run's `m5_verdict.json` says
`m5_pass=true` but contains ONLY the push/fault sections (the --skip
merge did not pick up the walk/yaw artifacts) — the honest full-suite
read for seed13 is m5_pass=FALSE on walk+yaw, identical to seed7. Do
not cite that file's m5_pass. Do not re-launch the seed grid or any
m5 section for seed13.** Prior banner below.)

Previous entry (~10:5x: **SEED-ROBUSTNESS TWIN #2 (seed23)
CONFIRMS THE PUSH RECALIBRATION FIX ON THE FULL TURN+FAULT+PUSH
COMPOSITION.** `cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-
seed23` (finished this cycle) VERDICTED PASS: raw `terminated` field
False on all 12/12 own-cfg DR-0 episodes (0/12 real falls), matching
the already-PASSed seed7 parent's own profile almost exactly (prog/
slip medians within 0.1 of each other, same benign walk/sto/4
sacrificed-leg-not-a-fall signature); the same seed fell 2/12 at the
old 10-25N push range. 2/2 recalibrated seeds now clean. `-seed13`
(third twin, another cycle's pod) still training — that result closes
the pre-registered n=3 grid; do not re-launch this grid. If seed13
also passes, promote `pushcal518` as the lineage's new M5-candidate
safety base and retire the 10-25N push range on turnfault-seq1
descendants. SKILLS.md row added. Prior banner below.)

Previous entry (~10:3x: **PUSH-FORCE RECALIBRATION FIX CONFIRMED
ON A FRESH RETRAIN: 0/12 real falls, transfers past eval-time
clamping.** This cycle triaged `cw-amp-m4-turnfault-seq1-pushcont1-
pushcal518` (single-lever respec of `pushcont1`: `dr.ext_push_n`
10-25N -> 5-18N, same seed=7/2M/fresh-init recipe) launched last
cycle to test whether the eval-time bisection finding (0/12 falls at
this narrower range on the OLD pushcont1 checkpoint) would ALSO hold
when the policy is actually TRAINED under the new range, not just
evaluated under it. Raw per-episode `terminated` field (not
`gait_valid`, confirmed again to never zero on a TERM): **0/12 real
falls**, every roll_peak <=13.3deg, INCLUDING `walk/det/3` (the exact
episode index where 5/6 turn+push+fault seeds toppled in the seed-
safety batch below) — clean at 11.0deg here. Direct baseline
(`pushcont1` itself, same recipe, untouched 10-25N range): 4/12 real
falls, roll_peak up to 27.4deg. VERDICTED PASS. **This is the fix,
not just a diagnosis** — recalibration is a real, sufficient
intervention for the fault+push (no turn) tier, and it works through
training, not merely as an eval-time force cap. Full writeup +
SKILLS.md row: see `SKILLS.md`.

**REFILL, same cycle (batch, 3 arms, all VERIFIED RUNNING, capacity
was 12/12 free):** the open question is whether this ALSO fixes the
FULL turn+fault+push composition (the tipfrac05 family, where 6/7
seeds fell at the old push range, mostly at this same det/3-style
episode) — a single-lever respec of `tipfrac05` itself
(`dr.ext_push_n` -> 5-18N, seed=7) plus 2 seed-robustness twins
(seed23, seed13) so the mechanism question and the seed-robustness
question are both answered in one batch instead of the 4-cycle serial
seed-farming the operator flagged 08-22:
`cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518` (seed=7,
already finished — off-limits, claimed by a concurrent cycle at
prestage time), `-seed23` (running, train-0), `-seed13` (running,
train-1). PASS on all three (0/12 falls each) would clear BOTH named
M5-candidate-promotion prerequisites' safety half at once; any
failure pattern (especially a repeat of det/3-style falls) would show
turn-in-place adds independent fall risk beyond push magnitude, not
just inheriting it. Do not re-launch this exact grid; triage the 3
results next.

Previous entry (2026-08-23 ~09:5x-c (**ROOT CAUSE TRACED PAST THE
7-SEED BATCH: the fall risk PRE-DATES turn-in-place AND fault, and
traces to PUSH.** Same raw-`terminated` audit (see the ~09:5x-b/09:5x
entries below), extended one step further back the lineage: checked
the actual ANCESTOR checkpoints' own-cfg DR-0 gate reports for raw
falls, not `gait_valid`. `cw-amp-m4-turnfault-seq1` (fault ONLY, no
push, the recipe's own root) is genuinely 0/12 falls — clean. The
instant push composes in, `cw-amp-m4-turnfault-seq1-pushcont1`
(fault+push, no turn-in-place yet) shows **4/12 REAL falls** that its
own same-day PASS-partial verdict never counted (it read "gait_valid
10/12 >= 9/12" as the safety floor — the identical metric bug). Its
`-ypfix1` yaw-pricing respec (an unrelated lever, same fault+push)
roughly halves that to 2/12 — also uncounted at the time. Going back
even further, an M3-era, fault-FREE, push-ONLY checkpoint
(`cw-amp-m3-pushcur1-noamp-b1530`) already falls 2/12 (`walk/det/3`
tilt_roll 40.9deg — the SAME episode index the whole tipfrac05 family
falls at — plus `walk/sto/0` tilt_pitch). **CONCLUSION: this is not a
turn-in-place-curriculum problem or a kernel-EMA-pricing problem — push
-disturbance recovery itself has carried an uncounted ~15-30% real
fall rate across the ENTIRE M3/M4 push-composition lineage since
BEFORE fault injection and BEFORE any turn-in-place lever existed.**
Everything built today to fix "turn+push erosion" (yaw-kernel-EMA,
overshoot-decay pricing, turn-in-place dose grids) has been layered on
top of, and did not cause, an already-shaky push-recovery foundation.
`dr.ext_push_n=(10,25)` N over 0.15-0.4s pulses, random direction/
timing (`domain_rand.py` `RandRanges`) — NOT YET DETERMINED whether
push magnitude, push-timing-vs-gait-phase (mid-swing vs mid-stance),
or genuinely undertrained recovery behavior is the actual mechanism;
that trace is the next dig-in step, ranked AHEAD of any further
turn-in-place or kernel-EMA arm on this lineage. **ACTION: every
push-composition verdict logged before this correction (`pushcont1`,
`ypfix1`, every M3 `pushcur*`/`pushhard*` arm, all tipfrac0x/seed
arms) should be re-read for raw fall count via `ops.sh report`'s
`TERM`/`terms N` output, not `gait_valid`, before being cited again as
a safety baseline or promoted.** Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_{,pushcont1,pushcont1_ypfix1}_gate/report.json`,
`logs/ckpt_eval/cw_amp_m3_pushcur1_noamp_b1530_gate/report.json`.

**CONFIRMED BY DIRECT ISOLATION (same cycle, 2 extra eval-only reads
on the controller CPU, zero GPU/training spent, checkpoint
unchanged):** re-ran `pushcont1`'s own 12-episode own-cfg panel with
ONLY `dr.ext_push_prob` flipped 1.0->0.0 (fault stays on) — **0/12
real falls**, every roll_peak <=18.5deg, `walk/det/3` specifically
drops from a TERM (tilt_pitch, 15.5deg) to roll_peak 4.0deg, clean.
The complementary swap (`dr.fault_prob=0`, push stays on) still falls
once (`walk/sto/0` tilt_pitch) though not at `det/3`. **Push is the
necessary driver of real falls on this checkpoint; fault only shifts
WHICH episode is vulnerable.** This is now a two-line reproduction,
not a hypothesis: any future push-composition arm should sanity-check
`dr.ext_push_prob=0` vs `=1` on its own panel before trusting
`gait_valid` as a safety signal. **ANSWERED, same cycle, one more
eval-only read: it IS a magnitude threshold, not a blanket recovery
failure.** Halving the push-force range (`dr.ext_push_n` 10-25N ->
5-12N, fault still on, same checkpoint): **0/12 falls, every
roll_peak <=8.0deg** (vs up to 41.3deg at full force) — clean
recovery every time at half force. Push-recovery behavior is real and
works up to roughly half the trained range; it specifically fails
somewhere in the 12-25N band. Next dig-in step is now precisely
bounded: bisect the actual threshold within 12-25N (a few more
eval-only reads, no training needed yet), THEN check push timing vs
gait phase and whether any `reward.*` term prices recovery-from-push
at all (unaudited), THEN decide recalibrate-the-range vs. add-
recovery-training-exposure/pricing at the top of the current range.
**BISECTED ONE MORE STEP (3rd eval-only read):** `dr.ext_push_n=
12-18N` (upper-mid of the original 10-25N range) also lands 0/12 real
falls, but roll_peak rises to 20.7deg (close to the 25deg term
threshold) and two `sto` episodes start SACRIFICING a leg instead of
falling — a visible transition zone. Threshold BRACKETED: clean
through ~18N, real falls appear by 25N (the range's own top end) —
tight enough to recalibrate `dr.ext_push_n` toward ~(5,18) as an
immediate low-risk mitigation while recovery-training/pricing is dug
into separately. Artifacts:
`logs/ckpt_eval/diag_pushcont1_{nopush,nofault,halfpush,midpush}/report.json`
(gitignored, controller-local diagnostics, not ledger evals).
Prior banner below.)

Previous entry (2026-08-23 ~09:5x-b (**7th AND LAST SEED CLOSES THE
BATCH: `-seed43` is the ONLY genuinely fall-free checkpoint of the
7, and it STILL nearly falls on the same hard episode.** Own-cfg
DR-0 gate: gait_valid 11/12 (1 sacrificed leg, walk/sto/4, a
wobble-and-recover on video, not a topple), and — checked via the
raw `terminated` field per the correction below, not `gait_valid` —
ZERO real falls, the only seed of 7 with none. But its OWN
`walk/det/3` episode (the exact index where 5/6 other seeds fall)
reads `roll_class=recovered`, `roll_peak_deg=17.6` — the highest
non-terminal roll peak anywhere in its 12-episode panel, i.e. it
skirted the same near-fall the other seeds didn't survive. Verdicted
PASS (own criteria: safe). Corrected 7-seed tally now FINAL:
seed7=2 falls, seed23=2, seed13=1(+3 sacrificed legs), seed31=1,
seed37=1, seed41=3, seed43=0 (near-miss) — 6/7 seeds have a real
fall, 1/7 clean. This closes the seed-safety-variance batch at a
worse-than-either-prior-estimate rate (not ~33%, not ~67% —
effectively "almost every basin, including the champion, is
vulnerable to one specific held-out maneuver") and reinforces the
~09:4x/09:5x correction below: the lever to pull next is understanding
and re-pricing/re-sampling that ONE maneuver (frozen/weak-leg fault +
turn-in-place, per video), not more seeds and not the
budget-stability/kernel-EMA track. Do not promote tipfrac05 as an M5
candidate, and do not spend further seed arms on this exact recipe —
the open question is now single-episode root cause, dig-in-tier.
SKILLS.md not touched (no bar newly cleared; this refines an existing
row's caveat). Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_seed43_gate/report.json`,
`..._seed43_gate/walk_sto_4_sheet.png`. Prior banner below.)

Previous entry (2026-08-23 ~09:5x (**CORRECTION to the ~09:4x entry below:
the "2 SAFE / 4 UNSAFE seed lottery" framing is WRONG -- the champion
seed7 (`tipfrac05` itself) and its seed23 twin (`-s2`) are NOT
fall-free either, and the falls that DO occur cluster overwhelmingly
on ONE fixed deterministic eval episode, pointing at a specific hard
command/maneuver, not per-basin luck.** Went back into the raw
`report.json` per-episode `term_reason`/`roll_peak_deg` fields (not
just the `gait_valid` scalar, which never zeroes on a TERM -- the same
harness quirk flagged below) for ALL SIX completed seeds, because the
09:4x batch's own "SAFE = matches seed7/seed23's clean profile"
language turned out to be asserted, not verified this cycle. Actual
raw fall counts on the 12-episode hazard-free own-cfg DR-0 gate:
seed7(tipfrac05)=**2** (det/3 tilt_roll roll_peak 39.3deg, det/5
tilt_roll 41.3deg), seed23(s2)=**2** (det/3 tilt_roll 35.4deg, sto/5
tilt_pitch), seed13=**1** (det/3 tilt_roll 36.3deg), seed31=**1**
(det/3 tilt_roll 38.4deg), seed37=**1** (det/3 tilt_roll 32.3deg),
seed41=**3** (det/2 tilt_pitch, sto/0 tilt_pitch, sto/5 tilt_roll --
notably NOT det/3). **FIVE of six checkpoints fall at the exact same
deterministic episode index (`walk/det/3`), all `tilt_roll`, all in a
tight 32-41deg roll-peak band** -- with eval `--seed 0` fixed and
per-mode command draws deterministic, episode index 3 is the SAME
commanded trajectory for every checkpoint in this family (frame
strips for seed7/s2/seed37/seed31 all show clean walking for the
whole strip then an abrupt topple on the last visible frame -- same
shape, not independent random tips). Only seed41 escapes det/3
(prog_ratio 1.172, no term there) but is the single worst checkpoint
overall (3 falls elsewhere) -- consistent with "avoided the common
failure maneuver by accident, not by being more robust." **REVISED
CONCLUSION: this is not primarily a training-seed safety lottery --
it's a near-universal (5/6) failure on one specific hard maneuver
in the fixed eval script, which nobody had actually read per-episode
term_reason for before (prior PASS verdicts on tipfrac05/s2 reported
"gait_valid 11-12/12" and, for s2, explicitly logged "2 isolated
terminations" but treated them as sub-threshold noise rather than a
shared, reproducible failure point).** This makes the problem MORE
tractable, not less: replay `walk_det_3.mp4` across the family to
identify the exact maneuver (heading/speed transition timing) next,
then check whether it's under-sampled in `stress_mix` training --
likely a real, buildable training-exposure fix, not a basin-luck
problem needing n=12 seeds to characterize. Does NOT reverse seed41's
or seed37's FAIL verdicts (both are still the worst of the batch by
raw fall count, 3 and 1-with-worse-severity-elsewhere respectively,
and both verdicts already used per-episode video evidence, not the
flawed gait_valid-only framing) -- it corrects the CHAMPION's own
safety claim (tipfrac05 is NOT zero-fall, 2/12) and reframes the open
blocker for the next (dig-in-tier) cycle. SKILLS.md not touched pending
that re-read (a PASS-record correction, if warranted, is dig-in work,
not a snap edit here). Evidence: same `..._{tipfrac05,s2,seed13,
seed31,seed37,seed41}_gate/report.json` + `walk_det_3.mp4` across all
six. Prior entry below.)

Previous entry (2026-08-23 ~09:4x (**SEED-SAFETY BATCH CLOSES AT n=6:
2 SAFE / 4 UNSAFE (~67%) -- the seed-safety-variance risk is now the
CONFIRMED DOMINANT blocker on tipfrac05 promotion, not a minor
caveat alongside the budget-stability question.** Triaged 2 of the
4-arm seed31/37/41/43 batch launched to pin the true unsafe rate
after seed13/s3 found 1/3 seeds unsafe: `-seed41` UNSAFE, and WORSE
than any prior seed -- own-cfg DR-0 gate's gait_valid metric nominally
reads 6/6 det + 6/6 sto (a harness quirk: it does not zero out on a
TERM), but the raw per-episode report + frame strips show THREE
separate video-confirmed topples (walk/det/2 tilt_pitch, walk/sto/0
tilt_pitch, walk/sto/5 tilt_roll), with no sacrificed-leg workaround
this time (sac=[] everywhere) -- a clean stability failure, not a
degraded-gait compromise. Also picked up `-seed37`, an ORPHAN whose
prestage never fired (no pullckpt/pod-evals in orchestrator.log,
unclaimed by the concurrent cycle) -- ran podeval by hand on its own
pod: also UNSAFE, 1 video-confirmed fall (walk/det/3 tilt_roll),
matching the pattern read (context only, not verdicted here) on the
concurrent cycle's `-seed31` (also 1 fall, det/3 tilt_roll). Combined
tally across all 6 completed seeds (7, 23, 13, 31, 37, 41): SAFE =
{7, 23}, UNSAFE = {13, 31, 37, 41}. This overturns the earlier n=3
read of "roughly 1-in-3 basins unsafe" -- **it's closer to 2-in-3.**
`-seed43` (batch's 4th arm) still training. **CONCLUSION: seed-safety
variance is not a rare edge case to root-cause "eventually" -- most
seeds sampled from this exact recipe (turn-in-place curriculum +
push+fault composition, 2M, seed=7's own hyperparameters) produce a
checkpoint that falls on a hazard-FREE walk eval. tipfrac05 (seed7)
remains a real PASS on its own gate, but the RECIPE cannot be called
seed-robust, and nothing past the single seed=7 checkpoint should be
promoted toward M5-candidate status until this is root-caused.** This
supersedes the priority ordering in the two named prerequisites below
(budget-stability, seed-safety variance) -- seed-safety is now the
more urgent of the two given the failure rate. No SKILLS.md update
(both are FAIL verdicts, no new bar cleared). Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_
{seed31,seed37,seed41}_gate/`. Prior banner below.)

Previous entry (2026-08-23 ~09:1x (**KERNEL-EMA QUESTION CLOSED AT n=5:
0/5 arms rescue yaw-tracking in either regime, and the fixed-basin
continuation test CONFIRMS a real transition-handling defect, not
fresh-retrain basin noise.** The two continuation arms launched off
the 3-arm decomposition grid's inconclusive close (below) both FAIL:
`-kernelema-cont1` (yaw+vel EMA, +6M from the tipfrac05 checkpoint
itself) tips 0.2385/0.3168 — worse than the NON-EMA `-acq1`
continuation's own erosion (0.2038/0.2692) on both signs, safety
floor unchanged (gait_valid 11/12, 1 sacrificed leg, 0 falls, same as
acq1). `-acq-kernelema` (identical lever, respec of `-acq1` directly)
tips 0.2659/0.2901 — worse still, AND adds a genuine safety
regression: 3 termination events across 12 DR-0 episodes
(video-confirmed topples) plus 1 sacrificed leg, vs tipfrac05's and
acq1's own zero falls. Both arms' isolated `eval_amp_m5`
walk/push/fault sections PASS and are each a touch better than
acq1's own numbers — kernel-EMA is not globally harmful, it
specifically fails (and here actively worsens) the one axis it
targets. Because BOTH continuations regress on the SAME fixed basin
acq1 already occupies, this is the exact discriminating test the
08:3x REFINED MECHANISM note asked for: pure basin noise predicted no
effect; a real transition-handling defect predicted continued
regression. It regressed — CONFIRMED real defect. Root cause
(diagnosed, still unbuilt): the EMA'd kernels never reset on a
`goal.vx_ref`/`wz_ref` command change, teaching a "damp body dynamics
for ~tau after any translation-heavy segment ends" habit that costs
nothing in ordinary stress_mix training but suppresses exactly the
abrupt heading-authority `eval_amp_m5`'s tip-left/right segments
measure. **Do not fund any further naive kernel-EMA arm on this
lineage.** Both M5-candidate-promotion prerequisites (budget-
stability, seed-safety variance) remain OPEN. Next concrete build:
the targeted command-transition-aware EMA reset (3 env variants +
`MJX_SNAPSHOT_EXTRA` + tests, real dig-in-cycle scope) or a
structural hold/forward income-repricing lever that skips kernel
smoothing entirely. Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_
{acq_kernelema,kernelema_cont1}_{gate,m5}/`. Prior banner below.)

Previous entry (2026-08-23 ~08:1x (**hold/forward income-repricing
(q_20260823T0240Z item b): FIRST MECHANISM BUILT + BANKED, not just
assumed — a 3-arm decomposition grid is now running to measure it.**
Two concurrent cycles this window both independently confirmed the
repricing build was needed but explicitly deferred it to a dig-in
cycle rather than build blind (see banners below). This cycle used
`probe_walk_income`'s own numbers (`hold_forward_income_ypfix1.json`:
reward_walk_yaw hold=374 vs forward=203 vs tip_left/right=132/147 per
15s segment) to find a concrete, precedented mechanism: the SAME
sway-tax defect the joystick track's `reward.walk_kernel_vel_ema` fix
(phasedir7/7b/8) repaired for the linear-velocity kernel exists,
unfixed, on the yaw-rate kernel too — a genuine full-stop command
pins wz at ~0 with near-zero variance (nothing moving), so the
instantaneous Gaussian sits at its peak almost every tick, while an
honestly-tracking turner's wz oscillates stride-to-stride around its
achieved mean even when that mean is correct, so the kernel never
pays it as richly as standing still. Built `reward.walk_kernel_yaw_
ema` (+`walk_kernel_yaw_tau_s`), the yaw-axis mirror of
`walk_kernel_vel_ema`, narrowly scoped (only the kernel's own err
term is smoothed; the already-on achieved-rotation/achieved-progress
gates and k_yaw_prog/k_yaw_still keep reading raw instantaneous wz,
so correctness gating is untouched). MECHANISM CONFIRMED on scripted
references before spending any GPU budget: a "tracked" reference
(achieves ~= the commanded rate with real stride oscillation) and a
"turn" reference (achieves only ~half the command, lower variance)
are STATISTICALLY TIED on the raw yaw kernel alone (153 vs 159/ep
return, n=6 seed/sign draws) — no exploitable gradient toward better
tracking on this channel — but cleanly SEPARATE with the EMA on (197
vs 155/ep); the structural fixed-drift reference does not benefit
(115.9->113.1, slightly down). 3 new/updated turn-bank tests added
(`test_task_semantics.py`), default-off bit-exact verified, full bank
re-run clean: 203 pass / 1 pre-existing known-red (`fastprof`,
unrelated) / 4 skip / 1 xfail. Snapshotted
(`exp/exp-amp-m4-walk-kernel-yaw-ema`). **LAUNCHED a 3-arm
decomposition grid off tipfrac05 itself** (seed=7 matched, 2M,
byte-identical recipe otherwise, single-lever discipline per arm):
`-kernelema1` (both axes on, the full bundle — FINISHED already,
~9 min wall, eval pending next cycle), `-kernelema-yawonly` (yaw axis
only, RUNNING train-0), `-kernelema-velonly2` (translation axis only,
RUNNING train-1; a `-kernelema-velonly` duplicate is also INTENT on
train-2 from a launcher-drain retry of an earlier pod-collision
REFUSED — harmless extra n, both may finish). Pre-registered joint
read: IMPROVED (tips tighter than tipfrac05's own 0.162/0.184 with
safety held) funds stacking with a budget-continuation retry of the
acq1 erosion question; FLAT (within noise) means kernel noise-tax is
real but small next to the gap, escalating to the harder untouched
piece (actuation-cost asymmetry: current/gyro/roll price 4-10x higher
on real motion than standing still, not addressed by this arm); WORSE
triggers a revert/re-scope. This does NOT resolve the seed13/s3
safety-seed-lottery finding (below) — that stays flagged for its own
dig-in. Evidence: `logs/probe_walk_income/hold_forward_income_
ypfix1.json` (mechanism source), `rl_move/tests/test_task_semantics.py`
(TURN_YAWEMA_OVERRIDES + 3 tests). Prior banner below.)

Previous entry (2026-08-23 ~07:5x (**TURN-EXPOSURE DOSE GRID CLOSED at
4 points (0.2/0.3/0.5/0.7): 0.5 is the confirmed peak, not the
largest dose tried.** `cw-amp-m4-turnfault-seq1-pushcont1-tipfrac07`
(dose 0.7, the grid's last untried rung) VERDICTED INFORMATIVE — the
pre-registered TURNOVER branch fired cleanly: yaw section tips
regressed to 0.236/0.225 (worse than tipfrac05's bar-clean
0.162/0.184, back into the tipfrac02/03 in-band-not-clean range) —
MORE dedicated turn-episode exposure made tip-tracking WORSE past
0.5, not better. Walk section also erodes: `n_translating` collapsed
to 2/12 (0 of 6 det episodes were translating at this dose — a real
harness-sampling gap for future high-turnfrac arms, not just a
training defect — so `det_prog_med`/`det_slip_med` are
null/unjudgeable) and the 2 translating episodes that did land both
miss the slip bar (4.03/4.30 > 3.5). Own-cfg DR-0 gate regresses too:
gait_valid 10/12 with TWO video-confirmed tilt_roll falls (roll peak
35.0/34.6deg) plus 2 sacrificed-leg sto episodes — vs tipfrac05's
clean 12/12/zero-falls — a dose-driven safety cost independent of
the `-s3` seed-lottery finding (same seed=7 as the champion here).
Fault section is the one axis that holds (10/12 gv, meets bar,
slightly better than tipfrac05's post-fix 9/12) — not enough to
offset yaw+walk+safety. **Grid CLOSED, champion UNCHANGED:
`tipfrac05` (seed7, turn_in_place_frac=0.5) remains the sole
M5-candidate; no further dose arms on this lever.** The two
already-flagged prerequisites for actual M5-candidate promotion —
hold/forward income repricing (budget-stability) and seed-safety-
variance root-cause — are unaffected by this result and remain the
funded path (in progress on a concurrent DIG-IN cycle at review
time; left untouched, no duplicate work). Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac07_{gate,m5}/`.
Prior banner below.)

Previous entry (2026-08-23 ~07:3x (**THIRD SEED (`-s3`, seed13) CLOSES
the tipfrac05 grid at n=3 and finds a SAFETY seed-lottery, not a
tip-tracking one.** Picked up this run too (its prestage never fired;
ran gate+m5 by hand). eval_amp_m5 tips 0.218/0.228 — IN-BAND
(<=0.25), so 3/3 seeds now land in-band on tip-tracking (seed=7
bar-clean 0.162/0.184, seed23 0.207/0.228, seed13 0.218/0.228): that
half of the recipe genuinely generalizes. But own-cfg DR-0 gait_valid
crashes to 7/12 for this seed — ONE video-confirmed tilt_roll FALL
(roll peak 30.2deg, clean topple on video, not a flag artifact) plus
THREE separate sacrificed-leg episodes — vs seed7/seed23's clean
12/12, and the m5 fault section also fails here (gait_valid 9/12 vs
bar 10, three sacrificed legs vs the usual two). VERDICTED FAIL.
**CONCLUSION: the turn-in-place curriculum's tip-tracking gain is
seed-robust; its walking SAFETY margin is not — roughly 1-in-3 basins
at this recipe lands unsafe even in the hazard-zeroed walk section.**
This is a second, independent open risk on the tipfrac05 lineage
alongside the already-flagged hold/forward income-repricing need
(see the `-acq1` erosion result below): before any M5-candidate
promotion, seed-safety variance needs either a larger n or a
root-cause (why does seed13's basin sacrifice legs/fall even
hazard-free?) — flagged for the next dig-in cycle alongside the
repricing fork, not launched blind here. SKILLS.md row amended
again. Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_s3_{gate,m5}/`.
Prior banner below.)

Previous entry (2026-08-23 ~07:2x (**SEED-TWIN + BUDGET-CONTINUATION
TRIAGE on tipfrac05's two named-in-flight children (this cycle's
own): `-s2` (seed23, 2M) VERDICTED PASS — eval_amp_m5 tips 0.207/
0.228, inside the run's own PASS band (<=0.20-0.25) though not
bar-clean like the seed=7 original (0.162/0.184); safety floors held
(own-cfg DR-0 gait_valid 12/12), push section PASS clean, fault
section PASS at the bar (gait_valid 10/12). 2/2 seeds now read
in-band (`-s3`/`-seed13` still pending, owned by a concurrent cycle)
— the turn-exposure recipe is looking reproducible, not a one-seed
lottery win, though only the original seed hit the strict 0.20 line.
`-acq1` (+6M steps warm from the tipfrac05 checkpoint itself)
VERDICTED FAIL — tips ERODED to 0.204/0.269 (worse than the 2M
parent on BOTH signs, tip-right badly over the 0.20 bar) while
training reward rose 110->226/ep across Q1-Q2 then sat FLAT at
226/226/230 for the rest of the run — the exact pre-registered
ERODED branch, not a flat-reward stall. Composition floors held
(own-cfg DR-0 gait_valid 11/12, m5 push/fault sections both PASS,
fault gait_valid even IMPROVED 9->11/12 vs the 2M parent). Video
clean six-leg cycling both arms, one legitimately carried fault leg,
zero falls. **CONCLUSION: exposure-only (the tipfrac curriculum
lever) is now measured on BOTH axes it needed to survive — a second
seed (holds, in-band) and more budget (does not hold, erodes) — and
the budget axis fails exactly as the mechanism predicts (turning
costs 4-10x more in current/gyro/roll than hold/forward per
`probe_walk_income`, so undirected extra training re-drifts toward
the cheaper income once the curriculum's initial exposure boost is
exhausted).** Per q_20260823T0240Z item (b), hold/forward income
repricing is now CONFIRMED NECESSARY (not merely assumed) as the
next M4 turn+push lever before promoting anything past the 2M
tipfrac05 recipe itself; do not spend further budget/seed arms on
this exact recipe. SKILLS.md row amended with both results. Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_{s2,acq1}_m5/`,
`..._{s2,acq1}_gate/`. Prior banner below.)

Previous entry (2026-08-23 ~07:1x (**CORRECTION to the 06:5x headline
below, from the post-fix ISOLATED `eval_amp_m5` re-reads (run on
train-8/9 with all three harness fixes; aliased originals preserved
at `..._m5_aliased/` / `..._m5_prefix/`): tipfrac05 is NOT a full M5
pass — its `m5_pass=true` was scored with push/fault still ALIASED
(byte-identical composed reads, the very defect fix #3 repaired).
Isolated re-read: walk PASS (12/12 gv, 0 terms, translating slip
3.36), yaw PASS (tips 0.162/0.184 — the cell-closing result stands,
first composed checkpoint to pass the m5 yaw section), push isolated
PASS (12/12 gv, 1 det term), fault isolated FAIL BY ONE EPISODE
(gait_valid 9/12 vs bar 10; each flagged episode = one parked
carried-fault leg, video-clean upright walking, zero falls/terms;
tipfrac03's isolated fault flags 2/12 and passes) → `m5_pass=false`;
the SOLE remaining M5-suite gap is isolated fault-section gait_valid,
and it is noise-scale at n=12. tipfrac03's isolated re-read is the
mirror image: walk/push/fault ALL PASS, yaw alone misses by 0.0008
(0.2008/0.2135 vs 0.20). Ledger verdicts recorded this cycle:
tipfrac05 PASS (cell CLOSED per its pre-registered PASS-clean branch
— the M4 turn+push cell closure STANDS; only the M5-suite-green claim
is corrected), tipfrac03 PASS (0.25 band). SKILLS.md row amended to
the corrected claim. Refills launched+finished this cycle (triage
owned by next cycles): `-s2` (seed 23), `-s3` (seed 13), `-acq1`
(+6M warm from tipfrac05), `-tipfrac07` (dose 0.7 bracket); the
06:5x cycle's `-seed13` duplicates -s3's seed (harmless, extra n).
M5-candidate operator question q_20260823T0700Z filed and updated
with this correction; fault-gv adjudication joins the
q_20260823T0130Z mixed-fault-curriculum question the faultdose grid
tests. Prior banner below.)

Previous entry (2026-08-23 ~06:5x (**HEADLINE: `cw-amp-m4-turnfault-seq1-
pushcont1-tipfrac05` is the FIRST-EVER checkpoint to pass the whole
`eval_amp_m5` cross-engine suite (walk+yaw+push+fault all PASS,
turn-tip err 0.162/0.184, under the fault-only solo parent's own
0.18/0.17) -- VERDICTED PASS this cycle. Same lever that produced
tipfrac02/tipfrac03 (see banner below): `goal.walk_turn_in_place_frac`
(whole-episode dedicated turn PRACTICE, not pricing) at dose 0.5,
single lever vs `pushcont1-ypfix1`. Root cause the practice lever
targets: `probe_walk_income` this cycle showed a perfect heading-hold
segment earns ~1.7x a perfect tip-turn segment (1415 vs 842/796
return) purely from lower actuation cost (turning taxes current/
gyro/roll 4-10x harder than standing still), while turn-in-place is
only ~7.5% of independently-sampled training exposure -- too rare to
learn a skill that structurally can't out-earn hold/forward per
segment. This exact curriculum lever was tried and REFUTED once
before (tip50/tip90, pre-BC-turn-clone) but that was BEFORE RL had
any discoverable turning motor pattern at all; it had never been
retried post-clone until this grid. Clean monotonic 3-arm dose
response: frac 0.2 -> 0.207/0.234 (tipfrac02, INFORMATIVE), 0.3 ->
0.201/0.214 (tipfrac03, INFORMATIVE, misses m5's strict bar by
0.0008), 0.5 -> 0.162/0.184 (tipfrac05, PASS). CAVEATS: single seed
(seed=7) -- repro arms already in flight (`-seed13` this cycle's own
launch, FINISHED pending triage; a concurrent cycle's `-s2`/`-s3`
seed twins and `-acq1` budget continuation also running/queued, see
their own entries); push/fault sections are jointly-not-independently
tested (both hazards permanently baked in this checkpoint's cfg, the
known q_20260823T0130Z design tension) -- M6 hardware stays
[operator]-owned per tracks.json regardless. Also landed 3 small
`eval_amp_m5.py` correctness fixes this cycle, all discovered because
this is the FIRST lineage ever evaluated with
`walk_turn_in_place_frac>0` baked into a checkpoint's cfg: (1) a
`statistics.median` crash on turn-in-place episodes' legitimately-None
`progress_ratio`; (2) slip/fwd medians were being computed INCLUDING
turn-in-place episodes (a rotation-scrub artifact, not the "skating
while walking" defect those bars target) -- now restricted to
translating episodes only; (3) a `--skip` re-run used to overwrite the
WHOLE `m5_verdict.json`, silently dropping any earlier FAIL section
not recomputed this call and letting `m5_pass` go true on a partial
read (caught live: a concurrent `--skip walk,yaw` re-run of tipfrac02
briefly left its file reading `m5_pass=true` with yaw's real
0.2068/0.2341 FAIL erased; re-ran full and confirmed corrected file
now reads `m5_pass=false` as it should) -- `_prior_sections` now
merges into any existing verdict file instead of clobbering it,
5/5 new unit tests (`test_eval_amp_m5_merge.py`). None of these
changed the SUBSTANCE of any verdict already written (tipfrac02/03/05
were all read correctly the first time from full, non-skipped runs);
they protect the NEXT partial-re-run cycle from a false pass. Tags:
`exp/eval-amp-m5-none-median-fix`, `exp/eval-amp-m5-turnplace-
translating-filter`, `exp/eval-amp-m5-verdict-merge-fix`. SKILLS.md
row added for the tipfrac05 PASS. Evidence: `logs/ckpt_eval/
cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_{gate,m5}/`,
`logs/probe_walk_income/hold_forward_income_ypfix1.json` (controller
`/tmp/hold_forward_income_ypfix1.json`). Prior banner below.)

Previous entry (2026-08-23 ~06:2x (**`cw-amp-m4-turnfault-seq1-pushcont1-
tipfrac02` VERDICTED PASS-partial: turn-in-place EXPOSURE is a live
lever again post-BC-clone.** Lowest dose (frac 0.2) of the 3-arm
tipfrac grid: hazards-zeroed eval_yaw tips 0.2068/0.2341, improved
from ypfix1's 0.2471/0.2553 — pre-registered IMPROVED branch (both
inside 0.25 for the first time in the composed turn+push+fault
regime; tip-left misses the 0.20 clean bar by 0.007). Read-only peek
at the siblings (verdicts owned by the concurrent cycle) shows a
MONOTONIC dose-response: tipfrac03 0.2008/0.2135, tipfrac05
0.1620/0.1838 — 05 under the 0.20 m5 bar on both tips, likely closes
the M4 turn+push cell pending its own safety floors. This overturns
the pre-clone tip50/tip90 "exposure does nothing" refutation exactly
as the grid hypothesis predicted: exposure was useless only while
turning was undiscoverable. Safety floors held on tipfrac02 (own-cfg
gait_valid 11/12, m5 walk section 12/12 with 0 terms — best in
lineage). ALSO: found + FIXED an `eval_amp_m5` harness defect the
triage exposed — for permanent-hazard cfgs the push and fault
sections both ran fault+push composed (byte-identical episode data
judged against two different bars; tipfrac02's fault-section "FAIL"
was this aliasing + one sto term of noise, not a regression).
`eval_amp_m5.py` push/fault sections now force the OTHER hazard to
0.0 (fix #3, same last-wins pattern as the 08-23 walk/yaw fixes);
pre-fix push/fault sections of permanent-hazard runs are composed
reads, not comparable to post-fix isolated ones (aliased original
preserved at `..._tipfrac02_m5_aliased/`). Isolated per-mode-6
re-read of tipfrac02 push/fault DONE (fix fd018563): push PASS (1 sto
term, 12/12), fault PASS CLEAN (0 terms, 12/12, det fwd 0.641 m) —
the aliasing accounted for the entire fault-section "FAIL"; all of
tipfrac02's hold-and-report gate requirements are met (ledger
addendum recorded). No
SKILLS row (no new bar cleared on this arm; grid close-out rides on
tipfrac05's verdict). Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac02_{gate,m5,m5_aliased}/`.
Prior banner below.)

Previous entry (2026-08-23 ~05:4x (**`cw-amp-m4-turnfault-seq1-pushcont1-
ypfix1` VERDICTED PASS-partial: pricing helps, doesn't close the cell.**
Single-lever respec of pushcont1 (banked overshoot-decay/avg_s keys ON,
matched 2M, same pre-cheat turnfault-seq1 init): hand-run eval_yaw tips
0.2471/0.2553, down from pushcont1's own 0.2727/0.3029 (-9%/-16%,
closing on the fault-only parent's 0.1818/0.1708 ceiling) but
tip-right still clears the 0.25 PASS-clean bar by ~0.005 while
tip-left is already inside it -- lands in the pre-registered PARTIAL
branch by a hair, not PASS-clean. Safety floor also improved:
own-cfg DR-0 gait_valid 12/12 (pushcont1 was 10/12), zero falls,
video-clean six-leg cycling with the expected one legitimate
carried-fault-leg per episode. `eval_amp_m5`: m5_pass=false as
expected (push/fault PASS clean, walk fails on the already-documented
permanent-hazard design tension, yaw fails only the suite's generic
0.10 bar). CONFIRMS the escalation call below: pricing-correctness is
a real but insufficient lever on turn+push; per q_20260823T0240Z item
(b) the funded next M4 lever is hold/forward income-dominance
repricing, not another composition/dose/pricing-key arm on this same
recipe. SKILLS.md not updated (no new bar cleared, same precedent as
pushcont1's own PASS-partial). Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_ypfix1_{gate,m5}/`.
Prior banner below.)

Previous entry (2026-08-23 ~05:1x (**LAUNCHED: `cw-amp-m4-turnfault-seq1-
pushcont1-ypfix1` -- the push-composition dose sweep never tested
PRICING, only EXPOSURE.** The just-closed 3-arm dose sweep
(pushdose025/05b/075, see entry below) held training-time push
probability fixed at pushcont1's LEGACY yaw pricing throughout --
none of pushcont1 or its dose children ever carried the already-built,
already-banked overshoot-farm fix (`reward.yaw_prog_overshoot_decay`/
`reward.yaw_prog_avg_s`, landed 08-23 from the solo-turn income audit,
so far only tried on the unrelated turnpushfault1-style05/clamp-pinned
lineage's `ypfix1-r3`, which stayed bit-identical for a different,
clamp-quantization reason). Single lever vs pushcont1: both keys
0.0->1.0, same 2M budget, respec inherits pushcont1's own pre-cheat
`--init-from turnfault_seq1` checkpoint (init-basin rule satisfied
automatically). VERIFIED RUNNING on hexapod-mjx-train-9
(wandb tracked via ledger). Pre-registered branches: PASS-clean
(tips <=0.20-0.25, closes the M4/M5 turn+push cell outright),
PARTIAL (better than pushcont1's 0.27/0.30 but still >0.25 --
hold/forward repricing remains the next lever), FLAT (~unchanged --
refutes pricing too, leaving only the from-scratch-capacity /
operator-sizing branch of q_20260823T0240Z item b). Two launches also
observed in flight from a concurrent cycle this same window (not
touched by this entry): the M4 fault-training-dose grid
(`turnfault-seq1-faultdose025/05/075`, testing the walk-section design
tension named in q_20260823T0130Z) and cpg's closed-loop yaw-trim
build (`cpg-yawtrim-closedloop`, STATUS Next#2) -- left alone,
different tracks/axes, no overlap. Prior banner below.)

Previous entry (2026-08-23 ~05:0x (**COMPOSITION-ORDER AND PUSH-DOSE BOTH
CLOSED for M4 turn+push erosion; escalates the deferred hold/forward
income-repricing build.** `cw-amp-m4-turnfault-seq1-pushcont1`
(fault-then-push order, single lever dr.ext_push_prob=1.0 vs the
turnfault-seq1 parent) PASS-partial: own-cfg DR-0 floor clears
(gait_valid 10/12 >=9/12), but hand-run eval_yaw (hazards zeroed)
reads tip-left/right err 0.2727/0.3029 -- worse than the fault-only
parent's 0.18/0.17 and over the <=0.20-0.25 preserved band, though
roughly HALF the erosion of every push-FIRST composition order tried
(turnpush1-style05-acq1-r2 0.38/0.43, turnpushfault1-style05-r2
0.42/0.49, ypfix1-r3 0.39/0.47, cont1 0.41/0.41). Order is a real,
measured, but insufficient lever -- CLOSES the 3-way composition-order
search (direct graft / push-first / fault-first all tried, all erode
turn to some degree). Full `eval_amp_m5`: m5_pass=false (push section
PASS clean; walk+fault sections FAIL on the already-known permanent-
hazard design tension, q_20260823T0130Z -- not new). Follow-up 3-arm
TRAINING-TIME push-probability dose sweep (`-pushdose025/05b/075`,
dr.ext_push_prob 1.0->0.25/0.5/0.75, single lever, re-init from the
SAME pre-cheat turnfault-seq1 checkpoint per the init-basin rule) all
landed FAIL, clustered in the SAME 0.24-0.27 tip-err band regardless
of exposure fraction (dose025 0.264/0.269, dose05b 0.239/0.257 --
closest but still misses tip-right, dose075 0.261/0.255) -- a small,
non-monotonic improvement over pushcont1's full-dose 0.27/0.30 shared
by all doses, never reaching the fault-only parent's 0.18/0.17 or the
0.20 m5 bar. Safety floors and video all clean (gait_valid 11-12/12,
0-2 terms, clean six-leg cycling, legit single carried-fault legs).
**CONCLUSION: dose is not the lever either -- the mere PRESENCE of
push in the training distribution (not cumulative exposure fraction,
not graft order) drives the erosion.** This is real income competition
between push-recovery and honest turn-tracking, the same class of
problem the M2-yaw income audit already measured and partially fixed
elsewhere (hold-freeze richer than the honest tip ceiling, 1473 vs
1209/ep) -- the ALREADY-BUILT overshoot-decay pricing keys
(`reward.yaw_prog_overshoot_decay`/`yaw_prog_avg_s`) were tested
directly on a composed turn+push+fault checkpoint (`ypfix1-r3`) and
found INEFFECTIVE there (bit-identical clamp-pinned behavior) --
repricing hold/forward income dominance (the OTHER named-but-deferred
half of that audit, q_20260823T0240Z item b) is now the funded next
lever, not another composition/dose arm on this axis. Separately,
launched a matching 3-arm TRAINING-TIME fault-probability dose sweep
(`-faultdose025/05/075`, same pre-cheat re-init, dr.ext_push_prob
left at 1.0) testing the OPEN q_20260823T0130Z design-tension
question directly: does a mixed (not permanent) fault curriculum let
eval_amp_m5's walk section actually draw a fault-free episode
fraction and clear its own terms/gait_valid bar -- results pending
next-cycle triage (all VERIFIED RUNNING at cycle end, 2M steps each,
finish in ~1-2 min of GPU wall time). Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_{gate,m5}/`,
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushdose{025,075,05b}_m5/`.
SKILLS.md not updated (no new capability cleared a bar). Prior banner
below.)

Previous entry (2026-08-23 ~04:1x (**`cw-amp-m4-turnfault-seq1` PASS-preserved:
FAULT grafted sequentially onto the clean turn champion keeps tip
tracking almost intact (eval_yaw 0.1818/0.1708, vs parent's own
0.1525/0.1614) — decisively names PUSH, not axis-count, as the M4
turn-erosion driver (push composition eroded to 0.38/0.43+; a fresh
3-way stack parks totally at 0.30/0.30; this fault-only graft stays
near-parent). Safety floor gait_valid 11/12, 0 terminations, one
legit carried-leg (video-confirmed clean 6-leg cycling). Caveat: the
m5 suite's scripted stop-hold scenario recorded one deterministic
fall the parent didn't have — narrow single-trial anomaly, doesn't
move the tip-tracking verdict, flagged for a follow-up look. Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_gate/`,
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_m5/`. SKILLS.md row added.
**Refill: launched `cw-amp-m4-turnfault-seq1-pushcont1`** — grafts
PUSH onto THIS checkpoint (fault-then-push order) to test whether
sequencing push AFTER an already-turn-preserving fault graft avoids
the erosion turnpush1 showed when push was grafted directly onto the
clean turn substrate; if tips stay <=0.20-0.25 here too, fault-first
composition order becomes the M4/M5 recipe. Prior entry below.)

Previous entry (2026-08-23 ~03:3x (**TOOLING: `ops.sh m5eval` built +
run for the record on the two live M4 composition candidates —
`pushfault1-noamp-acq1` (best non-turn substrate) and
`turnpushfault1-style05-r2` (only all-4-axis checkpoint); both
m5_pass=false as expected, one real structural finding, no verdict
changes.** New tool (`rl_move/orchestrator/m5_pod_eval.py` +
`ops.sh m5eval <run> [pod]`, COMMANDS.md updated): derives a run's own
training cfg-set from the ledger exactly like `evalcmd`, syncs code to
the target pod, runs `eval_amp_m5`, copies `logs/ckpt_eval/<run>_m5/`
back — no more hand-rolled kubectl plumbing for this suite. Results:
(1) `pushfault1-noamp-acq1`: push and fault sections PASS clean; WALK
section fails ONLY on the zero-sacrificed-legs bar (`sacrificed=[0]`,
the same 1/12-episode carried-leg pattern its own PASS verdict already
named legitimate) — root cause: this checkpoint (like every M3/M4
push/fault arm to date) trains with `dr.fault_prob=1.0` on 100% of
episodes, so it has NO clean hazard-free walking mode for the walk
section to test; walk/push/fault sections read numerically IDENTICAL
here since nothing distinguishes them. Flags a real design tension for
what an M5-passing checkpoint's training curriculum needs to look like
(mixed hazard probability vs permanent-on) — filed as an amendment to
`q_20260823T0130Z`, not unilaterally resolved. (2)
`turnpushfault1-style05-r2`: third independent reproduction of the
known 0.4248/0.4932 tip-err (walk/yaw fail as already verdicted,
push/fault pass). While building the suite invocation, found
`eval_amp_m5`'s yaw section passed a checkpoint's baked
`dr.fault_prob=1.0`/`ext_push_prob=1.0` straight into `eval_yaw`
unfiltered instead of the zeroed hazards its own bar text assumes —
fixed (append explicit `dr.fault_prob=0.0`/`ext_push_prob=0.0`
overrides, last-wins per key) and then hand-verified the fix is
BEHAVIORALLY INERT on this checkpoint (bit-identical 0.4248/0.4932
with/without the override) because `eval_yaw` always builds its env
at `dr_scale=0` regardless of `--cfg-set`, which already routes those
guarded draws to zero — correctness/clarity fix only, no historical
number was actually confounded, no verdict needed re-opening. Previous
entry below.)

Previous entry (2026-08-23 ~03:4x (**cont1 (the matched no-keys control
named in the entry immediately below) FINISHED and VERDICTED
INFORMATIVE — completes the keys-vs-budget joint read, and REFINES
the FAIL-branch root cause: the safety erosion is the KEYS, not
continued training.** cont1 (8M total, same checkpoint/budget as
ypfix1-r3, reward pricing UNCHANGED) reproduces the identical
clamp-saturated tip attractor (`logs/probe_walk_income/
turnpushfault1_cont1_income_probe.json`: yaw_ratio 1.649/1.786,
wz_mean +0.478/-0.518 — matching r2's parent probe to float
precision, same as ypfix1-r3's own probe) and eval_yaw tips
0.4062/0.4104 (bar 0.20, still far over) — so BOTH arms agree the
overshoot-farm/clamp attractor is untouched by 6M more steps
regardless of pricing, closing that half cleanly. BUT the two arms
DIVERGE on safety: cont1's own-cfg DR-0 gate holds EXACTLY at r2's
own numbers (gait_valid 12/12 det+sto, zero new sacrificed legs,
topples 2/6 det + 0/6 sto = 2/12, identical to r2), while ypfix1-r3
(keys ON, same budget, same starting checkpoint) eroded to
gait_valid 10/12 with TWO new sacrificed legs. Since cont1 is the
byte-for-byte pricing-unchanged control and shows ZERO erosion, the
erosion the dig-in needs to root-cause is caused BY the overshoot-
decay/avg_s keys specifically (some interaction with the clamp-
pinned regime, not generic extra training) — narrows the dig-in's
search space. Other turn axes also moved for cont1 specifically:
fwd-hold spurious yaw worsened 0.157->0.264 rad/s and arc tracking
softened slightly (0.288->0.345 / 0.320->0.436) even though tip
itself barely moved — continued unpriced training leaks yaw
elsewhere while the clamped tip cell stays frozen. Video (cont1
det_1/det_4 topples): genuine clean six-leg cycling 5-10s then a
real end-frame tilt flip, not a statue — matches r2's own read, no
new pathology. Evidence: `logs/ckpt_eval/
cw_amp_m4_turnpushfault1_style05_cont1_gate/`, `logs/
probe_walk_income/turnpushfault1_cont1_income_probe.json`,
`/tmp/cont1_eval_yaw.json` (+ a same-session r2 reproduction at
`/tmp/r2_eval_yaw_verify.json` confirming the eval_yaw cfg-set match
is apples-to-apples). Ledger verdict on `cw-amp-m4-
turnpushfault1-style05-cont1`. Do not re-run this control; the open
work is the dig-in on ypfix1-r3 (owned by the cycle that flagged it)
now sharpened to "why do the overshoot keys specifically cost 2
sacrificed legs on a clamp-pinned substrate." Prior banner below.)

Previous entry (2026-08-23 ~03:4x (**ypfix1-r3 FINISHED — pre-registered
FAIL branch conditions met, verdict DEFERRED to dig-in
(model-tiering).** The banked overshoot keys (6M continuation on the
only all-4-M5-axes checkpoint, 8M total, reward rose 99->194) did NOT
move tip behavior AT ALL: signed income probe
(`logs/probe_walk_income/ypfix1_r3_income_probe.json`) reads yaw_ratio
1.649/1.786 — the deterministic tip rollouts are **BIT-IDENTICAL to
the parent r2 probe** (wz_mean/rmse/slip exact float match) even
though the weights genuinely moved (max|dW|=0.87, tensor-by-tensor
check). The keys DID reprice the farm (yaw_prog on the same
trajectory 291->131 / ~294->100) — income moved, behavior frozen.
Corrected-bus eval_yaw tips 0.3909/0.4742 (bar 0.20; parent
0.4248/0.4932). Safety bar eroded to the edge: gait_valid 10/12 with
TWO new sacrificed legs (sto sac[5], sac[0], upright limps on video;
lineage baseline zero). MECHANISM LEAD: probe feas shows requested
joint speeds pegged EXACTLY at the 5.0 deg/tick safety clamp
(125 deg/s, p95=max) in tip scenarios — tip behavior looks pinned at
a clamp/quantization attractor that reward pricing cannot sculpt
(sub-count action drift leaves executed servo counts unchanged). Per
the run's own FAIL branch: NO further turn+X budget until the
reward-structure/actuation-regime dig-in lands. Matched no-keys
control `cont1` still training (keys-vs-budget attribution). Prior
banner below.)

Previous entry (2026-08-23 ~03:1x (**TWO READOUTS: (1) M4 push+fault
STYLE-VS-CONTROL COMPLETED — style LOSES at acquisition scale,
reversing its mild discovery-time edge; noamp is the M5 push+fault
substrate. (2) `turnpushfault1-style05-ypfix1-r2` LAUNCH_CRASHED
(0 steps, same obs-pad-transplant respec-hygiene bug already seen
twice this cluster), fixed+relaunched as `-r3`.** (1)
`cw-amp-m4-pushfault1-style05-acq1` (6M continuation, 8M total)
VERDICTED INFORMATIVE-ceiling: own-cfg DR-0 gate (fault+push both on)
lands 3/12 topples (2 det tilt_roll/tilt_pitch + 1 sto tilt_roll) —
the SAME total count as its 2M discovery (1 det + 2 sto, just
reshuffled), gait_valid held 12/12, zero sacrificed legs,
style_reward_mean held 0.117 (>0.1, not collapsed) — but det prog med
fell 1.21->0.65 and slip rose 2.77->4.84 (sto 0.87->0.71 / 3.59->4.84)
vs its own discovery numbers, with reward flat after Q1 (111->219->
213->212). Videos (det_0/1/2, sto_1/5) confirm the topples are
genuine end-frame flips after 6+ clean strides, not statues or a new
pathology — matches the gate's own pre-registered INFORMATIVE-ceiling
branch exactly (topples stay ~2-3/12, reward flattening), not PASS
(needs <=1/12, det prog>=0.9) or FAIL (needs topples increasing or
gait_valid<9/12 with reward still rising). Completing the named
style-vs-control comparison this run existed to answer: the noamp
twin's own acq1 (matched 8M budget, same dose, already PASSED)
lands 0/12 topples, det prog med 1.03, sto 0.81 — clean and BETTER
on every axis. Style05 REVERSES its mild discovery-time edge (3/12
vs noamp's 2/12 at 2M, zero-sacrifice) into a clear loss at
acquisition scale. Style is DEPRIORITIZED as the M4 push+fault
carrier; `pushfault1-noamp-acq1` remains the substrate for any M5
push+fault composition. (2) `cw-amp-m4-turnpushfault1-style05-
ypfix1-r2` never trained: pod log shows `--obs-pad-transplant 18 but
obs widened by 0 (93 -> 93); check cfg-sets` immediately after
warm-start then exit, zero PPO iterations, wandb state=finished with
steps=None — the respec inherited the stale transplant flag from its
parent's own launch template even though its `--init-from` checkpoint
is already 93-dim (fail-closed, not silent corruption). Same bug
class as the pushfault1-noamp/turnpushfault1-style05 crashes already
logged this cluster (strip transplant flags when respeccing from a
run that already consumed them). VERDICTED LAUNCH_CRASH; fixed and
relaunched same-cycle as `cw-amp-m4-turnpushfault1-style05-ypfix1-r3`
with an explicit trailing `--obs-pad-transplant=0` override
(argparse last-flag-wins, verified) — VERIFIED RUNNING on
hexapod-mjx-train-1. This is the live repricing-keys test the
concurrent cycle's entry below names (yaw_prog_overshoot_decay +
yaw_prog_avg_s ON, matched no-keys control `cont1` running in
parallel); read both together when they land. Previous entry below.)

Previous entry (2026-08-23 ~03:0x (**DIG-IN RESOLVED SAME CYCLE, run
VERDICTED PASS-partial (ledger 02:53), WRONG-SIGN HYPOTHESIS REFUTED
BY SIGNED PROBE**: the entry below inferred from eval_yaw |err| medians
(0.42/0.49 > park's 0.30) that `turnpushfault1-style05-r2` might spin
the WRONG WAY on tip commands. The signed income probe on the exact
checkpoint (`probe_walk_income --stack yawcmd0`, dirs tip_left/right,
`logs/probe_walk_income/turnpushfault1_r2_income_probe.json`) reads
**wz_mean +0.478 on tip_left (cmd +0.3) and −0.518 on tip_right (cmd
−0.3)** — CORRECT sign both directions, yaw_ratio **1.65/1.79**,
reward_yaw_prog 291/294 collected through the legacy 1.25x overshoot
clip. This is the SAME audited overshoot farm as the substrate's own
1.78 ratio (08-23 yawcmd0 income audit), continuous through the fault
graft, NOT a new wrong-sign defect: the eval_yaw |err| median conflates
~1.7x over-rotation with stride oscillation (wz_rmse 0.47-0.50), which
is why it exceeds the pure-park fingerprint. Root-cause chain closed:
behavior (1.7x over-spin) <- incentive (yaw_prog pays overshoot) <-
pricing (legacy clip; fix keys built+banked 08-23). Consequence: the
built repricing keys are exactly aimed; per the 08-21 ruling this is
misalignment-to-repair, so a keys-ON 6M continuation
(`turnpushfault1-style05-ypfix1-r2`, overshoot_decay+avg_s ON) plus a
matched no-keys 6M control (`turnpushfault1-style05-cont1`) were
launched to attribute any turn repair to pricing vs budget. If keys-ON
lands ratio~1.0 but tips stay >0.20, the residual is yppeak's
hold/forward income dominance — THAT is the next code+bank lever.
Superseded analysis kept below for audit.)

Previous entry (02:5x, wrong-sign hypothesis now refuted, safety-bar
numbers still correct): Own-cfg DR-0 gate (fault+push+yaw all active, corrected fast
servo profile throughout — this run's numbers are NOT subject to the
bus-cfg bug flagged in the correction below): gait_valid 12/12
det+sto (perfect, beats the fresh-stack turnfault1-style05's 9/12
AND matches/exceeds every prior M4 arm), zero sacrificed legs,
topples 2/6 det (both genuine tilt_roll knockdowns after 5+ clean
walking frames on video, not statues) + 0/6 sto = 2/12 total, det
prog med 0.77/slip 4.57, sto prog med 0.70/slip 5.10 (one sto
episode, ep3, reads prog 0.08/slip 36.1 with gait_valid still True —
video shows continuous six-leg movement, likely a turn-dominated
command segment with near-zero net translation, not a pathology).
Mechanism-safety verdict alone: clear PASS, best gait_valid of any
3-way M4 arm to date. BUT hand-run `eval_yaw` (matched 93-dim cfg,
CORRECT fast servo profile per the correction below — `bus.
write_speed=1500`/`write_acc=80`/`servo_vel_max_counts_s=write_speed`/
`safety.max_delta_q_deg=5.0` all set) reads tip-left/right err
**0.4248/0.4932** — WORSE than both named comparison points: worse
than its own immediate parent `turnpush1-style05-acq1-r2`'s corrected
0.3838/0.4302, and worse than the fresh-stack `turnfault1-style05-
acq1-r5`'s 0.2985/0.3002 "total park" reading. Since a pure-park
policy (wz≈0) would read err≈wz_ref=0.3, an err of 0.42-0.49 implies
the realized wz is NEGATIVE on a positive-commanded tip-turn — i.e.
this checkpoint spins the WRONG WAY under a turn-in-place command,
worse than not turning at all. arc-left/right also elevated (0.29,
0.36, 0.36, 0.26 vs turnclone's clean 0.13-0.17 band) and even
fwd-hold shows 0.157 rad/s of spurious yaw at wz_ref=0 (stop-hold is
clean, 0.0). This DIRECTLY REFUTES the hypothesis this run was
launched to test ("inheriting real tip tracking beats a fresh
stack's none-at-all") — sequential composition did not preserve the
(already-eroded) inherited turn skill, it degraded it FURTHER, to a
point worse than the fresh stack's total-park failure mode. Reads as
the M2 yaw-income-audit mechanism (hold/forward income dominates the
tip cell, `k_yaw_prog` pays overshoot not accuracy) COMPOUNDING with
every additional stacked pressure (push, now fault): each new axis
gives the optimizer more reason to abandon the tip cell for richer
hold/forward/limp-survival income, and here it went further and
started actively profiting from wrong-direction rotation on the
tip commands specifically. Per the cycle-tiering rule this is left
UNVERDICTED (no PASS/FAIL written to the ledger) with a DIG-IN flag:
the safety-bar half is unambiguous (PASS) but the turn-tracking half
is a genuine anomaly-vs-both-parents that decides whether "sequential
composition" is a real M4 route at all, or whether NEITHER order
(fresh-stack or sequential) can carry turn accuracy through
additional axis-stacking without a pricing fix — root-cause needed
(is this the SAME overshoot-farming mechanism as yppeak's yaw audit,
now expressed as wrong-sign rotation under compounded pressure, or a
new defect specific to 3-axis stacking?) before spending any more
acquisition budget on ANY 3-way turn+X+Y checkpoint. Do NOT acquisition-
continue this checkpoint blind — more steps under the same pricing
would likely just deepen whichever basin (overshoot or wrong-sign)
the optimizer already picked. Artifacts:
`logs/ckpt_eval/cw_amp_m4_turnpushfault1_style05_r2_{gate,yaw.json}`.
Previous entry (the correction) below.)

Previous entry (2026-08-23 ~02:46x (**CORRECTION (urgent, read before
trusting any "0.14/0.12" or "turn+push substrate is SOLID" claim
below): `cw-amp-m3-turnpush1-style05-acq1-r2`'s eval_yaw tip-err
reading of 0.1431/0.1152 (this cycle's own earlier PASS verdict,
cited in the entry immediately below AND in the M4-turnfault-acq1-r5
entry below that) was measured WITHOUT the training servo profile —
`bus.write_speed`/`write_acc`/`servo_vel_max_counts_s`/
`safety.max_delta_q_deg` were left at their gentle DEFAULTS
(400/20/-/none) instead of the trained fast profile (1500/80/
write_speed/5.0), the same self-inflicted eval-cfg bug already
flagged once on the joystick track (stotight45 second-seed re-eval).
Re-run through `eval_amp_m5` (which always sets the correct bus cfg)
AND by hand with the correct cfg, TWICE reproduced: tip-left/right
err = **0.3838/0.4302** — FAR over the 0.21 bar and WORSE than the
park fingerprint (0.28-0.33), not a near-substrate pass. The
yawcmd0-r2 substrate baseline (0.1525/0.1614) reproduces exactly
through the same M5 harness with the correct cfg, so the baseline is
fine — only the turnpush checkpoint's own reading was wrong. TRUE
STATE: turn+push is turn-eroded, not solid (ledger corrected to
INFORMATIVE-turn-eroded, SKILLS.md row removed). PRACTICAL EFFECT:
`cw-amp-m4-turnpushfault1-style05-r2` (currently training, launched
on the false "solid substrate" premise) is NOT a clean sequential-
composition test — read its result knowing turn was already broken
going in, and re-check ITS eval_yaw with the correct bus cfg before
citing any tip-err number from it. The `turnfault1-style05-acq1-r5`
entry below independently hand-measured 0.2985/0.3002 on the
fresh-3-way-stack fault checkpoint and used the (wrong) 0.14/0.12 as
its point of comparison — that comparison's DIRECTION may still hold
(fresh-stack tip response reads as near-total park, arc tracking a
hair better than turnpush's) but re-verify with the corrected
0.38/0.43 number before drawing conclusions from the gap size.
Previous entries below, uncorrected — treat any inherited "0.14/0.12"
citation in them as wrong.)

Previous entry (2026-08-23 ~02:4x (**M4 TURNFAULT ACQUISITION: SAFETY
BAR FIXED, TURN-IN-PLACE STAYS BROKEN (pre-existing, not caused by
this run) — sequential composition route is now the live test.**
`cw-amp-m4-turnfault1-style05-acq1-r5` (6M continuation of the fresh
3-way turn+heading+fault stack's 2M discovery) PASSES its own named
mechanism-safety gate outright: gait_valid 12/12 det+sto (was 9/12,
3 sto statues), zero terminations/sacrificed legs, prog/slip in the
same ballpark as the solo-axis fault comparison (faultobs2-
headingsfull-style05). BUT hand-run `eval_yaw` (matched 93-dim cfg)
shows tip-left/right err 0.2985/0.3002 rad/s — essentially total
park, vs turnpush1-style05-acq1-r2's 0.14/0.12. Root-caused by running
the IDENTICAL panel on the 2M discovery checkpoint: it ALSO reads
0.30/0.30 (arc tracking a hair better at 2M, 0.13-0.17 vs 0.16-0.23
here) — the tip-park defect predates this run entirely; 6M more
steps fixed gait safety but never touched turn-in-place. Same income-
farming class as the M2 yaw audit (hold/forward income dominates the
tip cell) — a FRESH 3-way stack's optimizer never bothers to learn
tip response at all, unlike the turnclone-BC-taught pairwise turn+push
substrate which had partially learned it before fault was grafted on.
Video clean (six-leg cycling throughout, no flag-leg/drag/statue).
SKILLS.md row added. Names the live fork this cycle already acted on:
`cw-amp-m4-turnpushfault1-style05-r2` (fault grafted sequentially onto
the turn+push-SOLID checkpoint instead of a fresh 3-way stack, VERIFIED
RUNNING train-1) tests whether inheriting real tip tracking beats a
fresh stack's none-at-all — the previous same-named launch attempt
LAUNCH_CRASHED on a self-inflicted `--init-from-source`/checkpoint
mixup (0 steps trained, verdicted, fixed same cycle). Previous entry
below.)

Previous entry (2026-08-23 ~02:3x (**THREE READOUTS: (1) PUSH+FAULT
COMPOSES FOR FREE — `cw-amp-m4-pushfault1-noamp-r2` PASSES the
discovery safety bar at 2M with BOTH hazards every episode (gait
11/12, topples 2/12, BETTER than solo-push's matched-2M 4/12; the one
gv=False episode walks on five carrying its disabled leg; knockdowns
are genuine end-frame flips) — reward still steep at cutoff, 6M
acquisition continuation `pushfault1-noamp-acq1` RUNNING toward
solo-axis bars; SKILLS row added. Read jointly with turnpush-acq1-r2's
same-cycle PASS: NO axis pair is a broken combination. NOTE: the
original `pushfault1-noamp` launch died fail-closed on an inherited
one-shot `--obs-pad-transplant 18` (parent ckpt already 92-dim) —
strip transplant flags when respeccing FROM the run that consumed
them. (2) M3 FORCE AXIS CEILINGS BELOW 40N:
`cw-amp-m3-pushcur2-noamp-n2040-c2r2` INFORMATIVE-plateau — 18M total
lands 3/12 topples vs 12M's 4/12 (inside n=6 noise), tilt terms flat
~102/window over the final 2M (never re-reached the parent's 96 exit)
while reward rose (warm-start recovery shape); joint with
pushhard1-noamp-n2040-c1r1's raw-budget plateau, BOTH curriculum and
budget are refuted at 20-40N. Named lever stands (get-up/recovery
mechanism: tilt-term relaxation + get-up reward + longer episodes —
semantics-bank work first) but is DEPRIORITIZED: M3's brief bar
(repeated pushes, recovery-without-reset, no crouch) is already met
at 10-25N x3 (repeat3); 20-40N is beyond-brief hardening
(q_20260823T0240Z). No more same-recipe continuations at this dose.
(3) M2-YAW OVERSHOOT REPRICING READS OUT FAIL-branch-but-informative:
`cw-amp-m2-yawcmd0-acq2-yppeak` tips 0.272/0.238 (>0.20 bar) BUT
halves the erosion vs matched-budget no-fix acq1-r2 (0.399/0.347);
with the farm closed, yaw income + walk_yaw_err went FLAT all 6M
while total reward rose 152->363 — 100% of optimization pressure is
hold/forward income, confirming audit branch (c) as the residual
driver. yawcmd0-r2 REMAINS champion (its 0.153/0.161 already meets
the <=0.20 M2 bar); hold/forward repricing logged as
named-but-deprioritized vs M5 composition (q_20260823T0240Z). Keep
RL doses short on the yawcmd0 substrate. Previous entry below.)

Previous entry (~02:2x (**M2+M3 TURN+PUSH COMPOSITION CLOSES
CLEAN AT ACQUISITION BUDGET — the 2M "turning makes push harder" read
was undertraining, not a broken combination; fault graft onto this
checkpoint launched (sequential M2->M3->M4 route).**
`cw-amp-m3-turnpush1-style05-acq1-r2` (6M continuation of
turnpush1-style05-r2, 8M total, same 10-25N single-shove dose) PASSES
every pre-registered branch: DR-0 own-cfg gate (push+yaw both on) det
prog med 1.113 (bar >=0.9, was 0.37 at 2M), topples 0/6 det + 2/6 sto
(bar <=1/<=2, both sto losses genuine tilt_pitch/tilt_roll falls on
video's LAST frame after 6-9 clean strides — not statues), gait_valid
6/6 det+sto, zero sacrificed legs; eval_yaw (run manually, matched
phase-obs cfg — not part of the standard prestage) tip-left/right err
0.1431/0.1152, both <=0.21 and within 0.05 of the pre-push turn
substrate's own 0.1525/0.1614 (tip-right actually IMPROVED under
push training — turn was not sacrificed for push survival). Strips
watched (det_0, sto_0/2/4): clean six-leg gait with visible heading
rotation continuing under shove; the two sto topples are genuine
late-episode falls, not paddle-creep/statue. SKILLS.md row added.
Pattern now measured 3/3: every M3/M4 axis that read
INFORMATIVE-undertrained at 2M discovery (push-alone, fault-alone,
turn+push) closed clean at ~3x acquisition budget — no axis has yet
been a genuinely broken combination, only an underbudgeted one.
LAUNCHED this cycle (as `-r2`, after a self-inflicted launch-config
crash on the first try — `respec --init-from-source` warm-starts from
the SOURCE run's own checkpoint, not `--parent`, so the first attempt
accidentally re-initialized from turnfault1-style05's own 93-dim
checkpoint and padded it AGAIN, `--obs-pad-transplant 18` -> "widened
by 0", zero steps, LAUNCH_CRASH recorded honestly; corrected `-r2`
with an explicit `--init-from` override is RUNNING on train-1):
`cw-amp-m4-turnpushfault1-style05-r2` — fault grafted onto THIS
now-solid turn+push checkpoint (75-dim, correct init), same
dr.fault_prob=1.0/obs.fault_health=1/--obs-pad-transplant 18 wiring
as turnfault1-style05, 2M discovery — the sequential composition
route the 08-23 ~00:xx entry named as the alternative to a fresh
3-way stack, now that turn+push no longer needs solidifying.
Previous entry below.)

Previous entry (~02:0x (**M3 PUSH CURRICULUM STAGE 2: COUNT
AXIS CLOSED at 3 shoves (PASS, staged beats cold jump 1/12 vs 3/12
topples); FORCE axis at 20-40N misses its bar by ONE sto episode with
terms still falling — ceiling-continue launched.** (1)
`cw-amp-m3-pushcur2-noamp-repeat3` PASS: repeat_max=3 warm from rung 2
lands 0/6 det + 1/6 sto topples (bar <=1/<=2), gait 12/12, det prog
med 1.17, zero sacrificed — count axis closes at 3, staged-count is
the recipe (SKILLS row added). The pre-named free-rung alternative
fired again (tilt terms low-flat ~19-21/window all 6M, reward flat
after Q1), so the eval-side DENSITY probe was run same-cycle
(repeat_max=6 at 15s = DOUBLE the trained count, same ckpt): 1/6 det
+ 0/6 sto topples, gait 12/12, prog 1.10/1.06 — count/density at
10-25N is CLOSED OUTRIGHT; force is M3's only open axis
(`logs/ckpt_eval/cw_amp_m3_pushcur2_noamp_repeat3_dense6`). (2)
`cw-amp-m3-pushcur2-noamp-n2040` INFORMATIVE-ceiling: staged 20-40N =
4/12 topples (det 1/6 PASSES its half, prog med 1.13; sto 3/6 misses
<=2/6 by one) with tilt terms falling MONOTONICALLY 136->96/window
and reward rising every quarter (-47/52/72/83) — NOT the flat-jump
plateau fingerprint; per its own branch + 08-21 ruling, 6M
continuation RUNNING as `pushcur2-noamp-n2040-c2r2` (18M total; the
-c2/-c2r1 stubs are SUPERSEDED launch-race casualties, never
trained). Caveat:
decline is slow (~4-5/window per 1M); if c2 flattens in the 90s band,
staging saturates below 40N and the recovery-specific mechanism
(get-up reward / longer episodes) becomes the named M3 lever. The
curriculum-vs-budget fork is now ADJUDICATED (02:1x):
`pushhard1-noamp-n2040-c1r1` (raw +6M at the flat 20-40N dose, 12M
total) read out INFORMATIVE-plateau — 3/12 topples (det 3/6, all
genuine end-frame knockdowns incl. full flips; sto 0/6), tilt terms
noisy-flat ~35-45/window over the last 2M, reward flat-to-down after
Q2 (131.8/131.2/121.1). Same topple count as the curriculum today
(3/12 vs 4/12, within noise) but ONLY the curriculum's terms are
still falling — raw budget is refuted as the 20-40N lever, staging
is the live mechanism; c2 decides whether it converges or saturates
into the recovery-mechanism branch. Note: c1r1's knockdowns are FULL
FLIPS — if c2 flattens, that is direct evidence the missing skill is
righting/get-up, which a 15s tilt-terminated episode can never learn
(no gradient after knockdown). (3) Also spent the STATUS-named turnpush lever:
RUNNING as `cw-amp-m3-turnpush1-style05-acq1-r2` (the unsuffixed
stub is a SUPERSEDED launch-race casualty) — 6M acquisition
continuation of turnpush1-style05-r2 (reward 16->197 still climbing
at its 2M cutoff = undertrained read; gate separates budget-refuted
vs turn-eroded vs turn-vs-push-interference branches). Previous
entry below.)

Previous entry (~01:5x): (**M4 FAULT-SIGHT SIGHTED-VS-BLIND PAIRING CLOSED:
fault_health obs-sight more than triples the safe-limp rate vs blind on
the full-heading substrate, generalizing faultobs1's forward-only
finding.** `cw-amp-m4-faultobs2-headingsfull-blind` (identical to the
already-PASSed `..._noamp`/`..._style05` pair except `obs.fault_health`
1->0) clears its own mechanism-safety floor (gait_valid 9/12, exactly
the >=9/12 bar) and reads decisively on the paired comparison it was
built for: same seed=0/same fault+command draws as the sighted
siblings, sto gait_valid blind 3/6 (sacrificed legs ep0/1/3: legs
0,2,5) vs noamp 5/6 (1 sacrifice) vs style05 6/6 (zero sacrifices);
det stays 6/6 for all three but blind's det slip is also worse (med
3.58 vs 3.08 both sighted). Video (walk_sto_0/_3 strips) confirms
blind's failures are genuine LIMPS (one dangling/static leg, other
five still cycling), not statues -- same fingerprint as the sighted
arms elsewhere, just 3x more frequent; progress_ratio is NOT the
discriminating metric (blind's sto prog median is actually slightly
higher, inflated by the other 5 legs covering ground). Closes the
faultobs2-headingsfull sighted/style/blind trio. SKILLS.md row added.
M4's open frontier is now graft-onto-turn composition (turnfault1,
already INFORMATIVE/undertrained at 2M, verdicted by a concurrent
cycle), not sighted-vs-blind. Previous banner below.)

Previous entry (2026-08-23 ~01:4x (**M3 PUSH CURRICULUM STAGE 1: BOTH
noamp rungs PASS — force bridges (15-30N clean where the flat 20-40N
jump plateaued) and count rung 2 is nearly FREE; but the count x
force COMBINATION plateaus at the same 4/12 floor — stage-2 batch
launched.** (1) `cw-amp-m3-pushcur1-noamp-b1530` PASS: 15-30N
single-shove bridge from pushacq1-noamp lands 1/6 det + 1/6 sto
topples, gait 12/12, det prog med 1.20, tilt terms 26→14/window
(prediction <25; n2040's flat jump sat ~53) — bridged dose keeps the
recovery gradient alive; genuine 22° recoveries on strips.
(2) `cw-amp-m3-pushcur1-noamp-repeat2` PASS-but-free: repeat_max=2
lands 0/6+2/6, but terms started AND stayed low (13→8.5/7→5) — the
pre-named "rung 2 inherited from single-shove prior" alternative
fired; rung 3 is the real staging test. (3)
`cw-amp-m3-pushcur1-style05-r3b1530-r1` INFORMATIVE-plateau: count
HELD (repeat3) + force bridged 15-30N over-topples its laxer bar
(3/6 det all tilt_pitch genuine knockdowns + 1/6 sto = 4/12, the
n2040 flat-jump floor) with the decisive pitch term FLAT all run
(30→27.5) and reward flat after Q2 — force-under-count does NOT
compose at 6M; envelope stays ~25N under count. Joint read: force
alone bridges, count alone is cheap, the combination saturates —
if the stage-2 batch also plateaus, a RECOVERY-SPECIFIC mechanism
(get-up reward / longer episodes) is the named M3 lever past ~25N.
style_reward_mean thinned to 0.103 (disc not vetoing; style still
not load-bearing). LAUNCHED (3, all verified RUNNING):
`cw-amp-m3-pushcur2-noamp-n2040` (stage 2: 20-40N from b1530 ckpt),
`cw-amp-m3-pushcur2-noamp-repeat3` (rung 3 from repeat2 ckpt), and
`cw-amp-m3-pushhard1-noamp-n2040-c1r1` (the matched budget control
that NEVER TRAINED — c1 was REFUSED 3x on a stale pod code marker
and fell into backlog_failed silently; relaunched via respec --now
sync path; c1 stub marked SUPERSEDED). Stage-2 fork at 12M total:
curriculum vs raw budget vs both-plateau→recovery-mechanism.
Previous banner below.)

Parallel entry (~01:5x, checkup-cycle spend of the M2-yaw named
lever: **INCOME AUDIT EXECUTED — THE TURN-EROSION MECHANISM IS
MEASURED AND REPRICED; fix arm launched.** Plain English: we now know
why more training made turning worse — the reward pays a robot that
spins PAST the commanded rate more than one that tracks it. Evidence
(probe_walk_income extended with a `yawcmd0` stack +
tip_left/tip_right/hold dirs + yaw-tracking metrics;
`logs/probe_walk_income/yawcmd0_turn_income_audit.json`, 3 seeds,
turner=yawcmd0-r2 vs parker=acq1-r2 vs scripted refs): (a) the eroded
policy does NOT park — it OVER-ROTATES (yaw ratio 1.78 vs champion
1.09, wz_rmse 0.40 vs 0.25) yet collects MORE reward_yaw_prog (169 vs
148): the legacy `k_yaw_prog` clip pays up to 1.25x for overshoot, so
the income gradient points PAST the command — this also explains
yawprice3x (3x income amplified the farm, not accuracy); (b) deeper,
yaw terms price INSTANTANEOUS wz: a 0.95-ratio scripted tracker earns
NEGATIVE yaw_prog (stride-oscillation sign flips hit the -1.5 clip)
while a smooth 2.0-ratio spinner earns positive — the exact DC-vs-AC
defect class fixed for k_yaw_still on 08-11; (c) the parker's
+28.6/ep total advantage sits in hold (+160) and forward (+118)
segments — hold-freeze (1473/ep) is the richest cell in the table,
above the honest tip ceiling (1209). FIX LANDED (walk_task.py, both
keys NEW + default-off + bit-exact; TURN bank extended to 8/8 green
incl. legacy defect-proof, fix, and bit-exact cases):
`reward.yaw_prog_overshoot_decay` (income peaks at ratio 1.0, decays
past it, never negative on overshoot) + `reward.yaw_prog_avg_s`
(ratio priced on the wz EMA like yaw_still_avg_s). ARM LAUNCHED:
`cw-amp-m2-yawcmd0-acq2-yppeak` = exact acq1-r2 respec (6M
continuation off yawcmd0-r2) + the two keys ON — if pricing was the
erosion driver, tip errs hold <=0.16 at 6M instead of decaying to
0.35-0.40; if erosion recurs at matched budget, the residual lever is
the hold/forward income dominance in (c). yawcmd0-r2 REMAINS champion
until the arm reads out.)

Previous entry (~01:3x (**AXIS COMPOSITION DOES NOT COME FOR
FREE: grafting M3 push and M4 fault onto the new turn-capable
substrate (`cw-amp-m2-turnclone-yawcmd0-r2`) each land INFORMATIVE,
well below the same mechanisms' solo-axis numbers at matched/lighter
budget.** `cw-amp-m3-turnpush1-style05-r2` (push graft, 2M discovery,
same dose pushsmoke1-style05 used) lands det prog med 0.37/slip 3.24,
sto 0.21/5.33 (one genuine fall, sto/0 TERM tilt_pitch), dir_err
48/63deg — vs pushsmoke1-style05's OWN numbers at the identical 2M
budget on the non-turn substrate (det 1.20/3.27, sto 0.94/3.59):
~3x worse progress for the same push dose, purely from the turn
channel being present. `cw-amp-m4-turnfault1-style05` (fault graft,
2M discovery) misses its own pre-registered mechanism-safety bar
(gait_valid 9/12 vs >=10/12: 6/6 det but only 3/6 sto, 3 sacrificed
legs — video walk_sto_2 shows a near-stationary shuffle, not a
limp), prog collapses (det 0.29/sto 0.14) well below
faultobs2-headingsfull-style05's 1.09/0.57 (that ran at 4M, not 2M,
so not a clean control, but the gap is stark). BOTH runs: training
reward still climbing at budget end (turnpush 16→197, turnfault
46→203, neither flat) — per the 08-21 ruling this reads UNDERTRAINED,
not misaligned: turn+push and turn+heading+fault are genuinely
harder joint skills that do not transplant for free from each axis
solved separately, unlike push-onto-plain-heading (worked immediately
at 2M) or fault-onto-plain-heading (PASSED at matched-family budget).
Zero crashes, gait never fully breaks (no NaN, det gait_valid stayed
6/6 both runs). NEXT (named, not spent): acquisition-budget
continuation (6M, matching pushacq1's dose) is the direct test of
"just needed more steps"; the cleaner alternative is SEQUENTIAL
composition (solidify turn+push before adding fault, rather than a
fresh 3-way stack at once) — matches the brief's own M2->M3->M4
milestone ordering. Also this cycle: the pre-registered
style-vs-noamp fork on `cw-amp-m4-faultobs2-headingsfull-style05`
(this cycle's assigned run) was independently triaged and PASS
(neutral)-verdicted by a concurrent cycle before this cycle's own
per-episode analysis finished; that analysis (paired same-seed
episodes, 8-9/12 same-direction deltas but driven mostly by one
outlier episode, combined medians actually favor style05 on slip)
is CONSISTENT with the neutral verdict once the outlier is accounted
for — no dig-in filed, no re-verdict needed. Previous banner below.)

Previous entry (2026-08-23 ~01:2x (**M2-YAW STYLE CONTROL: PASS-neutral
— the task-only twin of the M2-yaw champion matches it on every
axis; style is still not load-bearing anywhere (7th axis).**
`cw-amp-m2-turnclone-yawcmd0-r2-noamp` (single lever: style 0.5→0.0)
VERDICTED PASS on the pre-registered within-noise branch: eval_yaw
turn err med 0.1557 vs sibling 0.1548, tips 0.1806/0.1870 (both
≤0.20 bar, nominal +0.03 vs 0.1525/0.1614 but inside per-scenario
scatter — noamp 0.06 better on arc-right-max), hold 0.0038, 0 falls;
DR-0 walk gate 6/6 det+sto (det prog 0.99/slip 2.31; sto 0.75/4.38,
slightly better than sibling's 0.65/5.00), zero terms, det strips
clean. yawcmd0-r2 (style05) STAYS champion. Also verdicted:
`cw-amp-m3-turnpush1-style05` r1 FAILED launch-config (obs 75!=74 —
respec from pushsmoke1-style05 lacked the yaw cfg block; zero steps);
the fixed `-r2` is training on train-2, turn+push composition read
pends it. M2-yaw open lever unchanged: income audit + bank case (or
accept yawcmd0-r2 as champion + short doses). Previous banner below.)

Previous entry (~01:0x (**M2-YAW BUDGET LEVER REFUTED —
MORE TRAINING ERODES TURNING; M4 FAULT-SIGHT PASSES ON THE
FULL-HEADING SUBSTRATE (style neutral again), blind control
launched.** (1) `cw-amp-m2-turnclone-yawcmd0-acq1-r2` VERDICTED FAIL
on its own pre-registered branch: the 6M acquisition continuation off
yawcmd0-r2 landed eval_yaw turn err med 0.264 vs parent 0.155 vs raw
turn-clone 0.104; tip-left/right err 0.399/0.347 vs parent
0.153/0.161 — WORSE than parking (0.30), i.e. RL actively unlearned
the clone's turn pattern while reward rose the whole run (146→350)
and translation held (gate 6/6 det+sto, det prog med 1.10; only det
slip eroded 2.24→2.99). Joint story with yawprice3x (3x income → err
0.208): turn-skill erosion is MONOTONE in training steps AND income
multiplier — pricing, exposure, and budget are now ALL refuted on
this substrate; per the 08-21 ruling this is misalignment, not
undertraining. The yaw stack's optimum is provably not accurate wz
tracking (hold segments stay perfect 0.001-0.007 in every arm while
turn segments decay — the stack pays holds reliably; turn income is
outbid or farmable). NEXT LEVER (named, not spent this cycle): an
INCOME AUDIT + semantics-bank case — extend
probe_walk_income/test_task_semantics to prove an accurate-turner
out-earns the erode-to-park policy under the yawcmd stack, then
reprice until it does (bank-first per the 08-21 ruling); the
legitimate fallback is accepting `yawcmd0-r2` (2M, tips 0.15/0.16,
<=0.20 milestone bar, clean full-heading translation) as the M2-yaw
champion and keeping RL doses SHORT (<=2M) on turn-clone substrates.
yawcmd0-r2 REMAINS champion; artifact
`logs/ckpt_eval/cw_amp_m2_turnclone_yawcmd0_acq1_r2_yawgate.json`.
(2) M4 fault line: `cw-amp-m4-faultobs2-headingsfull-noamp` PASS —
fault-sighted walker survives the jump from forward-only to the
full-heading substrate at 4M under guaranteed faults (DR-0 own-cfg:
gait_valid 11/12, det prog med 1.14/slip 3.08; the sac=[0] episode
WALKS 0.47m limping on 5 legs, worst-fault ep advances upright — no
statue, no crouch, heights 0-22mm, 0 terminations).
`-style05` twin PASS-neutral (12/12, paired same-seed episodes within
noise on every axis, hard fault draws degrade BOTH arms identically;
disc unsaturated d_real 0.78/d_fake -0.93, style_reward 0.121) — the
first genuinely off-distribution axis (fault transients absent from
teacher_v2) still finds style functionally neutral: no styleveto, no
help. CAVEAT: the blind-vs-sighted delta (faultobs1's +18% prog/-27%
slip) is still unmeasured on this substrate (faultobs1's blind
control was forward-only) — LAUNCHED
`cw-amp-m4-faultobs2-headingsfull-blind` (obs.fault_health 1→0,
pad-transplant 0, single lever, 4M, train-0); its same-seed paired
read vs the noamp gate report answers whether fault-sight
generalizes to command diversity. Previous banner below.)

Previous entry (~00:5x (**M3 COUNT AXIS CLOSED ON THE
STYLE05 LINE: `cw-amp-m3-pushhard1-style05-repeat3` VERDICTED PASS
on every pre-registered bar** — up to 3 shoves/ep at 10-25N: DR-0
own-cfg topples 1/6 det + 0/6 sto (bar ≤2/≤3), gait_valid 12/12,
zero sacrificed legs, det prog med 1.18 / slip 2.78,
style_reward_mean 0.109 (>0.1, thinning 0.85→0.11);
recovery-without-reset on mechanical record: 10/12 episodes
roll_class=recovered (det/0 rides a 16.9° roll spike back to 1.0°
tail while covering 1.21m), strips watched — upright six-leg
cycling, no crouch/flag-leg. Caveats: training tilt-terms roughly
FLAT across the 6M at ~20 pitch/~15 roll per window (~2.5-3x the
parent's single-push floor — eval-time multi-push survival was
largely inherited from pushacq1, same-dose budget mined out again,
matching the grid's plateau pattern), and pushes cost HEADING (det
dir_err mean 38.7°, fwd scatter 0.09-1.21m — tracking useful, not
tight under fire). JOINT READ vs `noamp-repeat3-r1` (3/12 topples,
flat learning): style05 lands better on the COUNT axis (1/12 vs
3/12 topples, late pitch-term dip 21→8 vs noamp flat) — the first
push-axis read where style05 beats noamp — but n=12 each with
unequal-strength parents (pushacq1-noamp 0/12, -style05 1/12), so
treat as within-noise until a matched retest, do NOT flip the
style-not-load-bearing finding on this alone. LAUNCHED:
`cw-amp-m3-pushcur1-style05-r3b1530` (repeat3's own ckpt,
repeat_max=3 KEPT, per-shove force bridged 15-30N, 6M) — the
style05-line curriculum rung the 00:4x noamp batch
(b1530/n2040-c1/repeat2, all single-shove or count-only) leaves
open: count is solved at base dose on this substrate, so escalate
force UNDER count. Also this cycle: yawcmd0-r2 + tip90 turnclone
reads were verdicted by their owning cycles; this cycle's duplicate
eval_yaw on yawcmd0-r2 independently agreed (tip 0.153/0.161, 0
falls, artifact `cw_amp_m2_turnclone_yawcmd0_r2_yawgate.json`).
Previous banner below.)

Previous entry (~00:4x (**M3 ESCALATION GRID READ: BOTH flat
jumps (force 20-40N, count x3) hit their pre-registered plateau
branches; style-kept 20-40N arm PASSES its own laxer bar but the
joint read says style is STILL not load-bearing; push CURRICULUM
batch launched.** `cw-amp-m3-pushhard1-noamp-n2040` INFORMATIVE-
plateau: 20-40N single shove from the 0/12 pushacq1-noamp champion
costs 4/12 genuine knockdowns (3/6 det + 1/6 sto, full flips on
video, no statues), pitch-terms flat over the last 2M (53.1→53.0)
though roll still creeps (46→43) and reward still rises (84→115) —
flat dose jump exceeds the 6M-recoverable envelope.
`cw-amp-m3-pushhard1-noamp-repeat3-r1` INFORMATIVE-plateau, the
starker one: up-to-3 shoves/ep learns NOTHING in 6M (reward flat
315→322 after Q2, tilt-terms flat pitch ~15-17 / roll ~9-11 the
whole run), 3/12 topples vs parent 0/12; NOT a FAIL — video shows
real multi-shove survival (det_0 absorbs shoves, 1.24m) and terms
run ~2.5x parent at 3x count (sublinear). `cw-amp-m3-pushhard1-
style05-n2040` PASS its pre-registered bar (topples 2/6+2/6 vs
≤2/≤3, gait_valid 12/12, det prog 1.23 / slip 2.98,
style_reward_mean 0.13>0.1 though thinning 0.29→0.13) — but both
n2040 twins land statistically identical (4/12 topples each; the
verdict split is purely the differing pre-registered bars), so the
running style-not-load-bearing finding extends to the hard-shove
axis. LAUNCHED (queued to drain, 3x6M on the noamp line):
`cw-amp-m3-pushcur1-noamp-b1530` (force-curriculum stage 1, 15-30N
bridge from pushacq1-noamp ckpt; stage 2 = 20-40N chain on PASS) vs
`cw-amp-m3-pushhard1-noamp-n2040-c1` (+6M at 20-40N from n2040's own
ckpt, 08-21-ruling continuation — the matched curriculum-vs-budget
control) + `cw-amp-m3-pushcur1-noamp-repeat2` (count-axis rung:
repeat_max=2 from pushacq1-noamp ckpt, tests whether staging unlocks
repeat3-r1's flat gradient). style05-repeat3 joint read pends its
owning concurrent cycle. Previous banner below.)

Previous entry (2026-08-23 ~00:1x (**CADCOUPLE VERDICT: the speed-coupled
phase clock WORKS but trades gait quality for range — CLOCK-ONLY
COUPLING IS INSUFFICIENT, both style05 and nostyle land in the SAME
partial pathology.** `cw-amp-m2-bcinit-sec5-style05-speedrange-
cadcouple-r1` (style 0.5/0.5) and `-cadcouple-nostyle`: DR-0 gate
gait_valid stays 6/6 det+sto both arms, zero terminations/sacrificed
legs (the mechanism doesn't break the gait outright) and realized
speed_mean now spans 0.051-0.114 (r1) / 0.057-0.118 (nostyle) m/s —
clears the pre-registered >=2.0x widening-ratio bar (2.24x/2.07x) vs
the pinned ~1.5x parent band, confirming the fastphase/nostyle root
cause (fixed-rate clock) was correctly diagnosed. BUT per-episode
speed does NOT correlate with COMMANDED speed (r=0.02 r1, r=-0.05
nostyle, computed against each episode's own cmd_dist_m/15s) — the
range widened in absolute spread but not as a function of the command
— and slip roughly doubled (det med 4.40 r1 / 4.20-ish nostyle vs the
parent's ~2.7, worst episodes 6-12/m) with progress collapsing on the
harder-commanded episodes (prog med 0.25-0.32 vs ~0.6-0.7 before).
Video (walk_det_1, walk_sto_5 both arms): on high-commanded-speed
segments the legs cycle visibly FASTER but the body barely advances
across the checkerboard — a high-frequency march/shuffle, not a
longer stride; the pre-registered "widens but paddle-creeps" failure
mode named in the gate text (video overrides scalar). Style added
NOTHING protective or harmful — r1 and nostyle are statistically
indistinguishable on every axis, matching this whole M2 curriculum's
running finding that style is not yet functionally load-bearing.
Root-cause chain: behavior (fast legs, flat body speed, no command
correlation) <- incentive (reward+style blend still nets positive
since quarters rose 48->180, so the busier-but-not-faster gait is not
actively disincentivized enough) <- pricing gap (nothing charges
wasted-motion/slip specifically vs commanded speed) <- mechanism
(ONLY the clock rate was coupled to command; stride length/workspace
amplitude stayed fixed by construction, so faster stepping can only
waste more foot-contact time as slip, not cover more ground). VERDICT:
INFORMATIVE (both arms), CLOCK-ONLY LEVER CLOSED. Per the pre-
registered joint read, "both widen" was the true branch, but the
speed/slip/video evidence overrides the raw widening-ratio pass —
real command-following speed modulation is not yet achieved. NEXT
(not yet launched, real code + bank work, named not spent this
cycle): couple stride length/workspace amplitude to commanded speed
alongside the clock, OR add an explicit slip-vs-commanded-speed
price (semantics-bank test first per the 08-21 ruling) — a plain
"~0.05-0.14 m/s achieved, matches the teacher's own 0.06-0.10 m/s
hardware band" acceptance is also a legitimate fallback if the next
lever also fails, since M2's speed-range item is a beautification
target, not the M3/M5 DONE gates. Previous banner below.)

Previous entry (2026-08-23 ~00:2x (**M2-YAW TIP-PARK WALL BROKEN:
CLOCK FIX WAS NECESSARY-NOT-SUFFICIENT; BC-TURN-CLONE (new tool this
cycle) IS THE FIX. `turnclone-yawcmd-tip50`/`-tip90` PASS the run's
own gate; `tip50/90-clockyaw` VERDICTED FAIL-park.** The clock-fix
joint read landed FAIL on both arms: `tip50-clockyaw` (tip err
0.290/0.326) and `tip90-clockyaw` (0.254/0.332) are statistically
indistinguishable from their pre-fix parents (0.2996/0.3000,
0.2982/0.2999) — `goal.walk_phase_run_on_yaw=1` lets the gait cycle
continue through a turn-in-place segment but cannot invent a turning
motor pattern the substrate was never shown. ROOT CAUSE (found this
cycle): `bc_init_gait.py`'s `collect()` called
`gait.set_velocity(vx=.., vy=..)` and NEVER drove TripodGait's own
native `omega` channel (`_foot_target_in_body`'s standard
`v = v_lin +/- omega x r` turn-in-place kinematics), even when the
env's sampled goal carried a nonzero `wz_ref` — every BC clone in
the whole lineage was demonstrated ONLY straight-line gaits, so every
downstream RL arm had to invent rotation from a zero-omega prior with
no anchor and never did. Probe confirmed the teacher's omega channel
genuinely works in-sim first (fail-fast check before spending
compute): raw TripodGait at omega=+/-0.3 achieves 0.15/-0.19 rad/s
body wz (45-63% realization, matching the pre-existing
`DRIFT_RIDE_WZ` calibration of ~40% while translating). **FIX
LANDED**: `bc_init_gait.py --drive-omega` (default off, requires
`goal.walk_yaw_cmd=1` in the stack, fails closed on
`obs.mode_onehot`+`walk_phase_obs` combos) drives the teacher's omega
from the env's own sampled `wz_ref`, with the tail phase-index
arithmetic fixed to account for the extra `wz_ref` obs column.
Bank `rl_move/tests/test_bc_init_gait_omega.py` 5/5 (bit-exact
default-off regardless of yaw_cmd in the stack; requires-yaw-cmd
fail-closed; mode_onehot fail-closed; action stream actually changes
on turn ticks; phase stays correctly synced with the tail shift);
full suite re-run 163 pass/1 pre-existing-red (`fastprof`, unrelated,
same fail on the pre-change tree)/4 skip/1 xfail. Built a fresh
clone `ppo_goal_cw_bcgait_turnclone_fullprof_phase1.zip` (same
fullprof/phase1 env contract as the lineage's original clone, plus
`goal.walk_yaw_cmd=1`/`walk_yaw_max_rad_s=0.3`/
`walk_turn_in_place_frac=0.3`/`walk_phase_run_on_yaw=1` +
`--drive-omega`; holdout act err 0.0069, obs width 75 = matches the
yawcmd family's own obs natively, no `--obs-pad-transplant` needed).
**The RAW clone alone (zero RL) already scores `eval_yaw` tip err
0.096/0.108** (turn med 0.1035) — dramatically better than any
RL-trained arm in the lineage, before any fine-tuning. Launched
byte-identical respecs of the FAILed `tip50/90-clockyaw` arms with
only `--init-from` swapped to the turn-clone
(`cw-amp-m2-turnclone-yawcmd-tip50`/`-tip90`, 2M, same reward/
exposure stack): **BOTH PASS** the run's own pre-registered
tip-err-<=0.20-both-directions bar — tip50-turnclone 0.160/0.135,
tip90-turnclone 0.133/0.174 (RL mildly eroded the raw clone's own
0.10 floor rather than sharpening it, but stayed far above the
0.25-0.33 park level; dose 0.5 vs 0.9 frac reads as a wash, not a
clear response). Non-tip translation episodes stay clean at both
doses (gait_valid true, slip/m 2.0-2.5, no sacrificed legs) — turning
capability did not cost translation quality. One crash caught+fixed
in-flight: the zero-dose control (`turnclone-yawcmd0`, no dedicated
tip episodes) inherited `--obs-pad-transplant 1` from its parent's
own respec lineage and crashed at step 0 (`obs widened by 0 (75 ->
75)`) since the turn-clone's obs already matches natively; retried as
`turnclone-yawcmd0-r2` with `--obs-pad-transplant 0`, RUNNING.
SKILLS.md row added (AMP section); this closes the "next lever" this
cycle's tip50/90-clockyaw verdicts pre-registered. Next: read
yawcmd0-r2 (does dedicated tip exposure matter at all, or does the
turn-taught init alone already clear the bar on arc-turns-while-
translating?); if the pattern holds, re-run the style05 lineage's
full heading/backward/lateral M2 milestone from this turn-capable
substrate instead of the old yaw-blind one.)

Previous entry (2026-08-23 ~00:0x (**STYLE-KEPT PUSH WALKER PASSES
ACQUISITION; SINGLE-PUSH DOSE MINED OUT; M3 ESCALATION GRID
LAUNCHED: `cw-amp-m3-pushacq1-style05` VERDICTED PASS** — 6M under
a guaranteed 10-25N shove: DR-0 own-cfg topples 1/6 det + 0/6 sto
(smoke was 1+1), gait_valid 12/12, det prog 1.23 / slip 3.19, no
crouch-statue on video, style_reward_mean 0.116 (>0.1 bar but
thinning 0.30→0.12). KEY NEGATIVE: training tilt-terminations went
FLAT over the final ~3M at constant dose (pitch ~6-7, roll ~5-6.5
per window; predicted halving to ≤4 did NOT happen) — same-dose
budget is exhausted. LAUNCHED same cycle from this checkpoint, per
M3's explicit repeated-push requirement:
`cw-amp-m3-pushhard1-style05-repeat3` (dr.ext_push_repeat_max=3,
up to 3 shoves/ep, gaps 1-3s, dose/shove unchanged) +
`cw-amp-m3-pushhard1-style05-n2040` (single shove, dr.ext_push_n
20-40N) — decomposes push COUNT vs push FORCE as the next
frontier. Code this cycle: one-line domain_rand fix + regression
test (cfg-set delivers repeat_max as float 3.0; range() crashed —
caught in launch prep, never trained broken). Style-vs-noamp 6M
joint read pends the concurrent cycle's `cw-amp-m3-pushacq1-noamp`
verdict; if noamp reads clearly better, escalation respecs onto
the noamp line are cheap. Previous banner below.)

Previous entry (~23:4x (**STYLE CHANNEL CLEARED FOR M3
PUSH HARDENING: `cw-amp-m3-pushsmoke1-style05` VERDICTED PASS on the
pre-registered joint read vs noamp r4 — the discriminator does NOT
veto off-distribution shove-recovery transients.** Training tilt
terminations fell at constant dose (pitch 22→8.3, roll 16.3→7.8 per
window) while reward rose 28→246 over 2M; DR-0 own-cfg gate 6/6
gait_valid det+sto, det prog 1.20 / slip 3.27 / fwd 0.72m (r4 band);
topples 1/6 det + 1/6 sto vs r4's 1/6+3/6; disc healthy
(style_reward_mean 0.19). FAIL-styleveto refuted at 2M scale.
LAUNCHED same cycle: `cw-amp-m3-pushacq1-style05` — 6M acquisition
from the smoke's own checkpoint, dose unchanged, the style-kept M3
hardening candidate + matched style-vs-nostyle read beside the
running `cw-amp-m3-pushacq1-noamp`. Also this cycle: checkup rc=1 on
`-tip90-clockyaw` was a benign race — run finished its 2M budget
cleanly, ledger flipped RUNNING→FINISHED; tip-pair joint read owned
by the concurrent cycle. Previous banner below.)

Previous entry (~23:3x (**M3 OPENED FOR REAL:
`cw-amp-m3-pushsmoke1-noamp-r4` VERDICTED PASS on the mechanism-
safety bar — first-ever training use of the mid-episode shove
(`dr.ext_push_*`, 10-25N / 0.15-0.4s / random direction, once per
episode) is safe AND the walker visibly LEARNS to survive it:
training tilt terminations fell ~3x at CONSTANT dose (tilt_pitch
42→15, tilt_roll 27→7-11 per window) while reward rose 2.3→335;
DR-0 own-cfg gate gait_valid 6/6 det+sto, zero sacrificed legs, det
prog med 1.24 / slip 3.05 / fwd 0.70m. Blunt: the shove still wins
1/6 det + 3/6 sto episodes (det_3 strip: clean walking ~13s then
flipped; sto ep4 degraded prog 0.21 / slip 18.3) — this is
survive-most-pushes, NOT recovery-after-knockdown. r1-r3 are
REFUSED launch stubs, nothing trained. FOLLOW-UP BATCH LAUNCHED
(this cycle): `cw-amp-m3-pushsmoke1-style05` (style twin from
style05-headingsfull, speed pinned 0.08 so task cfg == r4's; first
axis where AMP style could actively FIGHT the task — recovery
transients are in no teacher_v2 clip; FAIL-styleveto branch
pre-registered) + `cw-amp-m3-pushacq1-noamp` (6M acquisition from
r4's checkpoint, dose unchanged — terminations still falling and
reward still rising at the 2M cutoff, 08-21 ruling continuation;
plateau branch names the push-magnitude curriculum and the
REPEATED-push mechanism extension as next levers — M3's bar says
repeated pushes and `dr.ext_push_*` currently draws exactly ONE per
episode; that extension is real code work a future cycle owns).
Tip-clock banner below.)

Previous entry (~23:2x (**TIP PARK ROOT CAUSE FOUND: the
gait clock FREEZES during turn-in-place commands — env defect, fixed
this cycle, clock-fix dose pair launched.** Same defect FAMILY as the
speed-cap finding in the next banner down — the phase clock ignores
part of the command — but a different axis and a different key. The
tip dose pair is in and BOTH arms park: `tip50-r2` and `tip90` both
FAIL-INFORMATIVE, eval_yaw tip err 0.2996/0.3000 and 0.2982/0.2999
== |wz_ref| exactly (parent fingerprint, artifacts
`logs/ckpt_eval/..._yawcmd_tip{50_r2,90}_yawgate.json`) — zero
dose-response 0.5→0.9, so COMMAND EXPOSURE IS REFUTED. Digging past
that: `walk_task._augment_obs` only advances the `walk_phase_obs`
clock while `s_ref>1e-3`, so during tip episodes (vx=vy=0, wz≠0) the
phase obs this BC-clone substrate is phase-locked to simply STOPS —
no time-base to step with, so it parks. Predicts every fingerprint:
zero rotation exactly at vx=0, unlearnable at any exposure, yet
yaw-while-TRANSLATING (clock running) improved in every arm
(fwd-hold drift err 0.169→0.075-0.080, arc-right 0.273→0.150-0.180).
FIX LANDED: `goal.walk_phase_run_on_yaw` (default 0 = bit-exact
legacy; =1 also advances the clock while |wz_ref|>1e-3; pure park
stays frozen) — walk_task.py + 3 new tests in
test_phase_speed_coupling.py (10/10; sim_env+phasedir 78/78;
semantics bank 163 pass / 1 PRE-EXISTING fail
`test_fastprof_obeying_the_command_beats_overspeed`, fails
identically on the pre-change tree — not this change). `_phase` is in
MJX_SNAPSHOT_EXTRA → pool-safe on warp. Tag
`exp/cw-amp-m2-bcinit-sec5-style05-yawcmd-tipclock-pair`. LAUNCHED
(batch): `-tip50-clockyaw` / `-tip90-clockyaw` — byte-identical
respecs of tip50-r2/tip90 + the clock key, so the parked arms are
matched controls; joint read: clock binding ⇒ command-signed tip +
dose-response reappears; both still park ⇒ clock refuted, next lever
= BC-turn-clone (TripodGait native omega + bc_init_gait extension —
which ALSO needs this key: collect() derives the teacher clock from
the same frozen phase obs). TRIAGE NOTES: (1) eval_yaw for these arms
MUST include goal.walk_phase_run_on_yaw=1 in the cfg-set or the eval
clock freezes again; (2) goal.walk_turn_in_place_frac rides the run
cfg into the standard harness eval — tip-arm DR-0 panels are
~frac-fraction parked episodes; panel medians look like collapse but
are contamination (tip50-r2's genuine translation episodes were fully
preserved: prog 0.94-1.11, slip 2.9-4.2, fwd to 1.02m). Recorded
assumption (no-operator-pause): clock-fix continuation funded before
the BC-turn-clone because it is one cfg key vs new tool code and its
control pair already exists. Speed-cap banner below.)

Previous entry (~23:1x (**SPEED-CAP ROOT CAUSE CONFIRMED:
the fixed-rate phase clock is the defect; style and clock RATE are
both exonerated. Fix landed in code; coupled-clock pair launched.**
The pre-registered fastphase / fastphase-nostyle joint read came back
NO-CHANGE on both arms: walk_phase_hz 1.333->2.0 kept the gait
perfectly clean (gait_valid 6/6 det+sto both arms, zero terms/sac
legs, clean six-leg strips) but realized speed stayed pinned
(fastphase det 0.075-0.112, nostyle det 0.074-0.118, vs parent
0.084-0.136) against 0.053-0.171 commanded — the band did not even
shift up under a 1.5x faster clock, and zeroing AMP style changed
nothing (twins statistically identical). Conclusion: no CONSTANT
clock rate can produce speed MODULATION; the clock's
speed-INDEPENDENCE (walk_task._augment_obs advances at fixed hz for
any s_ref>1e-3) is the defect, same fingerprint joystick's
phasedir9-seed17 found. **CODE LANDED (snapshot
exp/cadcouple-phase-clock):** `goal.walk_phase_speed_scale` (default
0.0 = bit-exact legacy) makes hz_eff = hz*(1+k*(s_ref/s_nom-1)) with
`walk_phase_speed_nom` (0.08) + `walk_phase_hz_max` clamp; sim_env's
bc_anchor_phase_lock branch scales its accumulator by the same ratio
so the anchor stays locked; bank
`rl_move/tests/test_phase_speed_coupling.py` 7/7 PASS +
test_bc_anchor 63/63 + test_phasedir_semantics 34/34 green.
**LAUNCHED:** `cw-amp-m2-bcinit-sec5-style05-speedrange-cadcouple-r1`
(style 0.5/0.5, train-2) + `-cadcouple-nostyle` (train-3), 2M
discovery each from the fastphase{,-nostyle} checkpoints, scale=1.0,
hz cap 3.0. Joint read: both widen => timing was the cap, done; only
nostyle widens => the fixed-cadence teacher_v2 library vetoes varying
cadence => build the cadence/speed augmentation (Next item 2's
STILL-OPEN half); both pinned => cap is stride/actuation or budget.
(Plain `-cadcouple` ledger entry is a REFUSED launch-tag collision,
not a run. Prior banner below.)

Previous entry (~23:0x (**TURN-IN-PLACE EXPOSURE REFUTED —
the tip dose pair is in and BOTH arms park.**
`cw-amp-m2-bcinit-sec5-style05-yawcmd-tip50-r2` and `-tip90` both
FAIL-INFORMATIVE (FAIL-park): eval_yaw tip-left/right err
0.2996/0.3000 (0.5 dose) and 0.2982/0.2999 (0.9 dose) == |wz_ref|
exactly — the parent's park fingerprint, zero dose-response from
50%→90% whole-episode turn exposure (artifacts
`logs/ckpt_eval/cw_amp_m2_bcinit_sec5_style05_yawcmd_tip{50_r2,90}_yawgate.json`).
Both arms DID improve yaw-while-translating (fwd-hold drift err
0.169→0.075-0.080, arc-right 0.273→0.150-0.180): the policy can
modulate wz inside the walking gait but produces ZERO rotation from
the parked stance at vx=0 at ANY exposure. Read: motor-pattern
DISCOVERY block — the exact failure shape of the crouch-statue basin
(0/4 sec5 arms) that only BC-init fixed. Pricing (yawcmd) and
exposure (tip pair) are now BOTH refuted; recorded assumption
(no-operator-pause): of the pre-registered structural options, the
BC-turn-clone is funded FIRST (teacher turns near-perfectly,
yaw_along 0.99/1.01; TripodGait.set_velocity has a native omega
channel bc_init_gait.py simply never drives; mirror-symmetry
regularizer is the fallback). TRIAGE GOTCHA recorded in the tip50-r2
verdict: goal.walk_turn_in_place_frac rides the run cfg into the
standard harness eval, so tip-arm DR-0 panels are ~frac-fraction
parked episodes — panel medians look like gait collapse but are
contamination; judge translation from the non-tip episodes (tip50-r2's
were fully preserved: prog 0.94-1.11, slip 2.9-4.2, fwd to 1.02m).
Prior banner below.)

Previous entry (~22:4x (**YAW-COMMAND QUESTION ANSWERED FOR
PRICING: `cw-amp-m2-bcinit-sec5-style05-yawcmd` FAIL-INFORMATIVE —
even on the drift-free AMP substrate with real turn clips in
teacher_v2 and the full bank-verified yaw pricing stack, the walker
does NOT turn on command.** eval_yaw panel (artifact
`logs/ckpt_eval/cw_amp_m2_bcinit_sec5_style05_yawcmd_yawgate.json`):
tip-left/right err 0.2995/0.3008 == |wz_ref| exactly — the robot
PARKS on turn-in-place commands; while translating it carries a
command-invariant LEFT drift ~0.12-0.17 rad/s. Alignment read per
the 08-21 ruling: total reward rose 66->316 while env/walk_yaw_err
ROSE 0.129->0.230 (yaw income outbid by translation+style), so this
is misaligned/underexposed, NOT undertrained — no plain continuation.
Gait and translation were fully preserved (gait_valid 6/6 det+sto,
det prog 1.22/slip 2.81, dir_err det 32.9/sto 49.0 == headingsfull's
band). **PRE-REGISTERED NEXT LEVER LAUNCHED THIS CYCLE:** the 08-10
operator-directed command-EXPOSURE curriculum
`goal.walk_turn_in_place_frac` (whole-episode dedicated turns, 50/50
sign draw — already in walk_task.py, never tried on this substrate)
as a dose pair from the yawcmd checkpoint:
`cw-amp-m2-bcinit-sec5-style05-yawcmd-tip50-r2` (0.5, translation
must hold) + `-tip90` (0.9, erosion tolerated). Joint read: either
dose turns command-signed => exposure was the unlock, re-mix next;
both still park => exposure refuted, next lever is STRUCTURAL
(mirror-symmetry regularizer / turn-specific gait phase). NOTE for
the triage cycle: the watcher's standard evals do NOT measure wz —
run `rl_move/sim/eval_yaw.py` on the run's pod with the run's cfg
(--speed 0.08 --wz-max 0.3), as done for yawcmd this cycle.
`-tip50`/`-tip50-r1` are REFUSED launch stubs from snapshot races
with a concurrent cycle, nothing trained. Prior banner below.)

Previous entry (~22:2x (**STAGE-3 SPEEDRANGE VERDICTED
INFORMATIVE (partial): the walker survives the 0.05-0.25 m/s speed
command range cleanly but barely modulates speed on it.**
`cw-amp-m2-bcinit-sec5-style05-speedrange` DR-0 gate: gait_valid 6/6
det + 6/6 sto, zero terminations/sacrificed legs, height_err stayed
in the walking band (training env/height_err_mm 18.5mm at 2M, no
crouch), real per-episode travel 0.12-1.36m/15s, reward rose
monotonically 50->226. But achieved speed_mean clusters 0.084-0.136
det / 0.092-0.126 sto regardless of the per-episode commanded speed
(est. 0.053-0.171 in this sample) — a 5x commanded range compresses
into <2x realized range centered near the OLD fixed-0.08 pin; this is
exactly the pre-registered "low band tracked, top band capped"
partial, not a clean pass or a collapse. Root-cause lead (already on
record from the joystick track's own phasedir9-seed17 dig-in): the
`goal.walk_phase_hz` obs clock advances at a FIXED rate whenever any
nonzero speed is commanded — the actor's step cadence was never made
a function of commanded speed, so more speed can only come from
longer strides at the same step rate, which this codebase has already
seen plateau before (`cw-walk2-gait`: doubled stride, speed still
capped ~0.045 m/s). **FOLLOW-UP PAIR LAUNCHED THIS CYCLE** (both from
the speedrange checkpoint, fresh disc, 2M): `-speedrange-fastphase`
(single lever: `goal.walk_phase_hz` 1.333->2.0, same 0.5/0.5 style)
tests whether a faster reference clock lets the actor step faster and
widen the realized speed band; `-speedrange-fastphase-nostyle` (same
clock change PLUS `amp-task-weight=1.0/amp-style-weight=0.0`) isolates
whether the AMP discriminator — anchored to teacher_v2's clips at
their ORIGINAL recorded cadence — is itself vetoing faster-than-
demonstrated stepping. Joint read decides: both widen => phase_hz was
the cap; only nostyle widens => style/motion-library cadence is the
real cap; neither widens => the ceiling is elsewhere (actor capacity
at this budget) and the current ~0.05-0.14 m/s envelope should be
accepted as this stage's real ceiling (still within/near the teacher's
own 0.06-0.10 m/s hardware band) rather than spending more discovery
budget chasing 0.25 m/s. Prior banner below.)

Previous entry (~22:2x (**noamp-headings90-r1 PASS closes
the noamp lineage's own record of the +/-90deg rung** — gait_valid
6/6 det+sto, zero sacrificed legs, prog med det 1.12/sto 0.85, dir_err
med det 39.2/sto 51.9, height_err single-digit mm, essentially a wash
vs the style05 twin (style still NOT functionally protective at this
dose, matches headings20's read). No further noamp continuation
queued on this specific rung (the style05 lineage already advanced
past it via headingsfull); a concurrent cycle has since started
`cw-amp-m2-bcinit-sec5-noamp-headingsfull` independently. **NEW M4
LEVER LAUNCHED THIS CYCLE:** `cw-amp-m4-faultobs1-{noamp,control-r1}`
— wires the already-built-but-untrained `obs.fault_health=1` (18-dim
fault vector, test_fault_injection.py 14/14) into the faultsmoke1
pair via `--obs-pad-transplant 18` (zero-padded, bit-identical at
init), single lever vs faultsmoke1-{noamp,control}: can the policy
actually use SIGHT of its own fault to compensate better than the
faultsmoke1-noamp blind baseline (det gait_valid 5/6, faulted-episode
prog_ratio 0.49/slip 5.99), with -control-r1 as the no-fault sanity
twin isolating the obs-widening itself from the fault-seeing effect.
First real M4 adaptation reading (M4 was NOT STARTED before this).
**BOTH VERDICTED PASS 08-22 (~22:3x):** same eval seed reproduces
the IDENTICAL per-episode fault draw as faultsmoke1-noamp, giving a
clean apples-to-apples read — the faulted episode (leg[5] disabled)
improved prog_ratio 0.495->0.585 (+18%) and slip/m 5.987->4.378
(-27%) with fault-sight vs blind; a second degraded episode (a
non-disabled fault type) also improved (slip -21%); sto mode
improved in 4/6 episodes. Not a single-episode fluke, and the
control-r1 sanity twin (fault_prob=0) confirms the obs-pad transplant
itself is a true no-op (numbers indistinguishable from
faultsmoke1-control). Caveat stated bluntly: the disabled leg still
visibly drags in the frame strip either way — this is a measured
COST REDUCTION, not a clean masked-fault gait. M4 now has its first
real (non-smoke) adaptation evidence; fault_health obs-wiring is a
proven lever, not just a built-but-untested mechanism.
Prior banner below.)

Previous entry (~22:0x (**HEADING CURRICULUM COMPLETE AT
DR-0 — `cw-amp-m2-bcinit-sec5-style05-headingsfull` VERDICTED PASS:
the one-step jump from +/-25deg to FULL-CIRCLE headings (incl.
backward, never in the BC init) WORKS; the +/-90deg intermediate
rung is unnecessary.** DR-0 gate at own full-circle range:
gait_valid 6/6 det + 6/6 sto, zero sacrificed legs/terminations;
det prog med 1.19 / slip 2.77 / fwd 0.47m, sto 0.87 / 3.57 / 0.56m.
Direction NOT bimodal — rear-hemisphere commands followed (worst
det ep dir_err mean 39deg, wrong-way <=0.11, along/cmd 1.0-1.4);
det dir_err med ~33 / sto ~50 vs the ~90deg ignore baseline.
height_err 20.5mm all 2M (no crouch), reward 5.9->289 then plateau
(no continuation case), disc healthy (d_real 0.50 / d_fake -0.80,
style 0.22). Matches/beats the parallel headings90 rung (det prog
1.13 / slip 3.03 / dir 35.2). Det frame strips watched: upright
six-leg cycling, real off-axis displacement. Weak axis: sto
adherence loose (med ~50deg, single-ep p90 to 146deg) — later
stages must tighten command-following. **STAGE-3 SINGLE-LEVER PAIR
LAUNCHED from the headingsfull checkpoint (this cycle, 2M discovery
each, DR-0, fresh disc):** `cw-amp-m2-bcinit-sec5-style05-{speedrange,
yawcmd}` — speed 0.08 fixed -> 0.05-0.25 m/s (speed modulation on
command; stops already trained via park_start_frac) and yaw-rate
commands +/-0.3 rad/s (first turning demand; obs 73->74 via the
proven --obs-pad-transplant 1 tail-append; FULL landed bank-verified
turn pricing set, NOT the known-failed kernel-only subset — this
substrate has no chirality drift and teacher_v2 has real turn clips,
so the old champion-line turn failures do not pre-decide this).
Prior banner below.)

Previous entry (~21:5x (**FAULT INJECTION CLEARED FOR REAL
TRAINING USE — both `cw-amp-m4-faultsmoke1-{control,noamp}` VERDICTED
PASS** on the smoke's own mechanism-safety bar (NOT a graded M4
gate): the fault twin (`dr.fault_prob=1.0`, one random weak/frozen/
disabled-leg fault per episode, policy has ZERO observation of the
fault) trained the full 2M with finite/rising reward (9.3->209.0/
quarter, no NaN/crash) and its DR-0 gate shows visible compensation,
not collapse — det gait_valid 5/6 (1 ep sacrificed leg, degraded not
dead: prog_ratio 0.49 slip/m 5.99), sto 6/6, all 6 det episodes'
video/contact-sheets show genuine six-leg cycling and net travel
including the degraded one. The matched no-op control twin
(`dr.fault_prob=0.0`) stayed clean 6/6 det+sto with BETTER numbers
everywhere (prog_ratio 1.16/0.88, slip 2.36/4.08, reward 31.7->320.8)
— isolates the fault twin's softer numbers as the fault's real cost,
not just 2M more steps of noise. Ledger note: -control's status was
stuck at stale INTENT despite the pod-side train log showing a clean
finish at 2,031,616 steps; the prestage skipped its gate/owncfg evals
(orchestrator.log shows only wandbdump+pullckpt ran, no eval spawn)
so both gate evals were run manually on-pod (train-3/train-2) this
cycle per the fallback instruction. SKILLS.md updated (AMP section).
**NEXT LEVER: wire `fault_health()` (18-dim, already built) into
actor obs** — this smoke proves the mechanism is safe to train
against but the policy still can't SEE which fault it has, so it can
only compensate blindly; obs-wiring is the real M4 milestone item,
un-started, and the natural next step given free capacity. Prior
banner below.)

Previous entry (~21:4x (**HEADING CURRICULUM STAGE 1 PASS —
`cw-amp-m2-bcinit-sec5-style05-headings20` VERDICTED PASS: the
BC-init AMP walker survived +/-25deg headings AND got better.**
DR-0 gate at own heading range: gait_valid 6/6 det + 6/6 sto, zero
sacrificed legs/terminations; det prog med 1.30 / slip 2.17 / fwd
0.64m vs parent 1.16 / 1.88 / 0.69m; sto improved outright (prog
0.58->0.93, slip 4.71->3.38, fwd 0.23->0.65m); dir_err improved both
modes (det 33.8->26.9deg, sto 61.9->48.2deg). height_err held
22-26mm all 2M (no crouch), reward rose 19->247 monotonically, disc
healthy (d_real 0.47 / d_fake -0.75, style_reward 0.25). Frame
strips watched (det_0 straight, det_4 off-axis): upright six-leg
cycling, real displacement. Weak axis, stated bluntly: direction
ADHERENCE is loose (det dir_err 26.9deg on a 25deg command range) —
it walks well but only roughly where told; later stages must turn
this into command-following (dir_err tracked explicitly in stage-2
gates). Prediction-if-true fired exactly: heading diversity did NOT
re-trigger the exploration collapse from a walking init. **STAGE-2
DOSE PAIR LAUNCHED (this cycle):** `cw-amp-m2-bcinit-sec5-style05-
{headings90,headingsfull}` — both continue from the headings20
checkpoint, single lever each: walk_heading_max_rad 0.4363 ->
1.5708 (+/-90deg, first lateral demand) and -> -1 (full circle,
first backward demand — the operator's small-set->full jump tested
in one step). 2M discovery each, DR-0, seed 7, fresh disc per stage
protocol. The noamp-headings20 twin was still training and belongs
to its own triage cycle; its lineage's stage-2 should mirror this
pair if it passes. SIDE NOTE (cross-track): the paper-CPG
contextual-250 search COMPLETED — winner beats trial-43 5x on the
fixed contextual scorer (see CURRENT_TRUTHS.md); NO automatic
teacher/motion-library swap — adoption is a pre-registered fork,
q_20260822T2140Z. Prior banner below.)

Previous entry (~21:2x (**BC-INIT BATCH BOTH ARMS PASS —
first non-statue from-scratch-M2 result. Both `cw-amp-m2-bcinit-
sec5-style05` and its noamp twin VERDICTED PASS**: DR-0 gate
gait_valid 6/6 det+sto on BOTH arms, zero sacrificed legs, real net
travel (style05 det fwd med 0.69m/15s prog med 1.16; noamp 0.64m/
prog 1.09), env/height_err_mm stayed 18-31mm the WHOLE run on both
(never the 59-85mm crouch signature of every from-scratch sec5 arm)
— frame-strips of walk_det_4/walk_sto_3 watched, clean six-leg
alternating-tripod cycling with visible checkerboard displacement,
not the splayed statue. ROOT CAUSE: the 0/4 sec5-grid FAILs were an
EXPLORATION problem (PPO from a random init never finds the walking
basin under the minimal Gaussian-velocity+progress reward), NOT a
reward-shape defect — confirms brief §4.3 ("gait as an initialization
only") and §14 ("stop raw PPO from random initialization") exactly.
Style read: style05 edges noamp on every axis (det slip 1.88 vs 2.13,
sto slip 4.71 vs 5.38; prog both modes higher) — modest, consistent,
not yet proven outside n=6 noise. Config is forward-only / clone-
compatible obs (phase obs, body_vel=2, no yaw-cmd obs, no
asym-critic) — NOT yet the full M2 milestone (turn both ways/
backward/lateral untested). **NEXT LEVER LAUNCHED (this cycle):**
`cw-amp-m2-bcinit-sec5-{style05,noamp}-headings20` — continuation
from each checkpoint (`--init-from-source`), single lever change
`goal.walk_heading_max_rad` 0.0->0.4363 rad (25°, stage 1 of the
operator's own untried forward-only->small-set->full->irregular
staged curriculum, fb_20260822T003132 — previously only pre-
registered on the joystick track's phase-RL lineage, which reached
its own DONE gate before trying it; applied here for the first time).
Prediction-if-true: gait_valid stays >=5/6, height_err stays near
18-31mm under turning demand. Prediction-if-false: heading diversity
re-collapses the gait toward statue/drag even from a walking init —
motivates a smaller first stage or a turn-in-place sub-skill.
2M each, DR-0. Prior banner below.)

Previous entry (~20:0x (**SEC5 GRID FINAL 0/4 — the
section-5 minimal reward is NOT the M2 fix; prediction-if-false
branch fires. NEXT LEVER LAUNCHED: BC-clone-initialized batch.**
taskB (0.5/0.5), taskC (0.3/0.7) and the task-only noamp control all
VERDICTED FAIL-same-statue alongside taskA: DR-0 gate det gait_valid
1-3/6 with sacrificed legs ([0,4]/[1,3]), prog med 0.01-0.02, fwd med
0.01-0.03m/15s (bar 0.10), slip 9-14; contact sheets watched — the
same crouched splayed statue holding one pose, zero translation.
Shared mechanism: env/height_err_mm jumps 59->85mm in Q1 (the policy
crouches away from upright the moment training starts) in EVERY arm
including noamp — the basin exists WITHOUT AMP and no style dose
(0.3/0.5/0.7) rescues it; disc stayed healthy/unsaturated (d_real
~0.78/d_fake ~-0.96) while style_reward decayed (taskB 0.172->0.073,
taskC 0.177->0.113). No arm had rising reward (taskB -27->-38, noamp
-60->-132, taskC flat within noise with eval flat = stuck mechanism)
— genuine FAILs per RUN_INTERPRETATION_RULES, reward-shape tuning on
from-scratch M2 is CLOSED. Per the grid's own pre-registration and
brief sec4.3 ('the gait as an initialization only') / sec14 (stop
'raw PPO from random initialization'), LAUNCHED the BC-init batch:
`cw-amp-m2-bcinit-sec5-{style05,noamp}` — the verified scripted-gait
BC clone (`ppo_goal_cw_bcgait_init_fullprof_phase1`, walks with zero
RL) as `--init-from`, sec5 minimal reward verbatim (bank PASS),
clone-compatible obs/env (phase obs, body_vel=2, fast servo,
stress_mix fixed 0.08; yaw-cmd obs + asym-critic dropped for
init compatibility), warm-log-std -2.0, seed 7, DR-0, 2M each.
Decides the fork: init escapes the basin (walking survives ->
first controlled style-vs-noamp measurement on a locomoting actor)
vs even a walking init collapses (the sec5 reward itself destroys
locomotion -> task restructuring of height/upright/termination
pricing is the next lever). Assumption re 'from-scratch by design'
recorded in OPERATOR_QUESTIONS.md (q_20260822T2000Z). Prior banner
below.)

Previous entry (~19:0x, MECHANICAL NOTE on the entry below:
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
tuning). **taskA VERDICTED FAIL 08-22 (~19:2x, task/style 0.7/0.3)**:
fires the prediction-if-false branch — DR-0 gate det gait_valid 0/6
(legs [3,5] sacrificed), prog_ratio 0.02, slip/m 9.41, dir_err
68.7deg; sto gait_valid 4/6 but slip/m 15.20, dir_err 77.1deg,
prog_ratio 0.03, only 3/6 settled; video shows a crouched near-static
pose shuffling legs with no net translation, same march-in-place
basin as every prior M2 arm. Training reward DECLINED every quarter
(-39.3/-44.6/-72.5/-74.2, not the 08-21 rising-reward case): the
mechanism trace shows `env/height_err_mm` jumping 5.8->87.7mm in Q1
(policy actively crouches away from the upright target as soon as
training starts) and only partially recovering to 58.5mm by 2M;
`amp/style_reward_mean` rose slowly (0.009->0.106, disc healthy,
d_real 0.78/d_fake -0.95, unsaturated) but a 0.3 style weight is too
thin a share of the blend to rescue the crouch. taskB's early peek
(0.5/0.5) shows the identical pattern (det gait_valid 3/6 legs[0,4]
sacrificed, sto gait_valid 6/6 but slip 13.88/dir 76.1, still no net
travel) — 2 of 4 grid arms now confirm the same basin regardless of
task/style ratio; taskC/noamp (owned by a concurrent cycle) still to
land before the joint grid conclusion. Previous update ~18:2x
retained below (TASK/STYLE DOSE
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
- M3 push recovery: IN PROGRESS (08-23 ~00:0x) — built + bank-tested
  `dr.ext_push_*` (brief §7.4/§9.3): a mid-episode random-direction
  horizontal force pulse (10-25N/0.15-0.4s, half-sine, world-frame
  direction, once per walk-mode episode at a random 1.5-9s delay),
  distinct from the pre-existing `dr.walk_push_*` (a fixed roll TORQUE
  confined to the first ~1.5s that reproduces the hardware TAKEOFF
  wobble, not a recovery test). Wired across CPU sim_env AND the
  MJX/Warp batched stepper (mjx_backend/mjx_vec_env/
  mjx_sharded_vec_env); default OFF, bit-exact when off (guarded draw,
  same convention as fault/kick/rock/tipped). `xfrc_applied[chassis,
  0:3]` writes are OWNERSHIP-GATED to episodes that actually drew a
  push this episode, so the mechanism cannot clobber unrelated
  interactive tools that already use those same indices
  (`web_session.py`'s manual push slider, quad probes) — a real
  collision caught and fixed before launch, encoded as a permanent
  regression test. First training use, mechanism-safety smoke
  (`cw-amp-m3-pushsmoke1-noamp-r4`, 2M, DR-0): **PASS** — reward
  finite+rising every quarter, training tilt terminations fell ~3x at
  constant dose, DR-0 gate gait_valid 6/6 det+sto, zero sacrificed
  legs; blunt: the shove still won 1/6 det + 3/6 sto episodes
  (survive-most, not recovery-after-knockdown). Style twin
  (`cw-amp-m3-pushsmoke1-style05`) and a 6M acquisition continuation
  (`cw-amp-m3-pushacq1-noamp`) launched off the back of that PASS.
  **Acquisition PASS (08-23)**: 6M more steps at the SAME dose closed
  the gap outright — DR-0 own-cfg gate 0/12 terminations det+sto (was
  1/6 det + 3/6 sto), gait_valid 6/6 both, zero sacrificed legs, det
  prog med 1.06 (bar ≥0.9), height_err single-digit-to-12mm all
  episodes; training tilt terminations kept falling the whole 6M
  (pitch 18.6→7.5, roll 11.4→5.4/window), reward rose 143→381 —
  reward and gate agree. Video (the two hardest-hit episodes, roll
  peak 10-16°): clean six-leg cycling continues through and after the
  shove. Blunt limit unchanged: still ONE push per episode — M3's own
  bar wants REPEATED pushes, which `dr.ext_push_*` could not draw
  before this cycle. **BUILT + BANK-TESTED 08-23**:
  `dr.ext_push_repeat_max` (RandRanges dose menu, default 1 = the
  original single-pulse behavior byte-for-byte — the repeat-sampling
  branch only runs when >1, so it draws zero extra rng numbers at the
  default) draws that many independent pulses per episode, each timed
  `dr.ext_push_gap_s` after the previous one ENDS (non-overlapping by
  construction, so the policy gets a real window to recover between
  shoves) and stopping early (fewer pulses that episode) once the next
  start would land past `dr.ext_push_horizon_s` — a short/fast episode
  gets fewer pulses, not one crammed at the tail. `sim_env._ext_push_force_n()` sums every pulse's half-sine
  contribution (at most one nonzero at a time by the non-overlap
  sampling). Same dose-not-scaled convention as every other push/kick/
  fault menu (`.scaled()` shrinks only `ext_push_prob`). Bank:
  `test_ext_push_injection.py` 17/17 (12 original + 5 new: default
  bit-exact at repeat_max=1, dose menus obeyed for every extra pulse,
  non-overlap + monotonic timing, early-stop past horizon,
  force-summation in each pulse's own window); full regression
  `test_sim_env.py` 44/44; full semantics bank 163 pass/4 skip/1
  xfail/1 pre-existing-red (fastprof, untouched) — zero collateral
  damage. First training use launched (mechanism-safety smoke, same
  discipline as fault/single-push): `cw-amp-m3-pushrepeat1-noamp`
  (from `cw-amp-m3-pushacq1-noamp`'s checkpoint, `dr.ext_push_repeat_
  max=3`, `dr.ext_push_gap_s=(1.0,2.5)`, `dr.ext_push_horizon_s=13.0`,
  2M, DR-0) — decides whether the single-push acquisition champion's
  recovery skill generalizes to a second/third shove in the same
  episode, or needs its own acquisition budget.
- M4 fault adaptation: IN PROGRESS (fault injection built+trained;
  faultobs1/2 sighted-vs-blind line — see Now banners)
- M5 MuJoCo transfer (= DONE gate): **SUITE BUILT 08-23**
  (`rl_move/sim/eval_amp_m5.py`, amp-m5-v1: ONE plain-MuJoCo
  invocation composing walk/yaw/push/fault sections with
  pre-registered bars from the M2/M3/M4 lineage gates —
  q_20260823T0130Z records the assumed bars + one-checkpoint
  semantics; the trainer is MJX/Warp so every section IS the
  cross-engine replay the milestone demands). First baseline read on
  the M2-yaw champion `yawcmd0-r2` (det-only v1 pass): walk PASS
  (0 terms, prog 1.11, slip 2.03), yaw PASS (tips 0.15/0.16), push
  PASS at base 10-25N dose (2/6 det topples, bar <=2 — zero push
  training!), fault FAIL. FULL det+sto 15s read (final, faithful to
  the lineage gate — walk section reproduces the champion's own gate
  numbers exactly, det prog 0.9455/slip 2.235): walk PASS 12/12
  0 terms; yaw PASS tips 0.1525/0.1614; push PASS 2/6 det + 2/6 sto
  topples, gait_valid 12/12; fault FAIL — 0 terminations and det
  prog med 0.85 (it limps, never falls) but gait_valid 9/12 with
  sacrificed legs {2,3,5} (strip watched: faulted leg carried high,
  classic sacrifice) — m5_pass=false. The measured consolidation gap
  is FAULT-VALID GAIT (exactly what the faultobs/turnfault lines
  train) plus push margins at hardened doses; artifacts
  `logs/ckpt_eval/cw_amp_m2_turnclone_yawcmd0_r2_m5/`.

## Now

**UPDATE (08-23 ~16:1x) — DEMO-ANCHOR FORK (`-cpgdemo1`) FAILS, and closes the LAST named slip-mechanism lever on this lineage; ALSO found the walk-slip metric's real defect (thin-sample noise, not a hidden-axis dissociation).** Weight-movement precheck PASS (12/12 non-log_std tensors moved, log_std unchanged ~-2.0 as expected — this arm never touched anneal). Both slip instruments moved TOGETHER this time, and in the WRONG direction: m5 walk det_slip_med 4.3685 (parent 3.67, bar 3.5 — worse by +0.70) AND the slipdist probe (861 stances) reads 15.84mm vs the matched pushcal518_ctrl 14.03mm (worse, and far above both anneal rungs 9.55/9.6). Swapping to cpg_v1.npz (its own recorded forward_0.08 clip slip 0.324 vs teacher_v2's 0.527, -39%) does NOT transmit that lower slip into the trained policy's behavior — REFUTES the demo-anchor hypothesis cleanly. Yaw shows the now-familiar mixed asymmetric signature: tip_left 0.1936 IMPROVES and clears 0.20 (matches the predicted ccw-favorable asymmetry) but tip_right 0.2646 is worse than parent's 0.2351 — the SAME one-side-better-one-side-worse pattern already seen under pricing (k_yaw_prog), demos (teacher_v3), densification (tipspawn1b), and noamp1 (tip gain traded for slip cost) — four-plus independent mechanism classes now show this shape, suggesting a structural (stance-geometry/joint-authority) asymmetry the reward/demo/pricing layer cannot reach, not a to-be-found incentive fix. Fault gait_valid 10/12 with 2 sacrificed legs [2,5] — same degraded margin as `-stdanneal50`, a second independent lever now paying this same fault-safety cost. Push PASS. Safety clean (0 falls, video-reviewed clean six-leg cycling both sections). **Tally: every named slip lever on this lineage (additive pricing 6x/12x, income gate at 0.5x/1.0x, gait-income k_walk_swing, turn/reset densification 2x2, noise-floor anneal 2 rungs, demo-source swap) is now refuted or dose-saturated.** Per the gate's own text this forces the q_20260823T0700Z bar-amendment question — but this cycle also found the LIKELY REAL EXPLANATION for the whole "probe moves, m5 doesn't (or moves opposite)" saga: `eval_amp_m5._walk_stats` already excludes turn-in-place episodes from `det_slip_med` (`_translating()`, pre-existing code) — but at the default `--per-mode 6` only **`n_translating` is typically 3 of 12 walk episodes** (confirmed on both `-stdanneal50` and `-cpgdemo1` this cycle), i.e. every walk-section slip verdict this whole dose-grid history has been a median of THREE episode-level slip values. A single unlucky/lucky translating episode can swing that median by the same ~0.3-0.7 magnitude every arm's "win" or "loss" has been read on — the apparent dissociation from the (900-stance, low-noise) probe may be mostly THIS sampling thinness, not a hidden second mechanism. Assume-and-go resolution (no operator pause): before verdicting any FURTHER slip-mechanism arm on this lineage, bump `eval_amp_m5`'s walk-section sampling (`--per-mode` 6 -> 12+, or a dedicated `--walk-per-mode` override) so `n_translating` lands at >=6-8, not 3 — cheap (no code change, just a CLI default/flag), and should be done before spending another training arm on this axis. Recorded as the amendment to q_20260823T0700Z; the bar VALUES (3.5, 0.20) stay unchanged pending the operator, only the SAMPLE SIZE backing them is flagged as inadequate. Evidence: `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_cpgdemo1_m5/m5_verdict.json`, `logs/ckpt_eval/cpgdemo1_slipdist.json`.

Previous entry (08-23 ~15:5x) — NOISE-FLOOR ANNEAL DOSE GRID CLOSES AT `-stdanneal50` FAIL: the mechanism SATURATES at -4.5 and going deeper actively costs slip/tips/fault-safety, not just plateaus. Demo-anchor fork LAUNCHED.** Weight-movement precheck PASS (12/12 non-log_std tensors moved vs both ancestors, log_std annealed to -4.95, near the -5.0 target). Slipdist probe (hazard-free own-cfg, seed 0, 6 eps, 934 stances): median 9.6mm vs the `-stdanneal45-r2` rung's own 9.55mm — flat, no further gain, hits the gate's explicit FAIL trigger ("probe median >9.0, no gain over the -4.5 rung"). Worse, the m5 suite MOVED IN THE WRONG DIRECTION on every axis it touches: walk det_slip_med 4.1065 (parent 3.67, -4.5 rung 3.71, bar 3.5 — regressed, not flat); yaw tip_left/right 0.205/0.2668 (vs -4.5 rung's 0.2088/0.2287 — tip_right now clears even the loose 0.25 band); fault gait_valid dropped to 10/12 with 2 sacrificed legs (vs the -4.5 rung's clean 12/12, 0 sacrificed — a real robustness cost). Push section still PASS (0 terms, gait_valid 12/12). Safety intact throughout: 0 raw falls any section, video-clean upright six-leg cycling on walk + fault contact sheets. **This reconfirms the probe-vs-m5 dissociation (q_20260823T0700Z): stance-travel keeps dropping in lockstep with the -4.5 rung while m5 walk slip and yaw tips get WORSE — the m5 walk-slip metric is dominated by something the per-stance-travel probe doesn't capture (stress_mix turn-in-place phases remain the leading suspect).** CONCLUSION per the gate's own pre-registered branch: stop dosing this lever — the train-noise floor bought its one real win at the -4.5 rung and further dosing is net-negative. **REFILL, same cycle: launched the gate's prescribed next lever, the demo-anchor fork** — `cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-cpgdemo1` (single lever: `--amp-motion-lib` teacher_v2.npz -> cpg_v1.npz on the unchanged pushcal518 full turn+fault+push recipe, VERIFIED RUNNING train-2, 2M discovery). `cpg_v1.npz` (built 08-23, cpg track) is the CPG-search-optimized motion library already A/B'd against teacher_v2 on the SIMPLER `cw-cpg-teacherfork-ab` recipe (co-equal there) but never tried on this harder composed lineage where slip has been immovable by every reward-side and noise-floor lever tried so far (pricing 6x/12x, income gate, swing income, turn demos, densification, RSI, now anneal dose). cpg_v1's own recorded clip slip is measurably lower than teacher_v2's at the exact operating speed (forward_0.08 slip/m 0.324 vs 0.527, -39%); its turn clips achieve a CCW/CW asymmetry (ccw ~80% of commanded wz vs teacher's own ~50-60% ratio; cw ~55%, roughly on par) so a tip-symmetric fix is not guaranteed. Gate pre-registered: PASS clears walk slip <=3.5 + tips within 0.25 both signs + fault gait_valid>=10/<=1 sacrificed with 0 falls; PARTIAL = slip moves >=0.15 toward 3.5 without clearing, or one tip clears while the other doesn't; FAIL = slip/tips unmoved (+-0.15 of pushcal518's own numbers) or any new fall/fault regression, closing the demo-anchor mechanism too and forcing the q_20260823T0700Z bar-amendment ruling as the last open lever. Evidence: `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_stdanneal50_m5/m5_verdict.json`, `logs/ckpt_eval/stdanneal50_slipdist.json`.

Previous entry (08-23 ~14:3x) — `-tipspawn2-startonly` FAIL on its pre-registered branch: the RSI-reset-distribution half ALONE does not reproduce tipspawn1b's slip win; 2x2 isolation now 3/4 cells read.** `eval_amp_m5` walk det_slip_med=3.6015 (bar <=3.5), close to parent pushcal518's 3.67 (delta 0.07) and far from tipspawn1b's combined-dose 3.1855 (delta 0.42 >>0.15) — lands squarely in the pre-registered FAIL branch. Safety/gait unaffected (walk 0/12 falls, gait_valid 12/12; push PASS det_slip_med 3.5045 gait_valid 12/12; fault PASS gait_valid 12/12; yaw FAIL as expected, this arm never targeted yaw). Design table so far: (start_frac=0, spawn_wz=0)=parent 3.67; (0.5, 0)=this run 3.6015 (unmoved); (0.5, 1.0)=tipspawn1b 3.1855 (moved). **LAUNCHED the 4th cell, `-tipspawn3-wzonly`** (spawn_wz=1.0 alone, start_frac reverted to 0, 2M discovery, train-0): if its slip also lands near 3.67, the win requires the TRUE INTERACTION of both levers (promote the combined tipspawn1b dose, not either alone, pending a same-cfg reseed to rule out n=12 noise); if it lands near 3.1855, spawn_wz alone is the mechanism (promote standalone, start_frac was never needed). Evidence: `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_tipspawn2_startonly_m5/m5_verdict.json`.

Previous entry (08-23 ~12:0x) — PRICING-NUDGE DOSE GRID CLOSED AT n=4, BOTH LEVERS REFUTED: neither yaw-income nor slip-charge single-lever dosing recovers the walk-slip/yaw-tip margin.** All 4 arms of the pushcal518 dose grid (launched last cycle, `rl_docs/tracks/amp/STATUS.md` ~11:4x) finished and are VERDICTED FAIL, safety intact throughout (0/12 raw falls, gait_valid 12/12, video-clean six-leg gait on every arm):
- **Yaw axis** (`k_yaw_prog` 1.0->2.0->3.0, `-yawprice2`/`-yawprice3`): tip_left/right errs vs parent's 0.2157/0.2351 — 2x: 0.2164/0.2596 (right WORSE by 0.024); 3x: 0.2472/0.2206 (left WORSE by 0.032, right marginally better but inside noise). Each dose makes ONE side worse; neither clears the 0.20 bar. Joint FAIL closes this lever: **turn-in-place tip accuracy is not purchasable by raising k_yaw_prog income alone.**
- **Slip axis** (`k_loadslip_excess` 0->6.0->12.0 at `loadslip_ok=1.5`, `-slipexcess6`/`-slipexcess12`): walk det_slip_med vs parent's 3.67 — 6x: 3.629 (0.04 better, needed 0.15 for PARTIAL); 12x: 3.7905 (0.12 WORSE than parent, and worse than the 6x dose too — non-monotonic in the wrong direction). Joint FAIL closes this lever: **additive per-tick loaded-slip charging does not reduce measured slip, and stacking more charge makes it worse, not better.**
- **Calibration note for the dig-in (slipexcess12 triage cycle, 12:1x): if the redesign picks the partial-strength loadslip INCOME gate, `reward.loadslip_max` MUST be raised above the operating point.** `walk_task.py` computes `factor = clip((loadslip_max - ratio)/(loadslip_max - loadslip_ok), 0, 1)` with defaults ok=0.75/max=1.50 — at the family's operating ratio ~3.5-3.8 the factor is 0 with ZERO gradient, so a gate arm at defaults is a constant income tax carrying no slip signal (a no-information arm). E.g. ok=1.5 (bank-calibrated) + max~4.5 puts factor ~0.3 at ratio 3.6 with usable slope. Also note slipexcess12's economics: the policy paid ~0.98/tick (~1/3 of walk income) rather than slip less — any income-side lever must beat a marginal price of ~0.47/tick per unit ratio that was already refused.
- **Conclusion:** both halves of the "reward is underpriced" hypothesis from the ~11:4x dig-in are now refuted at the dosing level. The walk-slip/yaw-tip miss against the v1 M5 bars is not a simple income-magnitude problem on either axis. Next lever is STRUCTURAL, not more dose: for yaw, stance-geometry or a dedicated turn-in-place wz curriculum stage (the policy may not have the joint-space authority/gait-phase coupling to rotate faster while satisfying its other stance constraints, not just insufficient incentive to try); for slip, either a partial-strength loadslip INCOME gate (discount the forward-progress reward term under high loaded slip, rather than an additive side-charge that's apparently not on the policy's easily-reachable cost gradient) or a root-cause check on what is actually driving the measured slip (foot-plant/gait-timing artifact vs. a genuinely controllable action). Flagging DIG-IN for the mechanism redesign on both axes — this is a reward/env-code decision, not a triage-cycle call. Evidence: `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_{yawprice2,yawprice3,slipexcess6,slipexcess12}_m5/m5_verdict.json`, ledger verdicts on all 4 runs (08-23 ~12:0x).

**REFILL, same cycle (12:0x-12:1x) — LAUNCHED a structural diagnostic while the dig-in redesign is pending: `-noamp1` (respec of `pushcal518`, `--amp-style-weight` 0.5->0.0, single lever, VERIFIED RUNNING train-2).** New finding motivating it: `teacher_v2.npz`'s `turn_ccw`/`turn_cw`/`forward_turn_*` clips cap at only |wz|<=0.20-0.25 rad/s while training/eval command turning up to 0.30 rad/s (`goal.walk_yaw_max_rad_s`) — `eval_yaw`'s `tip-left`/`tip-right` scenarios command exactly the 0.30 ceiling, right past the library's demonstrated range. The achieved rate is only ~25-30% of commanded (tip errs 0.22-0.26 against a 0.30 command), more undershoot than the 0.25-vs-0.30 library gap alone would predict, but consistent with an AMP discriminator that resists ANY motion pattern (turning fast, or possibly the foot-plant pattern behind the slip metric too) outside its narrow demonstrated range, independent of task-income pricing — which would explain why the just-closed dose grid moved nothing. `--amp-style-weight=0.0` fully disables `AMPStyleVecWrapper` construction (`train_ppo_mjx.py` L2043/2273) — a clean, existing-flag no-AMP ablation, the same control axis the original M2 wave-1 plan already used, not a new reward mechanism (no semantics-bank gate needed). Gate (pre-registered): CONFIRMS AMP-as-cause if tip errs move >=0.03 toward the 0.20 bar and/or walk slip improves >=0.15 toward 3.5, with 0/12 falls preserved (a fall regression would mean the style term was also load-bearing for the push-recovery fix, reopening that question too); REFUTES if tips/slip are unmoved or worse, pointing the next dig-in at gait-phase/stance-authority mechanisms instead. ~10-15 min training (2M steps); read its `eval_amp_m5` walk+yaw sections next cycle. Evidence for the library-cap finding: `rl_move/sim/motion_library/teacher_v2.npz` (`command` field per clip, `clip_names`).

Previous entry (08-23 ~11:0x, same cycle) — full `eval_amp_m5` cross-engine
re-read on this exact recalibrated checkpoint lands `m5_pass=false`,
a GENUINE (not free) trade, not the clean promotion the PASS above
implied:** push section improves (0 terms 12/12 gait_valid, was 1
det term) and **fault section now fully clears its own bar for the
FIRST time on this lineage** (gait_valid 12/12, was 9/12 with 2
sacrificed legs — this was the named blocker on the ORIGINAL M5
attempt). But walk and yaw both slip just past their own strict bars:
walk det_slip_med 3.67 vs bar <=3.5 (parent 3.36, clean); yaw
tip_left/right_err 0.2157/0.2351 vs bar <=0.20 (parent 0.162/0.184,
clean). Zero falls/terminations in either section (walk roll_peak
max 5.7deg, both contact sheets video-clean six-leg cycling, no
visible pathology) — this is a marginal tracking-quality miss against
a strict threshold, not a stability regression. Net read: push-force
recalibration fixes the two things that actually endanger the robot
(real falls, fault-carry gait validity) at a small cost to walk-slip/
yaw-tip margin that were themselves already the tightest bars in the
whole suite (tipfrac05 passed them by a hair). Do NOT call this
checkpoint an M5-candidate base yet — it is safer and fault-cleaner
than tipfrac05 but currently FAILS the full M5 gate on 2 of 4
sections. Options for the next cycle (not yet chosen): (a) a small
walk/yaw-side pricing nudge to recover the lost tracking margin
without reintroducing the push range that caused the falls; (b) treat
the marginal miss as within noise and re-seed/re-check (seed23's own
tip-tracking margin, once its m5 suite is read, will show whether
this is basin noise or systematic); (c) accept the trade and revisit
whether the assumed 0.20/3.5 bars (q_20260823T0130Z, never operator-
specified) should have a small tolerance band given the safety gain.
Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_m5/{walk,yaw.json,push,fault,m5_verdict.json}`.
**CONFIRMED SYSTEMATIC, NOT BASIN NOISE (08-23 ~11:2x, same cycle):**
ran the same `eval_amp_m5` suite on the `-seed23` twin —
**same shape, both signs**: walk det_slip_med 3.621 (vs bar 3.5,
seed7's own read 3.67), yaw tip_left/right_err 0.2493/0.2393 (vs bar
0.20, seed7's 0.2157/0.2351 — seed23 misses by MORE on tip_left),
push PASS (0 terms, gait_valid 12/12), fault PASS (gait_valid 11/12,
1 sacrificed leg, vs seed7's clean 12/12) — `m5_pass=false` again,
2/2 seeds now. Option (b) from the entry above is answered: this is
option (a)/(c) territory, not seed noise — the recalibrated push
range systematically buys push/fault-section safety at a systematic
walk-slip/yaw-tip cost across every seed tested so far. Next real
lever (not yet built/launched): either a small hold/forward-vs-turn
income nudge sized to recover ~0.02-0.05 of tip-tracking margin
without touching push force again, or accept the trade and ask
whether the strict 0.20/3.5 v1 bars should carry a small tolerance
band now that they're in direct tension with a safety fix (flag as
an amendment to `q_20260823T0130Z` for the next cycle to decide,
not decided here to avoid loosening a bar unilaterally mid-cycle).
Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_seed23_m5/m5_verdict.json`.

Previous entry (08-23 ~10:5x — SEED23 TWIN CONFIRMS: 2/2 recalibrated seeds now
clean on the full composition.** `tipfrac05-pushcal518-seed23`
VERDICTED PASS: raw `terminated` False on all 12/12 own-cfg DR-0
episodes (0/12 real falls), profile nearly identical to the seed7
parent (prog/slip medians within 0.1, same benign walk/sto/4
sacrificed-leg pattern); the same seed fell 2/12 at the old 10-25N
range. Item (1) from the ~10:4x entry below is now 2/3 closed —
`-seed13` (another cycle's pod) is the last twin needed before
promotion. Do not re-launch this grid. SKILLS.md row added.

Previous entry (08-23 ~10:4x — PUSH-FORCE RECALIBRATION FIX TRANSFERS TO THE FULL
COMPOSITION ON A FRESH RETRAIN: `tipfrac05-pushcal518` (single lever
vs `tipfrac05`, `dr.ext_push_n` 10-25N -> 5-18N, seed=7, 2M,
otherwise byte-identical) reads 0/12 real falls (raw `terminated`
field, both det+sto) vs the parent's own 2/12 — including a clean
pass on `walk/det/3`, the exact episode index where 5/6 of the
seed-safety batch toppled. Video-confirmed (contact sheet + per-
episode frame strips): clean upright six-leg cycling throughout, no
topple frame anywhere; one sto episode still sacrifices a leg
(fault-carry pattern, not a fall). direction_err/slip_per_m stay in
the parent's own range — no new regression traded for the fix. This
directly answers the ~09:5x-c hypothesis below: push magnitude (not
turn-in-place, not fault) was the root cause, and the fix generalizes
across composition tiers, not just at the fault+push ancestor level.
STILL OPEN before promotion: (1) seed-robustness — `-seed23`/
`-seed13` twins launched alongside this arm, unverdicted; (2) a fresh
`eval_amp_m5` cross-engine read on this exact (recalibrated)
checkpoint — the existing M5 PASS on record is for the un-
recalibrated `tipfrac05`. PASS verdict + SKILLS.md row recorded this
cycle; do not promote to M5-candidate/champion status until (1)-(2)
close. Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_gate/`.**

Previous entry (08-23 ~09:5x-c — ROOT CAUSE TRACED PAST THE SEED BATCH TO PUSH: the
uncounted fall risk pre-dates turn-in-place AND fault. Fault-only
`turnfault-seq1` = 0/12 real falls (clean); the moment push composes
in (`pushcont1`) = 4/12 real falls, never counted (same `gait_valid`
metric bug), and an M3-era push-only/fault-free checkpoint already
falls 2/12 at the SAME episode index. Do not chase turn-in-place or
kernel-EMA further until push-recovery itself is root-caused (push
magnitude vs timing-vs-gait-phase vs undertrained recovery, next
dig-in step) — see the top-of-file banner for full evidence.**

**08-23 ~09:5x-b — 7th/last seed (`-seed43`) closes the batch: 1/7
seeds genuinely fall-free (0 real falls), and even it nearly falls
on the SAME held-out episode (`walk/det/3`, roll_peak 17.6deg,
`roll_class=recovered`) that topples 5 of the other 6. Final tally:
seed7=2 falls, seed23=2, seed13=1(+3 sac), seed31=1, seed37=1,
seed41=3, seed43=0. Do not promote tipfrac05, do not fund more seed
arms — see the top-of-file banner and CURRENT_TRUTHS.md for the full
writeup; next step is root-causing/re-sampling the ONE maneuver
(frozen/weak-leg fault + turn-in-place), dig-in-tier. Full detail
below in the ~09:5x correction this closes out.**

**08-23 ~09:5x — CORRECTION: the "2 SAFE / 4 UNSAFE seed-lottery" call
below is WRONG; the real finding is a near-universal (5/6) fall on
ONE fixed deterministic eval episode, not per-basin luck.** Went back
into raw `report.json` per-episode `term_reason`/`roll_peak_deg`
(the 09:4x entry, like several before it, read only the `gait_valid`
scalar, which never zeroes on a TERM) for all 6 completed seeds.
Actual hazard-free own-cfg-DR-0 fall counts: seed7(tipfrac05)=2
(det/3 tilt_roll 39.3deg, det/5 tilt_roll 41.3deg), seed23(s2)=2
(det/3 tilt_roll 35.4deg, sto/5 tilt_pitch), seed13=1 (det/3 tilt_roll
36.3deg), seed31=1 (det/3 tilt_roll 38.4deg), seed37=1 (det/3
tilt_roll 32.3deg), seed41=3 (det/2 tilt_pitch, sto/0 tilt_pitch,
sto/5 tilt_roll -- notably NOT det/3). **Five of six fall at the SAME
deterministic episode index (`walk/det/3`), all `tilt_roll`, all in a
tight 32-41deg band** -- eval `--seed 0` is fixed and command draws
are deterministic, so episode index 3 is the identical commanded
trajectory across every checkpoint in the family; the seed7/s2/
seed31/seed37 frame strips all show clean walking for the whole strip
then an abrupt topple on the same last frame -- one shared failure
shape, not four independent random tips. Only `seed41` escapes det/3
(prog_ratio 1.172 there, no term) but is the single WORST checkpoint
overall (3 falls elsewhere) -- reads as "avoided the common failure
maneuver by accident," not "more robust." **REVISED CONCLUSION: this
is primarily a near-universal gap on one hard maneuver in the fixed
eval script (a training-exposure gap, buildable and testable), not a
training-seed safety lottery needing more seeds to characterize.**
Champion `tipfrac05` (seed7) is NOT actually zero-fall (2/12) --
that safety claim needs correcting wherever it was implied. Does NOT
reverse `-seed41`/`-seed37`'s own FAIL verdicts (video-evidenced, and
still the worst of the batch by raw fall count) -- it reframes the
blocker and the next step: replay `walk_det_3.mp4` across the family
to pin the exact maneuver (heading/speed transition timing near
episode end), then check its representation in `stress_mix` training
sampling. Flagged for a dig-in cycle (deep per-leg/timing analysis,
possibly a training-distribution fix) rather than more seed arms.
SKILLS.md not touched pending that re-read. Evidence: same
`..._{tipfrac05,s2,seed13,seed31,seed37,seed41}_gate/report.json` +
`walk_det_3.mp4` across all six.

Previous entry (08-23 ~09:4x — SEED-SAFETY BATCH CLOSES AT n=6: 2 SAFE / 4 UNSAFE
(~67%), now the CONFIRMED DOMINANT blocker on tipfrac05 promotion.
The seed31/37/41/43 batch was launched to pin the true unsafe rate
after seed13/s3 found 1/3 seeds unsafe at n=3. This cycle triaged
`-seed41` (UNSAFE: own-cfg DR-0 gate's gait_valid metric nominally
reads 6/6+6/6 -- a harness quirk, it never zeroes on a TERM -- but the
raw report + frame strips show THREE separate video-confirmed
topples: walk/det/2 tilt_pitch, walk/sto/0 tilt_pitch, walk/sto/5
tilt_roll, worse than any prior seed and with no sacrificed-leg
softening this time) and picked up the orphaned `-seed37` by hand
(its prestage never fired; podeval run directly on its own pod:
UNSAFE, 1 video-confirmed fall, walk/det/3 tilt_roll). Read-only peek
at the concurrent cycle's `-seed31` (not verdicted here) shows the
same 1-fall pattern (det/3 tilt_roll). Combined tally across 6
completed seeds (7, 23, 13, 31, 37, 41): SAFE = {7, 23}, UNSAFE =
{13, 31, 37, 41} -- **2/6, not the 2/3 the n=3 read suggested.**
`-seed43` (4th batch arm) still training, owned by a concurrent
cycle. **CONCLUSION: most seeds sampled from this exact recipe (2M,
turn-in-place curriculum on the composed turn+push+fault stack,
seed=7's own hyperparameters) fall on a hazard-FREE walk eval.
tipfrac05 (seed7) remains a real PASS on its own gate, but the RECIPE
itself is not seed-robust for safety; nothing past the single seed=7
checkpoint should move toward M5-candidate status until this is
root-caused.** This re-orders the two named M5-candidate
prerequisites below: seed-safety root-cause is now MORE urgent than
the hold/forward income-repricing/budget-stability question (both
remain open; neither is funded with a build yet -- next dig-in cycle
should prioritize seed-safety). No SKILLS.md update (both FAIL
verdicts). Evidence: `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_
pushcont1_tipfrac05_{seed31,seed37,seed41}_gate/`.

Previous entry (08-23 ~09:1x — KERNEL-EMA QUESTION CLOSED AT n=5: 0/5 arms rescue
yaw-tracking in EITHER regime, and the fixed-basin continuation test
CONFIRMS a real transition-handling defect, not fresh-retrain basin
noise. Triaged the two continuation arms the 08:4x entry below
launched to discriminate the two live hypotheses. Both FAIL, both on
their own pre-registered non-rescue branches: `-kernelema-cont1`
(yaw+vel EMA, +6M from the tipfrac05 checkpoint itself) tips
0.2385/0.3168 — worse than the NON-EMA `-acq1` continuation's own
erosion (0.2038/0.2692) on both signs, safety floor unchanged
(gait_valid 11/12, 1 sacrificed leg, 0 falls, same as acq1).
`-acq-kernelema` (identical lever, respec of `-acq1` directly) tips
0.2659/0.2901 — worse still, AND adds a genuine safety regression:
own-cfg DR-0 gate shows 3 termination events across 12 episodes
(video-confirmed topples, tilt_roll x2 + tilt_pitch x1) plus 1
sacrificed leg, vs tipfrac05's and acq1's own zero falls. Both arms'
isolated `eval_amp_m5` walk/push/fault sections PASS and are each a
touch better than acq1's own numbers (lower slip, higher progress,
fewer sacrificed legs) — kernel-EMA is not globally harmful, it
specifically fails (and here actively worsens) the one axis it
targets. Because BOTH continuations regress on the SAME fixed basin
that acq1 already occupies, this is the exact discriminating test the
08:3x REFINED MECHANISM note asked for: pure basin-selection noise
predicted the continuation would be unaffected; a real
transition-handling defect predicted it would still regress. It still
regressed — CONFIRMED real defect, not noise. Root cause (diagnosed,
still unbuilt): the EMA'd kernels never reset on a
`goal.vx_ref`/`wz_ref` command change, so training under them teaches
a "damp body dynamics for ~tau after any translation-heavy segment
ends" habit that costs nothing during ordinary stress_mix resampling
but directly suppresses the abrupt heading-authority `eval_amp_m5`'s
tip-left/right segments (timed right after arc-max turns) measure.
**CONCLUSION: do not fund any further naive kernel-EMA arm on this
lineage.** The two named prerequisites for M5-candidate promotion
past the 2M tipfrac05 checkpoint — (1) budget-stability (still FAILED:
acq1 and both kernelema continuations all erode tips past the 2M
parent) and (2) seed-safety variance (1/3 seeds unsafe at n=3) —
remain OPEN. Next concrete build, when a dig-in cycle takes it: the
targeted command-transition-aware EMA reset (snap `_walk_kernel_
vema`/`_wz_ema` to the instantaneous value whenever the commanded
vx/vy/wz changes beyond a small epsilon — touches CPU env + MJX shard
+ MJX batched reset sites + `MJX_SNAPSHOT_EXTRA` + new regression
tests) OR skip straight to a structural hold/forward income-repricing
lever that doesn't rely on kernel smoothing at all. SKILLS.md not
amended (no new bar cleared, this is a clean double refutation).
Evidence: `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_
tipfrac05_{acq_kernelema,kernelema_cont1}_{gate,m5}/`. Prior banner
below.)

Previous entry (08-23 ~08:4x — LAUNCHED the funded next step: kernel-EMA as a
CONTINUATION (fixed basin), not another fresh retrain. The
3-arm decomposition grid (kernelema1/-yawonly/-velonly2, all
FAIL/INFORMATIVE — see entries below) closed without attributing the
mechanism to either axis because a FRESH 2M retrain's basin-selection
noise (tipfrac05's own seed grid: 0.207-0.234) swamps the lever's
effect size. That grid never actually tested the question the fix was
built for (q_20260823T0240Z item b: does repricing rescue BUDGET-
CONTINUATION, i.e. the `-acq1` +6M erosion 0.162/0.184 -> 0.204/0.269
while reward rose then plateaued). Launched
`cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-acq-kernelema`
(train-0, VERIFIED RUNNING): respec of `-acq1` (+6M warm-started from
the tipfrac05 checkpoint ITSELF — same fixed basin, same seed=7),
single added lever = both `reward.walk_kernel_yaw_ema` +
`walk_kernel_vel_ema` on (tau=0.75s). Pre-registered gate: tips
staying <=0.25 both signs without eroding past the 2M parent's own
seed-noise band = repricing rescues budget-continuation (promotes to
a standing default for future AMP turn continuations); erosion
matching acq1's despite the fix = repricing is real-but-insufficient
against the untouched actuation-cost asymmetry (current/gyro/roll
4-10x pricier on real motion), escalating to that harder lever next.
Prior banner below.)

Previous entry (08-23 ~08:3x — DECOMPOSITION ARM #2: `-kernelema-velonly2`
(translation-EMA ONLY, yaw kernel untouched) reproduces ~90% of the
bundle's yaw regression — the original gate-conflict hypothesis is
REFUTED.** VERDICTED FAIL. `eval_amp_m5` tip-left/right err
0.2064/0.2286 (parent tipfrac05 bar-clean 0.1620/0.1838; bundled
`kernelema1` 0.2264/0.2302) — over the 0.20 bar despite this arm
never touching `walk_kernel_yaw_ema` or any yaw-side reward term.
Since the yaw kernel is provably unchanged here, the bundle's
originally-suspected mechanism (an EMA'd yaw kernel fighting the
still-raw `walk_yaw_kernel_gate`/`walk_yaw_hold_prog_gate` achieved-
rotation gates) CANNOT explain most of the regression — velonly2 gets
there almost entirely from the translation-kernel EMA alone. New
leading suspect: the EMA'd velocity-error state carries stale
pre-transition error across the walk<->turn-in-place mode switch
(commanded translation speed drops to ~0 mid-episode when a
turn-in-place segment starts; if the EMA doesn't reset at that
boundary, the turn segment inherits noise from the preceding walk
segment). Walk/push/fault sections all PASS on this arm — fault
gait_valid actually IMPROVED to 10/12 (parent missed its own bar at
9/12) — so this is an isolated tracking-quality cost, not a stability
one; video-clean six-leg cycling on both walk and fault contact
sheets, zero falls. **UPDATE (same review window): the third arm,
`-kernelema-yawonly` (yaw-EMA only), has now landed — a concurrent
cycle read it at tips 0.2285/0.2053, the SAME ~0.21-0.23 band as
kernelema1 and velonly2 despite never touching the velocity kernel.**
All three arms regress together regardless of which axis got the EMA
— confirming the general "not axis-specific" branch predicted above.
The concurrent cycle's own read attributes this to fresh-2M-retrain
basin-selection noise (citing that the band overlaps the tipfrac05
lineage's own seed-variance spread) and proposes a CONTINUATION arm
on the tipfrac05 checkpoint itself (fixed basin) as the next
discriminating test — see the top-of-file banner for that verdict's
full text. The mechanism below is a SECOND, non-exclusive candidate
explanation the same continuation arm would also help distinguish
(a real transition-handling defect predicts the continuation still
regresses on the SAME fixed basin; pure basin noise predicts it
doesn't) — recorded here so whichever cycle runs that continuation
checks both.

**REFINED MECHANISM (same cycle, code read + `probe_walk_income`
re-read, correcting the transition-boundary guess above):**
`goal.walk_turn_in_place_frac` makes the WHOLE episode a dedicated
turn from reset (`walk_task.py` ~2066-2078, zero linear command the
entire episode) — there is no walk-then-turn transition inside those
training episodes, so a per-episode EMA-reset-at-turn-onset fix would
not even fire there. The real link is in `eval_amp_m5`'s OWN yaw-
section scenario chain (`yaw.json`'s `scenarios` dict): one
continuous rollout runs `fwd-hold -> stop-hold -> arc-left ->
arc-right -> arc-left-max -> arc-right-max -> tip-left -> tip-right
-> yaw-flip-*` in strict sequence — tip-left/right are graded
IMMEDIATELY after two translating+turning arc-max segments, every
single time, for every checkpoint. `walk_kernel_vel_ema`/
`_yaw_ema`'s own code comment confirms the update is UNCONDITIONAL
and never resets on a command change ("cannot be gamed by timing a
segment boundary") — by design, for TRAINING's frequent stress_mix
resamples. The theory: because ordinary training (stress_mix
resampling every ~4s +/- jitter) constantly serves the EMA'd kernels
transition tails identical in kind to arc-max -> tip-turn, a policy
trained under the EMA'd kernel learns a general "damp overall body
dynamics for ~tau after any translation-heavy segment ends" habit to
avoid the smoothed kernel's lagging penalty — a habit that costs
nothing during eval's own hold/arc segments (which don't need a fast
NEW rotation right at the segment boundary) but directly suppresses
exactly the abrupt heading-authority the tip-left/right segments are
timed to measure. This predicts the SAME regression direction for
ANY axis's kernel-EMA (translation or yaw) and requires no
turn-in-place-specific code path, consistent with velonly2 alone
reproducing most of the bundle's damage. NOT YET BUILT/TESTED: a
targeted fix (e.g. snap `_walk_kernel_vema`/`_walk_kernel_wz_ema` to
the fresh instantaneous value whenever `goal.vx_ref`/`vy_ref`/`wz_ref`
changes tick-to-tick beyond a small epsilon, so the kernel forgives a
command change immediately instead of lagging by `tau`) touches 3 env
variants (CPU env + MJX shard + MJX batched, each with its own reset
site per `grep _walk_kernel_vema`) plus `MJX_SNAPSHOT_EXTRA` state and
new regression tests — real dig-in-cycle scope, not built blind this
cycle. Do not stack either kernelema arm onto tipfrac05 meanwhile; the
hold/forward income-repricing prerequisite for M5-candidate promotion
remains OPEN. Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_kernelema_velonly2_{gate,m5}/`,
`logs/probe_walk_income/hold_forward_income_ypfix1.json` (re-read:
the hold(1415)/forward(797) gap is dominated by the vel/yaw KERNEL
sway-tax terms themselves, -401/-171, not by current/gyro/roll
actuation cost, -43 combined — the EMA mechanism was aimed at the
right term, its transition-handling is the bug).
Prior banner below.

Previous entry (08-23 ~08:2x — KERNEL-NOISE-TAX FIX (kernelema1) MADE TIP-TRACKING
WORSE, not better — the run's own pre-registered WORSE branch fired.)
`cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-kernelema1` (single
lever vs tipfrac05: `reward.walk_kernel_vel_ema=1` +
`reward.walk_kernel_yaw_ema=1`, tau=0.75s, bundled — de-noising the
velocity/yaw-rate tracking kernels so honest stride-to-stride sway
isn't taxed as mistracking, the hypothesized fix for the measured
hold/forward income-dominance gap) VERDICTED FAIL. `eval_amp_m5` tip-
left/right err WORSENED to 0.2264/0.2302 (parent tipfrac05's bar-clean
0.1620/0.1838) — both now OVER the 0.20 bar (parent passed clean
under it), landing back in the tipfrac02/07 in-band-not-clean tier;
yaw section flips PASS->FAIL. Walk section also crossed its own bar
(det_slip_med 3.79 vs bar 3.5, parent 3.36 just under) though
gait_valid held 12/12 with 0 terms — video-clean six-leg cycling
(contact sheet reviewed), so this is a tracking-quality regression,
not a stability one. Push (pass, det_terms 2=bar) and fault (pass,
gait_valid 10/12=bar, same 2 carried-fault legs as parent) held at/
near the bar; m5_pass=false overall. Root cause suspected exactly as
the gate text pre-registered: an unanticipated interaction with the
already-on achieved-rotation gates (`walk_yaw_kernel_gate`/
`walk_yaw_hold_prog_gate`), which still read RAW instantaneous wz
while the EMA'd kernel now rewards smoother-but-slower yaw-rate
tracking than command — the policy under-rotates more, not less.
Not a safety failure (0 falls, gait_valid floors held everywhere).
**CONCLUSION: kernel-noise-tax de-noising, applied bundled and
naively, is REFUTED as a drop-in fix for this checkpoint's
income-dominance gap** — the mechanism from the joystick track
(phasedir7/7b/8) does not transplant cleanly onto a reward that still
prices ACHIEVED rotation on the raw instantaneous signal. Decomposition
arms `-kernelema-yawonly` (yaw EMA alone) and `-kernelema-velonly2`
(vel EMA alone, respec after the first `-kernelema-velonly` was
REFUSED for a busy GPU slot) are in flight/finishing to attribute
which axis (or their interaction) drives the regression — owned by
whichever cycle triages them next, not duplicated here. Until
attributed, do not spend further budget on the bundled recipe; the
hold/forward income-repricing prerequisite for M5-candidate promotion
remains OPEN and is now more clearly a per-tick pricing problem than
a kernel-noise one. SKILLS.md M4 turn-erosion row amended. Evidence:
`logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_kernelema1_{gate,m5}/`.
Prior banner below.

**08-23 ~07:5x — TURN-EXPOSURE DOSE GRID CLOSED at 4 points (0.2/0.3/
0.5/0.7): 0.5 is the confirmed peak, not the largest dose tried.**
`cw-amp-m4-turnfault-seq1-pushcont1-tipfrac07` (dose 0.7, the grid's
last untried rung) VERDICTED INFORMATIVE — the pre-registered
TURNOVER branch fired cleanly: yaw section tips regressed to
0.236/0.225 (worse than tipfrac05's bar-clean 0.162/0.184, back into
the tipfrac02/03 in-band-not-clean range) — MORE dedicated
turn-episode exposure made tip-tracking WORSE past 0.5, not better.
Walk section also erodes: `n_translating` collapsed to 2/12 (0 of 6
det episodes were translating at this dose — a real harness-sampling
gap for future high-turnfrac arms, not just a training defect — so
`det_prog_med`/`det_slip_med` are null/unjudgeable) and the 2
translating episodes that did land both miss the slip bar (4.03/4.30
> 3.5). Own-cfg DR-0 gate regresses too: gait_valid 10/12 with TWO
video-confirmed tilt_roll falls (roll peak 35.0/34.6deg) plus 2
sacrificed-leg sto episodes — vs tipfrac05's clean 12/12/zero-falls —
a dose-driven safety cost independent of the `-s3` seed-lottery
finding (same seed=7 as the champion here). Fault section is the one
axis that holds (10/12 gv, meets bar, slightly better than
tipfrac05's post-fix 9/12) — not enough to offset yaw+walk+safety.
**Grid CLOSED, champion UNCHANGED: `tipfrac05` (seed7, turn_in_
place_frac=0.5) remains the sole M5-candidate; no further dose arms
on this lever.** The two already-flagged prerequisites for actual
M5-candidate promotion — hold/forward income repricing (budget-
stability) and seed-safety-variance root-cause — are unaffected by
this result and remain the funded path (in progress on a concurrent
DIG-IN cycle at review time; left untouched, no duplicate work).
Evidence: `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_
tipfrac07_{gate,m5}/`. Prior banner below.

Previous entry (08-23 ~00:2x — CORRECTION to this section's own prior
entry: `cw-amp-m2-turnclone-yawcmd-tip50` (and `-tip90`) are **PASS**, not
INFORMATIVE — a triage error in the entry below (now struck through
in substance, kept for the audit trail) misapplied `eval_yaw.py`'s
generic CLI-default turn-gate (0.10, a leftover from an older,
stricter lineage) instead of THIS run's own pre-registered ledger
gate (tip-left AND tip-right err <=0.20, correct sign, zero falls,
translation preserved) written into its own `gate` field at launch
time. Measured tip-left/right err 0.1601/0.1346 (tip50) and
0.133/0.174 (tip90) both clear <=0.20 decisively; SKILLS.md already
carries the correct PASS row (search "BC-TURN-CLONE BREAKS THE
TIP-PARK WALL"). The behavioral substance of the struck entry is
still true and useful (RL fine-tuning eroded, not sharpened, the raw
clone's own turning accuracy: 0.10 raw -> 0.13-0.17 after 2M RL) —
only the PASS/FAIL letter grade was wrong. Lesson for future triage:
always quote the RUN'S OWN `gate` ledger field verbatim, never a
tool's CLI default, when the two differ.

Given the corrected PASS baseline, the same-cycle repricing follow-up
below reads even more sharply: `cw-amp-m2-turnclone-yawcmd-tip50-
yawprice3x` (single lever, `k_walk_yaw`/`k_yaw_prog` 1.0->3.0 from
the NOW-PASSING tip50 checkpoint) took a PASSING arm and made it FAIL
its own inherited gate — tip-left/right err rose to 0.2018/0.2642
(was 0.1601/0.1346), both now over the 0.20 bar, while training
reward more than doubled (240->564/quarter) with both yaw reward
terms pegged near their kernel ceilings — pure reward-farming on a
larger, still-saturated kernel, not real tracking improvement (an
08-21-style misalignment). Translation/gait fully unaffected (same
termination counts, walk_speed/height_err/direction_valid unchanged),
so this is a clean isolated regression, not a destabilization
confound. CONCLUSION UNCHANGED: yaw income MAGNITUDE is refuted as a
useful lever on this recipe (it actively hurts) — do not spend more
budget on yaw-pricing-dose sweeps, and do NOT ship the yawprice3x
checkpoint (it is strictly worse than tip50 on the very axis it was
meant to improve). The `cw-amp-m2-turnclone-yawcmd0` zero-dose control
LAUNCH_CRASHED (stale `--obs-pad-transplant 1` from the pre-clone
parent chain vs the already-75-dim clone checkpoint) — corrected
respecs (`-r2`, then `-acq1-r2`) already relaunched by a concurrent
cycle. With tip50/tip90 both PASS and the pricing lever refuted, the
M2-turn milestone's remaining open item is STRUCTURAL if further
polish is wanted (a training-time mirror-symmetry regularizer,
distinct from the joystick track's existing inference-time
`mirror.MirrorPolicy` reflection trick in `probe_mirror_turn.py`
which steers by selecting naked-vs-reflected rollouts post-hoc rather
than training symmetry in) — genuinely unbuilt, a cycle-sized code
task, not queued blind. The init question (can BC ever teach turning)
is answered YES and the turn-in-place gate itself is now CLOSED.

<details><summary>Superseded entry (wrong PASS/FAIL letter grade, kept for audit)</summary>

08-23 ~00:1x — BC-TURN-CLONE CONFIRMED REAL BUT PRICING-LIMITED:
`cw-amp-m2-turnclone-yawcmd-tip50` VERDICTED INFORMATIVE (WRONG —
see correction above, this was actually PASS). The turn-clone init
(TripodGait's native omega channel demonstrated for the first time)
genuinely closes the motor-pattern discovery gap that made every
prior yawcmd/tip arm park (err == |wz_ref| exactly): eval_yaw on this
checkpoint reads turn |wz_err| med 0.140 vs the old park fingerprint's
0.28-0.33 — real, command-signed rotation at roughly half the
commanded magnitude, not a frozen statue. The RAW untrained clone
itself already scored 0.1035 before any RL, so 2M steps of RL under
the unchanged tip50 pricing recipe made yaw tracking WORSE (0.10->
0.14) while training reward rose 57->248/quarter. Translation fully
intact (gait_valid 12/12 det+sto, 0 falls, prog/slip in the usual
yawcmd band).

</details>

Previous entry (08-22 ~20:0x, **FAULT INJECTION BUILT + TESTED** (M0
checklist "fault injection works"; M4 prerequisite; last
confirmed-not-started M0 item besides the push curriculum's
random-direction extension):
`dr.fault_*` per-joint fault injection per brief §8: with
`dr.fault_prob` (default 0.0 = OFF, guarded draw, legacy rng stream
bit-exact) an episode carries ONE fault drawn from `dr.fault_mix` —
weakened joint (`fault_weak_scales` 0.7/0.4/0.2/0.0 strength on
kp+torque; 0.0 = dead servo free-swinging on its kv backdrive),
frozen joint (actuator force zeroed + `FROZEN_DOF_DAMPING=500`
viscous lock — measured creep 0.05 rad/2 s under a worst-case
drop-settle load, stable), or disabled leg (3 joints dead).
Implementation is PURE MjModel field edits applied after
`apply_params_to_model` at all three DR sites (sim_env private
model, `ModelDrScratch.rows_for` for both batched/sharded MJX vec
envs, compare_standup probe); every touched field
(actuator_gainprm/biasprm/forcerange, dof_damping) is already in
`MODEL_DR_FIELDS`, so the per-world model-DR upload carries faults
to the GPU stacks with ZERO stepper changes. §8.2 health vector
available as `EpisodeRandomization.fault_health()` (18-dim, 1
healthy / strength / 0 dead-frozen) — actor-obs wiring is M4 work,
not done on purpose. Bank: `rl_move/tests/test_fault_injection.py`
10/10 (default-off no-op + rng-stream guard, per-row edit scoping,
mode/joint sampling validity, scaled() prob-not-dose convention,
health vector, CPU env end-to-end with `--cfg-set dr.fault_prob=1`);
regression `test_sim_env.py`+`test_physics_ease.py` 52/52,
`test_struct_compliance.py` 6/6. First training use should ride a
2M smoke arm per new-mechanism discipline. Eval-side fault cases
(M4/M5 gates) need no new harness: `eval_checkpoint.py --cfg
dr.fault_prob=...` reaches it through the existing dr.* override
loop.

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
