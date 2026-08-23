# amp - AMP locomotion from scratch

Last updated: 2026-08-23 ~01:4x (**M3 PUSH CURRICULUM STAGE 1: BOTH
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

**08-23 ~00:2x — CORRECTION to this section's own prior entry:**
`cw-amp-m2-turnclone-yawcmd-tip50` (and `-tip90`) are **PASS**, not
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
