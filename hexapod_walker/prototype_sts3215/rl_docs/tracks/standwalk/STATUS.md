# standwalk — mesh-model stance retrain, then distill into walking

Last updated: 2026-08-26 ~18:0x (**ANCHOR-LEAK ROOT CAUSE FOUND, FIX
LANDED + TESTED, REPAIRED PAIR RUNNING.** The pre-registered
routing/gradient-isolation dig-in on `-anchor1`/`-anchor1-s1` closed
with a mechanical answer: the stance-only anchor never leaked
GRADIENT into the walk core — it leaked **shared-Adam MOMENTUM**. On
a stance-only aux minibatch the dual-core gate multiplies the walk
path by exactly 0.0, but autograd still POPULATES all-zero `.grad`
tensors on core A's GRU + actor head, and `bc_anchor.BCAnchorPPO.
train()`'s `policy.optimizer.step()` (the shared Adam) then applies a
pure stale-momentum update to those params anyway — 8 extra
uncommanded steps per PPO update, every update. Empirical probe: one
momentum-priming step + 8 stance-only aux minibatches moved the walk
actor path by total |dParam| 0.815 with total |grad| exactly 0. The
existing `test_dual_gradient_isolation` passes because it asserts
grads are zero-or-None — it pins GRADIENT isolation, not UPDATE
isolation. This also explains the dual1-vs-anchor1 discrepancy:
`cw-arch-gru-dual1`'s walk core was near-converged (tiny PPO grads →
tiny momentum, walk=0.60 mix kept correcting) so the same leak was
benign; anchor1's walk head was mid-collapse on mesh (reward trough
−344/−456 → large momentum) at walk=0.30, so the leak became a
dominant seed-dependent random walk — matching the two DIFFERENT
catastrophes (seed0 freeze vs seed1 shuffle: optimizer noise, not a
reward incentive). Also corrected the "identical recipe" claim:
anchor1 was NOT dual1's recipe (coef 3.0 vs 1.0, + foot_z /
flat_time_indexed / min_h_ahead, mesh/100 Hz, mode_seq=0.75).
**Fix landed (commit `2f585a97`, tag `exp/bcanchor-isolate-update`):
`train.bc_anchor_isolate_update=1` drops populated all-zero grads
(sets `.grad=None`) before the aux optimizer step so Adam skips
untouched params entirely; default 0 = legacy bit-exact. Two new
tests pin the defect AND the fix (`test_dual_anchor_aux_step_leaks_
momentum_into_gated_out_core`, `test_dual_anchor_isolate_update_
protects_gated_out_core`); full `test_bc_anchor.py` (70) +
`test_gru_policy.py` (27) green. LAUNCHED + VERIFIED RUNNING:
`cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2`/`-anchor2-s1`
(train-0/1, 2M canary each), the anchor1 recipe with exactly ONE
change (isolate_update=1). Joint gate: LEAK-FIX PASS if walk shows no
anchor1-class catastrophe on both seeds (prog back in modeseq1's
0.19–0.38 band or better); FULL PASS additionally needs hold+lower
isolated (<=1/6) det+sto; FAIL-A (walk still wrecked) escalates to
the PPO-side twin of the same channel (single-family recurrent
minibatches under mode_seq=0.75 zero-grad momentum + value mixing);
FAIL-B (walk fixed, stance still majority-fail sto) moves the lever
to stance teacher/dose. NOTE for future audits: the SAME zero-grad
momentum channel exists inside PPO's own update whenever a recurrent
minibatch happens to be single-family — inherent to dual-core +
shared Adam since dual1; unfunded until anchor2 reads back.
Prior banner below.)

Previous entry, 2026-08-26 ~16:4x (**Stage-2 FALLBACK CELL ALSO CLOSED
FAIL: `cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1`(seed0) /
`-anchor1-s1`(seed1), the `cw-arch-gru-dual1`-proven stance-only/
walk-off `bc_anchor` fallback (coef=3.0, `bc_anchor_walk=0.0`), BOTH
CANARY FAIL - MECHANISM, cross-seed replicated, real per-episode
report.json, DR-0 + own-DR(0.5), det+sto (4 full reads).** Plain
English: pulling PPO's stance-tick gradients back toward the proven
mesh teacher does NOT rescue hold/lower to isolated failure, AND it
additionally wrecks walk far worse than the bare fine-tune it was
meant to fix — the fallback is a bigger regression than the problem.
**hold**: sto success is **0/6 in all four reads** (both seeds, both
DR) — the parent's exact 6/6-TERM signature survives completely
unchanged on the stochastic axis; det partially recovers (5/6 seed0,
4/6 seed1 at DR-0) but is DR-sensitive, collapsing to 3/6 and 2/6 at
own-DR — never isolated on both det+sto together, either seed.
**lower**: best single cell is 5/6 (seed1, DR-0, sto) but its own
det companion the same seed same DR is only 4/6, and own-DR drags
both back to 2-3/6 — the gate needs BOTH cells isolated (<=1/6 fail)
simultaneously; never achieved anywhere. Video: stuck mid-crouch,
worst_clear 34-52mm, never completes the plant. **walk: 0/6 success
in ALL 8 walk cells (2 seeds x 2 DR x det/sto)** — catastrophically
worse than the bare fine-tune's own weak-but-positive crawl
(prog_ratio 0.19-0.38 on modeseq1) — and the two seeds fail in TWO
DIFFERENT new ways: seed0 is a near-total leg-sacrifice freeze
(`sacrificed_legs` 5 of 6, prog_ratio -0.004..+0.06); seed1 is a
high-slip shuffle-in-place (`gait_valid` nominally True/legs cycling,
but prog_ratio NEGATIVE, slip/m 44-49, ~15x the 2.9 joystick cap).
Both contact sheets (walk_det_0, both seeds) show a static splayed
stance with ZERO visible translation across the full 30s strip.
Reward both seeds: healthy BC-init Q1 (+37/+39) -> deep Q3 trough
(-344/-456) -> still deeply negative Q4 (-112/-122, never recovers) —
aligned bad-and-stuck per the 08-21 ruling, not a rising-reward
continuation case. **This fires the gate's own pre-registered
escalation exactly, and in a stronger form than anticipated**: not
only does hold/lower stay majority-fail at the parent's own
signature, the anchor also destabilizes the walk head it was
explicitly built to leave unconstrained — evidence the dual-core
routing leaks the massive (coef=3.0) stance-anchor gradient into
walk/shared-value-function territory, not just failing to reach
hold/lower. **Per the gate's own text: do NOT fund a third
stance-anchor dose/config.** Next owner: a routing/gradient-isolation
dig-in reading `bc_anchor.py`'s dual-core wiring + the per-tick
gradient-masking tests to find where stance-tick gradients bleed into
walk-tick parameters, before any further arm on this lever.
**Also corrected this cycle**: `-anchor1-s1`'s original attempt had
been mis-recorded `CANARY FAIL - INFRASTRUCTURE`/KILLED by an earlier
pass of this same cycle acting on a stale "deadlock" read; fresh W&B
evidence (state=finished, global_step 2031616, full 787-row history,
real exported checkpoint byte-identical in size to seed0's) shows it
actually completed cleanly — the same false-positive shape already
seen on `modeseq1-s1r`. The infra-retry it spawned (`-anchor1-s1-r1`)
was killed this cycle at ~1.05M/1M-target steps (superseded, no skill
data, negligible compute lost); this verdict uses the ORIGINAL
checkpoint's real reads. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_stage2_dualbc1_anchor1{,_s1}_{gate,
owncfg}/`, W&B `sqprmus4`/`g6hghec7`. Prior banner below.)

Previous entry, 2026-08-26 ~14:2x (**Stage-2 first-cell JOINT CALL
CLOSED: `cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1`(seed0) /
`-modeseq1-s1`(seed1) both CANARY FAIL - MECHANISM, cross-seed
replicated with real per-episode report.json data (not just cached
W&B end-state). The bare dual-core RL fine-tune from the dual-teacher
BC init regresses hold and lower to majority failure while walk stays
a weak crawl — exactly the runs' own pre-registered FAIL branch.
Evidence, both seeds, DR-0 + own-DR(0.5), det+sto: **hold TERMINATES
in 24/24 episodes each seed** (hold_min_load / hold_low_height; the
hold contact sheets show the robot visibly sinking from standing into
a low splayed crouch over the episode, `settled` 0/6 every read, never
recovered from BC-parent's clean hold). **lower is 0/6 success
everywhere** (worst_clear 68-152mm, 1-4 over_current terms per read).
**walk never falls** (roll settled, gait_valid 5-6/6 = no sacrificed
legs) but is a near-static splayed stance on video (walk_det contact
sheets, both seeds — no visible translation across the 30s strip),
prog_ratio only 0.24-0.38 det / ~0.00-0.02 sto (vs the BC-parent's own
~2600-2700 walk return), dir_err 47-91deg (cap ~40deg), slip/m 3-34.
Reward: both seeds collapse from a healthy BC-init start (+37/+40 Q1)
through a deep trough (-502/-535 Q3) to a still-deeply-negative Q4
(-174/-202) — aligned bad-and-stuck per the 08-21 ruling, not
"still rising, needs more budget." **CORRECTION**: `-modeseq1-s1`'s
FIRST attempt was mis-recorded `FAILED`/"CANARY FAIL - INFRASTRUCTURE"
by a checkup false positive (SUSPECT fired on a slow periodic-eval/
video-render round, not a real hang) — its train log shows it actually
completed all 2,031,616 steps cleanly with a real exported checkpoint;
this call uses that checkpoint's real eval data, which matches seed0's
signature exactly, and supersedes the infra-failure framing. The
identical-seed retry `-modeseq1-s1r` that the false positive triggered
(separately triaged by a concurrent cycle, same FAIL signature) is a
redundant duplicate, not additional evidence. **Routed to the
pre-registered fallback**: launched `cw-standwalk-stance-mesh2-
stage2-dualbc1-anchor1`/`-anchor1-s1` (2M canary pair, both VERIFIED
RUNNING) — identical recipe + the `cw-arch-gru-dual1`-proven
stance-only/walk-off `train.bc_anchor_*` bundle (coef=3.0,
state_aligned/stratified/foot_z/flat_time_indexed/lower=1.0,
min_h_ahead_mm=8, **bc_anchor_walk=0.0**) so PPO gradients on stance
ticks get pulled back toward the proven mesh stance teacher while walk
ticks train unconstrained through their own dedicated dual-core head.
PASS if hold/lower termination collapses to isolated (<=1/6) on BOTH
seeds with walk holding/improving its current weak translation; FAIL
at the same majority-term signature escalates to a routing/gradient-
isolation dig-in (the dual-core architecture itself leaking
stance-destructive gradients), not a third anchor dose. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_stage2_dualbc1_
{modeseq1,modeseq1_s1}_{gate,owncfg}/`, W&B `zygtbdyy`/`v38ba434`.)

Prior entry, 2026-08-26 ~14:1x (**Stage-2 first-cell canary, seed-1
read posted: `-modeseq1-s1r` = CANARY FAIL - MECHANISM own-scope. The
bare PPO fine-tune ERASED all three dual-BC skills by 2M — walk
park-creep 0.44m/30s det / frozen-stance sto, hold 6/6 min-load
splay-collapse, rise/lower majority over_current, at BOTH DR-0 and
own-DR 0.5; reward trough -771 then plateau -185, far below the BC
init's +61. Every pre-registered FAIL clause fires. The infra retry
itself SUCCEEDED (no recurrence of `-s1`'s 1M-boundary eval deadlock —
sporadic, not reproducible). JOINT CALL + the pre-registered
`train.bc_anchor` (stance-only, walk-off) fallback pend the seed-0
twin, whose triage is owned by a concurrent cycle; its cached W&B
end-state shows the same signature (ep_rew -197, canaries 0).
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_stage2_dualbc1_
modeseq1_s1r_{gate,owncfg}/`, W&B `gmtsnhem`.**)

Prior entry: 2026-08-26 ~10:2x (**STAGE-2 DE-RISKED: the primitive
walk teacher (`stotight45-seed13`, 25 Hz) transfers to mesh dynamics +
the real 100 Hz motor contract almost for free, and composes with the
mesh stance teacher (`acq8m`) via `goal.mode_seq` at only a 10 % fall
rate concentrated entirely in the ALREADY-TRACKED rise/hold residual
— zero new walk-composition pathology. Dual-teacher BC distillation
launched (background, CPU) as the first real Stage-2 mechanism arm.**
Plain English: everyone assumed unifying rise/lower (mesh, 100 Hz)
with walking (still only proven on the OLD lighter/25 Hz robot) would
need a whole rate-conversion engineering effort before it could even
be tested. It didn't — three cheap measurement probes (no training,
pure inference, `eval_joystick_gate`/`verify_modeseq_teachers`, this
cycle) answer the Binding-Constraints "measure before trusting"
requirement directly:
1. `stotight45-seed13` run AS-IS on `env.model_source=mesh` at its
   OWN native `control.hz=25` (isolating the mass/geometry variable
   alone): joystick DONE-gate n=24 DR-0, PASS (0 falls, slip/m 2.567,
   dir_err 31.4deg, gait_valid 1.0) — the +66% mass / shifted hip axis
   does NOT break this gait.
2. Same checkpoint at `control.hz=100` under the CURRENT real mesh
   motor contract (`safety.max_delta_q_deg=0.375` default, not this
   run's own trained 5.0 deg/tick@25Hz — i.e. the actual contract any
   unified stage-2 policy must use), DR-0, n=24, full 60s episodes:
   PASS (0 falls, slip/m 2.426, dir_err 39.84deg — just inside the
   40deg cap, gait_valid 1.0). Adding own-DR(0.35) tips direction
   error just over the cap (combined med 40.72 vs 40.0; slip stays
   in-band) — CANARY FAIL on the full DR panel, but by the smallest
   possible margin, with zero falls and perfect gait validity anywhere
   in the panel. **Conclusion: no rate-conversion hack (action-hold /
   interpolation) is even structurally necessary — direct inference
   at the target Hz/contract already sits at the gate's edge.**
3. `verify_modeseq_teachers.py` (existing tool, generalized this cycle
   — see below) driving `acq8m` (stance) + `stotight45-seed13` (walk)
   through `goal.mode_seq` composition on mesh/100 Hz, DR 0.5,
   stochastic_frac 0.3 (deliberately harder than the gate panels): 60
   sequences, 30s each — **6/60 (10%) fell, split `{hold: 3, rise: 3},
   ZERO walk-segment falls.** The failures match the segfix dig-in's
   own already-open residual (rise flat-start / hold min-load) exactly
   — this is not a new walk-composition pathology, it's the
   pre-existing stance-side story recurring under composition, which
   is the expected/tracked risk, not a surprise.
**Tooling built (real code, tested):** `verify_modeseq_teachers.py`
gained `--extra-cfg-set` (was hardcoded to a stale r3/r4c-era overlay
that doesn't match ANY current teacher's real obs-width-affecting cfg
— crashed on stotight45's 74-wide obs otherwise); `distill_gru.py`'s
`--cfg-set` parser now shares `train_ppo_sim._parse_cfg_set` instead
of a local float-or-string parser that silently mis-typed `[lo,hi]`
range overrides (e.g. `goal.rise_height_mm=[79,87]`, needed to match
acq8m's real training cfg) as a raw string — latent bug since 08-14,
never triggered because no prior `--cfg-set` caller passed a bracketed
value. `test_mode_seq*.py` (23 tests) stays green; both changes
smoke-tested end-to-end (tiny dual-BC run, 4 transitions/8 episodes/2
epochs, full pipeline completes and saves a loadable SB3 zip) before
committing to the real-scale run. **BC distillation COMPLETED this cycle** (background CPU, `nohup`,
not GPU/ledger-tracked — same class of work as every prior
`distill_gru` arm; two smaller attempts first appeared to hang at
20+ min with zero log output — root-caused as `distill_gru.py`/
`verify_modeseq_teachers.py` missing the `OMP_NUM_THREADS`-family
thread-pool cap `eval_checkpoint.py` already carries [fixed, tested,
committed] COMBINED with plain stdout block-buffering under
`> file 2>&1 &` hiding real progress until process exit — neither
script was actually hung, both just needed patience/unbuffered output
to observe). Final recipe: `--transitions 20 --episodes 100 --epochs
25 --mix walk=0.30,rise=0.40,lower=0.15,hold=0.15`, walk-teacher
`stotight45-seed13`, stance-teacher `acq8m`, env cfg = the exact
merged walk+stance training cfg from probes 1-3 above. Result:
`ppo_goal_cw_standwalk_stage2_dualbc1.zip` (`DualGruActorCriticPolicy`,
obs 80 = 74 walk-teacher-width + 6 mode-onehot, act 18) — BC actor RMS
0.0216 action units (~1.8deg), end-of-run probes: walk episode returns
2601/2734 (both strongly positive, matching/beating teacher-in-context
returns), hold 490/634 (positive), rise 2178/-710 (one strong pass),
2 composed sequence probes 1 PASS (return 2378, no fall) / 1 FALL. No
DAgger round used (one lever at a time — plain dual-teacher BC first).
**GPU canary pair LAUNCHED AND VERIFIED RUNNING this cycle** (the
Stage-2 walking-source x mechanism matrix's first cell: source=
primitive `stotight45-seed13` via direct-inference BC transfer,
mechanism=dual-teacher BC + RL fine-tune): `cw-standwalk-stance-mesh2-
stage2-dualbc1-modeseq1` (seed 0, train-0) / `-modeseq1-s1` (seed 1,
train-1), 2M steps, `train_ppo_mjx --gru-dual --gru-hidden-size 256
--init-from ppo_goal_cw_standwalk_stage2_dualbc1.zip --task joint_walk
--cfg-set obs.mode_onehot=1 --cfg-set goal.mode_seq=0.75` + the full
merged env cfg, no `train.bc_anchor_*` (mirrors the `cw-arch-gru-
dual1` precedent's own finding: the mode-gated dual-core architecture
removes the shared-trunk anchor/walk interference BY CONSTRUCTION, so
the bare fine-tune is the first thing to try; stance-only/walk-off
anchor is the pre-registered fallback if walk regresses). Hypothesis/
gate in the ledger entry; NOT triaged this cycle — a fresh 2M canary,
next finish-triggered cycle owns the read. Evidence: `/tmp/
probe_stotight45_meshhz25`, `/tmp/probe_stotight45_
meshhz100_truecontract{,_full,_ownDR}`, `/tmp/verify_modeseq_{smoke,
full}.json` (controller `/tmp`, not artifact-durable — rerun cheap if
needed for audit; each probe command is fully specified above),
`/tmp/dualbc1_v4.log` (the completed BC run's full log).
Prior entry, 08-26 ~09:1x (**SEGFIX JOINT CALL: REAL DIVERGENCE,
NOT A CLEAN PASS — the composed-segment-window widen (6-8s->9-11s)
FIXES seed1's severe flat-in-composition collapse outright but
REGRESSES seed0, which had no such problem before. DIG-IN flagged;
do not promote the widened window.** Own scope (`-segfix-s1`, seed 1)
ACQUISITION PASS: the generic composed seqprobe (registered
instrument) happened to draw ZERO flat-kind episodes in its n=12 rise
sample (confirmed per-episode `start_kind`: rsi/crouch/bridge only,
deterministic under the shared `--seed 0`) — untestable for the
flat-specific hypothesis as configured, so this cycle built a
DEDICATED flat-pinned composed probe (`goal.mode_seq_stance=1` +
`goal.rise_flat_frac=1.0/partial=0/rsi=0` on each recipe's own segment
window, n=12 det+sto) and ran it on BOTH seeds' segfix checkpoints
AND their un-widened acq8m parents for a direct, matched before/after
(4 extra CPU-pod eval passes, read-only diagnostics, on top of the
standard gate/owncfg/seqprobe). Results: **seed 1** acq8m-s1 (6-8s
window) 2/12 valid, 10/12 `hold_low_height` terms (herr 48-72mm) ->
segfix-s1 (9-11s window) **12/12 valid, ZERO terms, herr 0.3-3.6mm**
— clean, video-confirmed fix (contact sheets: genuine
splay->tuck->full-stand every draw). **seed 0** acq8m (6-8s window)
12/12 valid, ZERO terms, herr 0.3-1.9mm (no flat-in-composition
problem existed) -> segfix (9-11s window) **9/12 valid, 3 NEW
hold_low_height terms** (herr 49-52mm), video-confirmed genuine
stall-low (crouched terminal frame, never completes the last stretch
to full height) on the failing draws. The identical config change
that cures seed1 (2/12->12/12) breaks seed0 (12/12->9/12) — the
gate's own PASS branch and its FAIL branch ("unchanged or worse")
BOTH fire on the same arm, split by seed; this is not the
"unchanged/worse in both seeds" the gate anticipated, nor a clean
joint pass. The `-segfix` (seed0) run itself belongs to a concurrent
cycle — its evidence is cited here as read-only diagnostic
cross-reference, not verdicted on its behalf. **Recommendation: do
NOT promote the widened segment window as a blanket replacement;
`acq8m`/`acq8m-s1` (6-8s default) remain the standing stage-1
checkpoints.** DIG-IN flagged: root-cause why widening — which the
Next-0.5 root-cause note predicted could only ever HELP a
time-starved first segment — instead introduces a brand-new failure
in seed0 (leading suspect: the flat-time-indexed BC-anchor clock is
tuned to a ~7s reference ramp and may mis-pace differently inside a
9-11s segment for a subset of draws than it did inside the old 6-8s
one). SKILLS.md row added. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_standheight_rung5_acq8m_segfix_s1_{gate,
owncfg,seqprobe,seqprobe_flat}/` (this run, own scope), `..._segfix_
{seqprobe,seqprobe_flat}/` (sibling, cited only), `..._acq8m_
seqprobe_flat/` + `..._acq8m_s1_seqprobe_flat/` (matched pre-fix
baselines built this cycle). W&B `i50v00p9`.)

Prior entry: 2026-08-26 ~07:5x (**STAND_HEIGHT RUNG-5 JOINT CALL
CLOSED: the composed rise->hold(height-cmd)->lower lower-phase fix
holds on BOTH seeds under the CORRECTED motor slew contract -- rung-5
is closed for its own question; flat-start rise carries forward as
the pre-existing, seed-sensitive residual.** A concurrent cycle found
+fixed a live bug (`pod_eval.py::legacy_eval_cfgs`) that had been
silently running every 100 Hz run's automated gate/owncfg/session
pass under the LEGACY 25 Hz slew contract (1.5 deg/tick, 4x looser)
instead of the real 0.375 deg/tick the policies actually trained
under, since the 08-24 rate flip (161 of 1712 ledger launches
affected). This cycle found `-s1`'s own gate/owncfg/seqprobe were
genuinely pre-fix too (`report.json.motor_contract.safety.
max_delta_q_deg == 1.5`), deleted them, and re-ran all three on
train-0 under the landed fix. **Composed seqprobe (the registered
gate instrument) post-fix: lower/det 6/6 + lower/sto 6/6, BOTH
zero-termination (herr 9.4-10.7mm)** -- matches seed-0's own
already-corrected 6/6+6/6 (herr 14.1-14.9mm) -- the exact fix this
arm was funded for (2M canary `-s1` lower/det was 0/6, ALL
over_current-pinned) replicates cross-seed under the TRUE trained
contract. hold clean both seeds. **rise reveals a real cross-seed
divergence invisible under the buggy contract**: seed0 10/12 (2
over_current only) vs seed1 7/12 (5 fails, mostly a `hold_low_height`
stall concentrated on flat starts) -- the pre-existing flat-start-rise
residual (Next 0.5/RUNG-9), sharpened, not a new blocker. Video
(fresh post-fix contact sheets + isolated strips, both seeds):
upright, six-foot-planted, no fall/tip/collapse; rise fails are
stall-low, not falls. **Verdict: rung-5's own question (does
composing the height-cmd hold segment into rise->hold->lower
sequencing preserve skill) = YES, closed cross-seed. Next: open
stage-2 design** (rise->walk->lower composition) off either seed's
acq8m checkpoint; walking source is still the track's pre-registered
fallback (`stotight45-seed13`, primitive-family) pending a mesh-era
joystick champion. SKILLS.md row added (joint-scope). Full detail +
per-episode numbers in the "Now" section below. Evidence: `logs/
ckpt_eval/cw_standwalk_stance_mesh2_standheight_rung5_acq8m{,_s1}_
{gate,owncfg,seqprobe}/` (both post-fix; `-s1` pre-fix artifacts kept
as `*_prefix_bugged`), W&B `i78gd6k3`/`auf0f70c`. Any pre-08-26-~06:33
`control.hz=100` eval read (cur_max/over_current numbers especially)
should be treated as suspect until re-verified under the fix — see
CURRENT_TRUTHS.md.)

Prior entry: 2026-08-26 ~04:3x (**JOINT CALL CLOSED: JOINT PASS — the
from-scratch real-std-anneal full hold=.1/rise=.45/lower=.45 mix clears
every registered clause on BOTH seeds; PROMOTED as THE mesh stancemix
recipe, closing the multi-day from-scratch-vs-warm-started/std-reopen
saga. PLUS: built + tested the STAND_HEIGHT rung-5 wiring (mode_seq hold
segments can now carry a height command) and launched its first canary.**
`-s1` (seed 1, this cycle): flat-pinned probe 11/12 valid_plant (det 6/6,
sto 5/6 — 1 over_current fall, not a majority/pin), herr 0.2–8.6mm, real
per-leg swing counts; DR-0 gate hold 6/6+6/6 zero-term (herr≤5.5mm),
lower 6/6 det+6/6 sto success (herr 3.0–5.0mm, well under the 10mm bar —
`valid_plant` reads 0 for lower because that flag means rise-height
plant, not lower's own success criterion, which is `success=true`
here); own-DR(0.2) hold 6/6+6/6, lower 6/6+6/6, rise 5/6 det (1 OC term)
+ 6/6 sto. Combined with seed-0's own 12/12 flat + zero-term hold/lower
(posted last entry), **both seeds clear all three registered clauses
(flat probe ≥10/12, hold ≥5/6+5/6 zero-term, lower ≥5/6 honest ≤10mm) —
JOINT PASS.** Promoting seed-0's checkpoint
(`ppo_goal_cw_standwalk_stance_mesh2_stancemix_tuckclock_scratch8m.zip`,
marginally cleaner: 12/12 vs 11/12 flat, zero own-DR rise terms vs
seed-1's one) as THE mesh stancemix recipe. Residual (both seeds, same
shape as every prior "solved" rise arm): cur_max still kisses 2.4–2.64A
on most rise episodes without tripping (structural, non-terminal); a
thin over_current/fall tail remains on the hardest deep starts
(bridge/rsi/flat-sto), ~3/84 read episodes per seed across all three
reports — this is Stage-1's cleanest result yet, not a zero-fall
closure, so it does not by itself close the track's Stage-1 GATE text
("zero falls/tips"). SKILLS.md row added (joint-scope). **Refill (built
this cycle, real code):** STAND_HEIGHT.md's rung-5 text claimed
`goal.mode_seq_stance`'s hold segments already carry a height command
"as a drop-in" — false on inspection: `_seq_segment_traj`'s hold branch
never called the existing `_hold_height_schedule` (mid-sequence hold
segments were always flat-zero regardless of `hold_height_cmd_frac`).
Added `goal.mode_seq_hold_height_cmd` (default 0 = OFF, bit-exact — the
array stays the pre-initialized zeros exactly as before) in
`rl_move/sim/goal_task.py::_seq_segment_traj`: ON (+ the generator's own
`hold_height_cmd_frac>0`) calls `_hold_height_schedule` in the segment's
own local clock (`n - tick` steps), which composes cleanly because that
schedule already starts every draw at 0 with its own settle window —
the same convention the rise/lower branches already rely on for a
seamless segment-switch. 4 new tests
(`rl_move/tests/test_mode_seq_hold_height.py`: default-off flat,
on-but-frac-zero flat, on-composes varying/continuous/in-range, unset-
vs-explicit-zero stream parity) + the full `test_mode_seq_stance.py`/
`test_mode_seq.py`/`test_hold_height_cmd.py`/
`test_eval_modeseq_rise_from_h.py` banks stay green (36/36). Launched
the pre-registered STAND_HEIGHT rung-5 first canary off THIS cycle's
promoted checkpoint (see below). Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_stancemix_tuckclock_scratch8m_s1_{gate,owncfg,
flatprobe}/`, W&B `nl1im163`.)

Prior entry: 2026-08-26 ~04:2x (**SCRATCH8M (seed 0) ACQUISITION PASS — the
cleanest single-seed mesh stancemix read of the whole campaign, and the
first from-scratch (not warm-started, not pinned-std) full-mix arm to be
read.** Flat-pinned probe (det+sto 6+6, DR-0): 12/12 valid_plant, herr
0.6–7.1mm, real per-leg swing motion (e.g. [13,0,1,0,5,1]), roll clean —
video confirms genuine splay→tuck-under→level six-foot plant, NOT the
stdreopen/seqrise family's press-pin or half-mast freeze (the 2.4–2.64A
seen during tuck/press is the same non-terminal structural ceiling every
solved rise arm rides, not a frozen pin: swings are real, herr is low).
DR-0 gate: hold 6/6+6/6 AND lower 6/6+6/6 both **ZERO terminations**
(herr ≤0.7mm hold, ≤2.8mm lower) — both registered clauses cleared with
zero terms, not merely above-bar; rise/sto DR-0 4/6 (2 bridge/rsi
over_current falls — the campaign's already-catalogued deep-start OC
tail, not new). Own-DR(0.2): hold det 6/6 + sto 5/6 (1 hold_min_load
term); **rise det 6/6 + sto 6/6, ALL valid_plant zero-term** (stronger
than this seed's own DR-0 rise/sto); lower 6/6+6/6 zero-term. Reward rose
every quarter (-32.4/20.5/659.2/1236.9), matching the isolated
tuckclock-acq8m trough-then-breakout arc this recipe deliberately
mirrors. **Verdicted own-scope only — the registered gate is JOINT with
`-s1`** (seed 1, identical from-scratch recipe, still training as of this
entry — off-limits to this cycle). If `-s1` also clears every clause,
this closes the from-scratch-vs-warm-started/std-reopen saga outright:
promote as THE mesh stancemix recipe, unblocking STAND_HEIGHT rungs 4-5
and walk distillation. If `-s1` instead shows the seed-fragility pattern
the stdreopen-acq8m pair did (2/12 vs 11/12), the honest read becomes
"from-scratch real-std-anneal is ALSO only partially seed-reliable" —
worth a 3rd seed before promoting either family. SKILLS.md row added
(seed-0 scoped). Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
stancemix_tuckclock_scratch8m_{gate,owncfg,flatprobe}/`, W&B `pjatb078`.)

Prior entry: 2026-08-26 ~03:3x (**STDREOPEN 3-SEED READ CLOSED:
seed-2 matches seed-0's total freeze (0/12 flat valid_plant, WORSE
than seed-0's 2/12, all 12 over_current-pinned at 2.64A, near-zero
swing_count), not seed-1's clean 11/12 pass — CANARY FAIL - MECHANISM
recorded. Three seeds now read 2/12, 11/12, 0/12: genuinely ~1/3
reliable, exactly this canary's own pre-registered "seed2 matches
seed0" branch. Hold/lower stay clean regardless (12/12+12/12 DR-0
both), confirming the flat sub-case alone is the broken piece. Root
cause is NOT the recipe's own "std reopen" lever, though — the
concurrent seqrise dig-in (see next entry) found `--log-std-init` is
a silent no-op under `--init-from`, so this entire warm-started
sub-lineage (stdreopen/-s1/-acq8m/-acq8m-s1/seqrise/-s1/-s2) trained
at the parent's pinned std 0.0183 the whole time; the seed spread is
pinned-std warm-start optimization variance, not an exploration
effect. No further stdreopen-recipe seeds are warranted — the correct
lever is the from-scratch real-std-anneal arm below.
`cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m`'s first
launch attempt was mechanically REFUSED (pod code-sync) and only its
retry (seed 0, train-6) ended up running with no seed twin queued;
completed the pre-registered pair this cycle —
`cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m-s1` (seed 1,
identical from-scratch recipe) launched VERIFIED RUNNING on train-7.
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_
tuckclock_stdreopen_s2_{gate,owncfg,flatprobe}/`, W&B `s9yxuq93`.)

Prior entry: 2026-08-26 ~03:2x (**SEQRISE JOINT CALL CLOSED: CANARY
FAIL - MECHANISM on both seeds — warm-starting the 3-way mix from the
solved riseonly flat-rise checkpoint does NOT preserve the skill —
PLUS a lineage-wide mechanical discovery that re-frames the whole
stdreopen family.** Seed 0 (this cycle, train-4 probe + gate, strips
watched): flat probe 0/12 valid_plant, robot lies belly-down all
episode waving raised legs (herr 79.3–87.0mm), NO OC-pin (cur_max
0.55–1.59A, zero terms, real swings on all six legs) — a THIRD
pathology (quiet lying) vs stdreopen's press-pin and s1's half-mast
plateau (30–34mm); hold 12/12 zero-term but parked 13.1mm off; rise
det 5/6 / sto 3/6; **lower 0/12** (herr 15.6–21.6mm, zero terms,
hangs at mid-height, never completes the sit) — the "hold/lower easy
to re-acquire" premise is refuted at 2M on this seed. **DISCOVERY
(W&B `train/std` + trainer source, encoded in CURRENT_TRUTHS):
`--log-std-init` is a SILENT NO-OP under `--init-from` — the entire
stdreopen sub-lineage (stdreopen/-s1/-acq8m/-acq8m-s1/seqrise/-s1,
and the running stdreopen-s2) trained at pinned std 0.0183 from step
0. The "std reopen" lever never existed; stdreopen-2M's "reopening
exploration collapsed tuck motion" mechanism story is refuted (those
were config-identical tuckclock1 re-runs; differences = replicate
variance), and the acq8m 2/12-vs-11/12 seed divergence is variance of
pinned-std warm-started mix training. The genuine lever is
`--warm-log-std-override`.** Re-pricing (the seqrise gate's
registered FAIL branch) deprioritized with cause: static pricing is
measurably correct (solved policies earn ~+1385 on in-mix flat rise
vs −344..−772 for lying; acq8m-s1 reached 11/12 with this pricing) —
the failure class is optimization-path fragility, not mispricing.
**Refill (launched this cycle): `cw-standwalk-stance-mesh2-stancemix-
tuckclock-scratch8m`/`-s1` — FROM-SCRATCH 3-way mix, flat clock, real
std 1.0→0.018 anneal (frac 0.5 = 4M), 8M x 2 seeds — the untested
cell two prior dig-ins flagged, methodology-matched to the only
recipe that solved mesh flat rise 2/2 (riseonly-tuckclock-acq8m
24/24). Named fallback if it fails: warm-start + `--warm-log-std-
override 0`.** Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
stancemix_seqrise_{gate,owncfg,flatprobe}/`, W&B `9zzi5ael`.)

UPDATE (03:3x, this cycle): the scratch8m launch above was initially
REFUSED (stale code marker on train-6, `95e8933f...` vs local HEAD) —
`snapshot.sh --sync hexapod-mjx-train-6` + `--sync hexapod-mjx-
train-7` fixed it; **both `scratch8m` (seed 0, train-6) and `-s1`
(seed 1, train-7) are now confirmed VERIFIED RUNNING (8M each)** —
no -s1 twin existed yet, launched this cycle with the identical
recipe/seed=1 to make it a real joint pair per this campaign's
2-seed convention. Also closed this cycle (own scope, complements
the seqrise joint call above): `cw-standwalk-stance-mesh2-stancemix-
tuckclock-stdreopen-s2` (the 3rd stdreopen seed) VERDICTED CANARY
FAIL - MECHANISM — flat probe 12/12 over_current-terminated at the
exact 2.64A ceiling, zero swings, pattern-matching seed0's
total-freeze (NOT seed1's clean pass); hold/lower clean both DR
settings. This closes the SEED-NOISE half of the earlier joint
question standalone: with seed0=freeze, seed1=pass, seed2=freeze,
the stdreopen recipe (warm-started, pinned-std per the discovery
above) is only ~1/3 seed-reliable — consistent with, and independent
confirmation of, the decision above to move to the from-scratch
real-std-anneal recipe rather than fund more stdreopen seeds.
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_
tuckclock_stdreopen_s2_{gate,owncfg,flatprobe}/`, W&B `s9yxuq93`.

Prior entry: 2026-08-26 ~03:1x (**seqrise-s1 (own scope) CANARY FAIL
- MECHANISM: the SEQUENCING lever avoids the OC-pin entirely but the
flat probe still doesn't reach a valid plant — it stalls at a NEW,
stable half-rise plateau instead. Separately, a concurrent cycle's
dig-in on the sibling `seqrise` run found a load-bearing bug:
`--log-std-init` is a SILENT NO-OP under `--init-from` (see
CURRENT_TRUTHS "CORRECTION (08-26, seqrise dig-in)"), so the ENTIRE
stdreopen/acq8m/seqrise sub-lineage actually trained at the parent's
pinned std 0.0183 the whole time — "reopening exploration" never
happened, and the acq8m s0/s1 2/12-vs-11/12 divergence is warm-started
mix-training seed variance, not an exploration effect.** This cycle's
own read (`cw-standwalk-stance-mesh2-stancemix-seqrise-s1`, W&B
`6x4ivpd1`, evidence `logs/ckpt_eval/cw_standwalk_stance_mesh2_
stancemix_seqrise_s1_{gate,owncfg,flatprobe_det,flatprobe_sto}/`):
flat-pinned probe 0/12 valid_plant (0/6 det, 0/6 sto) — BUT for the
first time in this whole rise campaign, NOT current-pinned (cur_max
2.04–2.62A, well under the 2.64A trip, zero terminations) and NOT
frozen (uniform per-leg duty 0.67–0.78, real swing counts every leg,
leg-idx1 hitting 9–26 swings). Video/contact-sheets (det+sto, both
clean): genuine splay→partial-tuck that SETTLES INTO A STABLE
HALF-RISE PLATEAU (height_err_end pinned 30–34mm — roughly triple the
solved lineage's <5mm) and never completes the push to the 79–87mm
target for the full 15s episode. This is a THIRD residual pathology —
neither stdreopen's over-current press-up pin nor tuckexempt's
snap-fold freeze — call it "stalls at half-mast." Non-flat/mixed-kind
DR-0 gate stays at-or-above the meshref parent's band (hold 6/6+6/6
zero-term, rise det 5/6 + sto 4/6 valid_plant, lower det+sto 0 terms);
own-DR(0.2) hold 6/6+6/6 zero-term, lower 0 terms both, rise 3/6 det +
2/6 sto (1 OC term each) — the compound gate's hold/lower clause
(≥5/6+5/6 zero-term) is clearly met on this seed alone; only the flat
valid_plant sub-clause misses (0/12 clears neither the 2/12 nor 11/12
reading of the registered comparator floor), and the literal FAIL
trigger ("still-majority-pinned") is also false here — a genuine
third outcome the gate text didn't anticipate. **JOINT SEQUENCING call
NOT decided here** — needs the sibling `seqrise` (seed 0, concurrent
cycle) and `stdreopen-s2` (still training) reads. If the sibling also
plateaus rather than pins, the honest joint read is "sequencing swaps
one residual (current-pin) for another (stalled plateau) — net motion
in the right direction, not yet solved," and likely worth an 8M
extension since the plateau is NOT current-limited (unlike the pin),
so budget has real room to work with. Evidence W&B `6x4ivpd1`.)

Prior entry: 2026-08-26 ~02:2x (**seed0 twin of the stdreopen-acq8m
pair now formally VERDICTED (FAIL, own scope) — the joint call the
prior entry flagged is now a confirmed 2/12 vs 11/12 split, not a
placeholder. Refill: 3-arm batch queued to resolve it (SEQUENCING vs
seed-noise), all VERIFIED RUNNING.** `cw-standwalk-stance-mesh2-
stancemix-tuckclock-stdreopen-acq8m` (s0) VERDICTED FAIL: flat probe
2/12 valid_plant (10/12 over_current-pinned at the exact 2.64A
ceiling, video-confirmed splayed press-up with no belly lift),
essentially unmoved from its own 2M canary (0/12) despite reward
rising 5x (quarters -52.0/174.9/764.5/1267.5) and hold/lower staying
fully clean (DR-0 6/6+6/6 both, own-DR 6/4 and 6/4) — the flat-rise
clause alone is budget-invariant on this seed, matching the gate's
own pre-registered FAIL branch, while the s1 twin (posted by a
concurrent cycle, prior entry below) cleared every clause at 11/12.
Two clean readings of the identical 8M recipe/budget/warm-start,
opposite outcomes — real seed-sensitivity, not a race/read error (both
sides re-checked their own report.json term_reason/valid_plant fields
directly, not just summary lines). **Refill (batch, 3 launches, one
question each):** (1)/(2) `cw-standwalk-stance-mesh2-stancemix-
seqrise`/`-s1` (2M canary pair) — the SEQUENCING lever: exact stdreopen
recipe (goal-mix hold=.1/rise=.45/lower=.45, mesh ref, flat-time-
indexed BC-anchor chain, log-std 0->-4 anneal-frac 0.5) but
`--init-from` swapped to the ALREADY-SOLVED riseonly-bcchain3-meshref-
tuckclock-acq8m checkpoint (12/12 flat valid_plant alone) instead of
`stancemix_bcchain3_stdanneal` (which never saw the mesh-native ref
during its own training) — tests whether starting the mix from a
policy that already knows how to stand up flat sidesteps the
interference entirely. Both finished their short 2M budget mid-cycle
(pods free again within ~13 min); evals (gate/owncfg/flatprobe) were
started but NOT read this cycle — next cycle triages them fresh. (3)
`cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-s2` (2M
canary) — the SEED-NOISE lever: identical stdreopen recipe, seed=2,
nothing else changed, to see whether a 3rd seed lands nearer seed0's
total-freeze or seed1's clean pass (the isolated tuckclock-acq8m
precedent replicated 24/24 cross-seed at just 2 seeds, so a 1/2 split
here is itself informative about how reliable this particular mix
port is). Whichever cycle reads these three should treat them as a
joint set: SEQUENCING passing on both seqrise seeds argues for
adopting the solved-checkpoint warm-start going forward regardless of
what stdreopen-s2 shows; stdreopen-s2 matching seed1 argues the
existing recipe is fine and seed0 was the outlier; stdreopen-s2
matching seed0 argues for a real ~1/3-reliable recipe needing either
more seeds or the seqrise fix. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_stancemix_tuckclock_stdreopen_acq8m_
{gate,owncfg,flatprobe_det,flatprobe_sto}/`, W&B `5xk1serz` (s0 FAIL),
`hryo39z1` (s1 PASS, prior entry); new arms W&B `9zzi5ael` (seqrise),
TBD (seqrise-s1, stdreopen-s2 — check ledger, not yet in local W&B
cache as of this write).)

Prior entry: 2026-08-26 ~02:1x (**STDREOPEN-ACQ8M-S1 (seed 1) ACQUISITION
PASS on all three pre-registered clauses, decisively — but the seed-0
twin's own already-posted evidence is a SHARP DIVERGENCE (2/12 vs
11/12 flat-pinned valid_plant), so the joint call this pair was set up
to answer is genuinely split, not a clean pass or fail.** The
`--evidence` the launcher demanded (see prior entry) DID get supplied
by whichever cycle relaunched this pair at 01:24 (evidence cites the
isolated `riseonly...tuckclock-acq8m` full-budget precedent) — both
8M runs completed cleanly. Seed-1's own read
(`logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_tuckclock_
stdreopen_acq8m_s1_{gate,owncfg,flatprobe}/`, W&B `hryo39z1`): flat
probe 11/12 valid_plant (det 6/6, sto 5/6, herr 0.2-10.9mm,
video-confirmed splay→tuck→plant), standard DR-0 gate 35/36 success
(hold 6/6+6/6, rise 6/6+5/6, lower 6/6+6/6), own-DR(0.2) 33/36 success
— every clause (>=10/12 flat, hold >=5/6+5/6, lower >=5/6 honest)
cleared with wide margin, the strongest mesh-mix stance read of the
whole campaign. Seed-0's own posted flat probe
(`logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_tuckclock_
stdreopen_acq8m_flatprobe_{det,sto}/`, not yet formally verdicted —
that run belongs to a concurrent cycle) is only 2/12 valid (0/6 det,
2/6 sto), i.e. essentially the SAME never-tucks 2.64A press-up
signature as its own 2M canary — budget did not move it at all,
which is precisely the gate's own pre-registered FAIL branch
("budget-invariant vs the 2M canary ... in both seeds"), except it is
true in only ONE seed while the other seed converged cleanly. Neither
named branch (clean joint PASS, clean joint FAIL) fits: this is a
real seed-sensitivity finding on an otherwise-identical
recipe/budget/warm-start. **Open call for whoever closes the pair**
(likely the cycle triaging the seed-0 twin, or a fresh cycle if that
one has already moved on): (a) promote seed-1's checkpoint alone as
THE mesh stancemix checkpoint — the gate text is written per-seed
("this becomes THE mesh stance mix checkpoint"), and seed-1
individually clears every clause; risk = the recipe itself may not be
reliably reproducible, so a lone-seed promotion is fragile evidence
for "this becomes the walk-distill base"; or (b) treat this as
PARTIAL — fund a 3rd seed (or re-run seed-0 with a different seed
value, since the failure could be an unlucky init rather than a
structural recipe defect) before promoting, given the campaign's own
standing bar of cross-seed replication before treating a rise fix as
solved (the isolated tuckclock-acq8m precedent this pair mirrors DID
replicate 24/24 across both its seeds). Recommendation (not binding,
this cycle only owned seed-1): (b) is more consistent with this
campaign's own track record of demanding cross-seed replication
before promoting a rise recipe — a single passing seed out of two,
with the other showing NO movement at all from its 2M canary, reads
more like "seed-1 got lucky/seed-0 got unlucky on a marginal fix"
than "the recipe is solved." SKILLS.md row added (seed-1 scoped,
divergence flagged). Evidence as above; W&B `hryo39z1` (s1) /
`5xk1serz` (s0, unverdicted).)

Prior entry: 2026-08-26 ~01:2x (**RECONCILING NOTE — two cycles
independently triaged the same stdreopen pair concurrently (both saw
it finish and picked it up as free-capacity runnable work); both
landed CANARY FAIL - MECHANISM independently from the same evidence,
but proposed different next steps. The queued `-acq8m`/`-acq8m-s1`
8M continuation named below was REFUSED mechanically (missing
`--evidence`, a launcher precondition for `acquisition`-phase
launches — not a judgment override) and never ran; nothing is
in flight on this lineage right now.** Given that launch didn't
happen, the live open question is genuinely undecided: is "fund 8M
on this exact recipe" still the right call, given (a) the flat probe
got WORSE with std reopened (total freeze both seeds, vs the
pinned-std parent's partial motion) rather than merely insufficient,
and (b) the isolated `riseonly...tuckclock`/`tuckclock-acq8m` recipe
that actually solved flat-start rise trained FROM SCRATCH at every
budget (`--init-from None`), while every mix respec in this lineage
(`tuckclock1`, `stdreopen`) has been warm-started from the converged
`stancemix_bcchain3_stdanneal.zip` — "mirroring the isolated arc" via
an 8M continuation of a warm-started checkpoint is not actually the
same methodology the cited precedent used. Both readings agree the
ep_rew trough shape at 2M is consistent with the isolated arm's own
2M trough (a real, cited data point in favor of more budget), but
neither cycle treated the from-scratch-vs-warm-started mismatch as
resolved. **DIG-IN stands** as the honest state: next cycle picking
this up should supply the `--evidence` the launcher demands (which
also forces writing down which precedent — continuation vs
from-scratch — is actually being claimed) before relaunching, not
just retry the same REFUSED command. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_stancemix_tuckclock_stdreopen{,_s1}_
{gate,owncfg,flatprobe}/`, W&B `1g08bnoc`/`77l3258u`.)

Prior entry: 2026-08-26 ~01:1x (**STDREOPEN JOINT CALL: CANARY FAIL
on the pair's own pre-registered budget route — reopening the std at
2M does NOT unpin flat starts in the 3-way mix; the registered next
lever (8M on the exact recipe) is queued.** This cycle verdicted both
`cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen` (s0) and
`-s1`: flat-pinned probe (rise_flat_frac=1.0/partial=0/rsi=0, det+sto
6+6, DR-0, run by hand on train-4/5) is **24/24 over_current-
terminated across the pair, every episode at the exact 2.64 A
press-up ceiling, swing_count ~0 on every leg** (s0: 5 total swings
in 12 eps; s1: 0), herr_end 2.7–25.2 mm and not settling; contact
sheets show the identical splay→stiff-leg press-up/no-tuck signature
as tuckclock1. The OTHER canary clauses PASS on both seeds (hold
6/6+6/6 zero-term both; lower s0 det 5/6 + sto 5/6, s1 det 6/6 + sto
5/6 honest; non-flat rise det 6/6 (s0) / 5/6 (s1)) — the noise
re-injection did NOT damage hold/lower. KEY READ: ep_rew_mean FELL
all run on both seeds (s0 4.9/-37.2/-55.8/-109.4, s1
0.7/-28.7/-52.0/-104.8) — which precisely matches the isolated
`riseonly...tuckclock-acq8m` trajectory at its own 2M mark (-58 and
falling, trough -198 at 3M, then +1570 with flat rise 12/12 solved by
8M): a 2M canary truncates this recipe mid-trough, before the 0→-4
std anneal completes and pays off. **Refill (the gate's own
registered FAIL route, budget not config): queued
`...stdreopen-acq8m`/`-acq8m-s1` — exact stdreopen respec (warm from
stancemix_bcchain3_stdanneal, mesh ref + flat clock, std 0→-4
anneal-frac 0.5), ONLY steps 2M→8M (anneal stretches to 4M).** Gate:
flat probe ≥10/12 valid_plant per seed, no majority 2.64A pin, hold
≥5/6+5/6 zero-term, lower ≥5/6 honest → THE mesh stance mix ckpt,
unblocks rungs 4-5/walk distill; FAIL (pin budget-invariant vs 2M in
both seeds, or reward still falling at 6M) → mix-context interference
is structural, next lever is SEQUENCING (riseonly flat acquisition
first, then re-introduce the mix), not more budget. Watcher SUSPECT
on -s1 was benign (normal finish at 2.03M). Evidence: `logs/
ckpt_eval/cw_standwalk_stance_mesh2_stancemix_tuckclock_stdreopen{,_
s1}_{gate,flatprobe}/`, W&B `1g08bnoc`/`77l3258u`.)

Prior entry: 2026-08-26 ~00:1x (**STANCEMIX-TUCKCLOCK1 JOINT CALL:
FAIL — the proven flat-start rise recipe does NOT survive porting
into the full hold+rise+lower mix at a PINNED std; both seeds still
pin/freeze on flat starts exactly as the canary's own pre-registered
FAIL route named. Next lever (re-open std) launched.** This cycle
verdicted both `cw-standwalk-stance-mesh2-stancemix-tuckclock1` (seed
0) and its `-s1` twin, running the flat-pinned pod probe manually
(neither run had it prestaged — only the standard gate/own-DR were
auto-started; launched `goal.rise_flat_frac=1.0/partial=0/rsi=0`
det+sto 6+6 by hand on each pod) plus the standard DR-0/own-DR
passes that were missing for seed 0 entirely (also launched by
hand — the watcher's "just finished" trigger only covered `-s1`,
seed 0 had finished earlier with no prestage). Flat probe: **seed 0**
det 1/6 valid_plant (5/6 over_current-terminated, all pinned
`cur_max_a=2.64A`), sto 4/6 valid_plant (0 term, but STILL pinned
2.64A every one of the 6 episodes) — every one of 12 episodes hits
the exact 2.64A ceiling the PASS bar requires ABSENT; **seed 1** is a
clean total freeze: 12/12 (det+sto) over_current-terminated,
near-zero `swing_count` on every leg in the large majority, height_err
GROWING not settling (7.0–22.9mm det, 8.9–20.1mm sto), pinned 2.64A
throughout — worse than seed 0's partial motion, and precisely the
"tuckfloor/tuckexempt freeze" signature the joint gate's FAIL route
names verbatim. Standard mixed-start DR-0 gate is otherwise strong on
BOTH seeds (hold 6/6+6/6 zero-term, lower 6/6+6/6 (s0) / 5/6+5/6 (s1)
zero-term, rise/det 6/6 (s0, non-flat draws) / 5/6 (s1, 1 bridge
term) zero-term) — hold/lower transfer perfectly and most non-flat
rise draws still succeed; it is specifically FLAT starts that pin,
exactly the predecessor `stancemix-bcchain3-slowchain` FAIL shape.
**Root cause:** this port warm-started from the already-converged
`stancemix_bcchain3_stdanneal.zip` with std PINNED the entire run
(`--log-std-init -4.0 --log-std-final -4.0`, i.e. zero exploration) —
the isolated `riseonly...tuckclock`/`tuckclock-acq8m` recipe that
actually solved flat-start rise trained FROM SCRATCH with std
annealing 0→-4 over the whole run. This confirms the run's own
pre-registered Prediction-if-false verbatim ("the flat clock needs
exploration noise on a warm policy, next arm re-opens the std").
**Refill (the canary's own registered FAIL route, not a new
hypothesis):** launched `cw-standwalk-stance-mesh2-stancemix-tuckclock-
stdreopen`/`-s1` (train-4/5, VERIFIED RUNNING, 2M canary pair) — exact
tuckclock1 respec with ONLY `--log-std-init` flipped 0 (was -4.0,
pinned); `--log-std-final -4.0` / anneal-frac 0.5 unchanged so std
still re-anneals to exploitation by the end, mirroring the isolated
recipe's own schedule. Gate: flat probe genuine non-freeze tuck both
seeds (no 2.64A pin in the majority of episodes) + hold ≥5/6+5/6
zero-term + lower ≥5/6 honest → fund an 8M mix acquisition pair
(mirrors the isolated recipe's own 2M→8M arc); FAIL (still
pins/freezes) → the next lever is budget (fund 8M on this exact
recipe directly), not another config change — reopened std alone may
not be enough at only 2M in a 3-way mix, matching how the isolated
arm itself needed the full 8M to fully close (its own 2M canary was
only a partial fix, 11/12 fell). Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_stancemix_tuckclock1{,_s1}_{gate,owncfg,
flatprobe}/`, W&B `q0l7wu20`/`ycgendqa`.)

Prior entry: 2026-08-26 ~00:0x (**RUNG-3 ACQ8M JOINT CALL: FAIL,
FALLBACK FIRES ON BOTH SEEDS — 8M budget is closed on this recipe;
the rung-3 champion stays the 2M canary pair.** This cycle verdicted
`-acq8m-s1` and closed the joint call seed-0's cycle deferred:
seed-1 reproduces the SAME misalignment shape independently rather
than being clean — DR-0 gate is letter-clean (det 6/6 + sto 6/6
valid_plant, zero terminations) but still breaches the tripwire on
BOTH named clauses: 2/6 sto episodes duty_min 0.84 (<0.85 floor) and
cur_max climbs to 1.08–1.72 A (rung-2/3 band is 0.62–1.06 A, nearly
2x the top); own-DR 0.2 is worse still — 4/12 hold_min_load
terminations (2 det + 2 sto), duty crashing to 0.17/0.37/0.62/0.76
on the failing episodes, vs the 2M canary's 1/12. Video
(`hold_det_5`, own-DR) shows the same single-leg unload/lift
signature named at rungs 1-2, ending `TERM hold_min_load`. Reward
rose all run (quarters 76.5/325.2/563.3/715.8, nearly identical
shape to seed-0's 79.8/316.9/545.0/709.3) while both evals
regressed → 08-21 MISALIGNMENT branch on both seeds independently,
not a lucky/unlucky single seed. **JOINT CALL: the tripwire's
either-seed condition is now met TWICE over — the registered
S-gate/min-load-pricing fallback FIRES.** Neither 8M ckpt is
promoted; rung-3 stays on the 2M canary pair
(`holdheight-rung3-hha1`/`-s1`). Next lever is reward/env code (price
the S-gate/min-load margin directly so honest all-duty holds beat
partial unloading even under 4x budget optimization pressure) — DIG-IN
already flagged by seed-0's cycle, not duplicated here. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_holdheight_rung3_acq8m_s1_
{gate,owncfg}/`, W&B `ux23038c`.)

Prior entry: 2026-08-26 ~00:0x (**ACQ8M CONTINUATION FAIL, seed 0 —
THE RUNG-3 TRIPWIRE FIRED. The registered S-gate/min-load-pricing
fallback is now LIVE, no longer holstered.**
`cw-standwalk-stance-mesh2-holdheight-rung3-acq8m` (8M, warm from the
rung-3 seed-0 canary ckpt) verdicted FAIL on its own pre-registered
fail branch: extended optimization RE-BUYS the leg-unload cheat —
DR-0 det 6/6 clean (duty floor 0.91) but sto 2/6 `hold_min_load`
terms at per-leg duty floors 0.75/0.76 (legs idx3/idx5; tripwire hit
on BOTH clauses: duty<0.85 AND DR-0 min-load terms, either-seed
suffices); det Imax drifts to 1.07–1.12 A (above the 1.06 A band
top); own-DR 0.2 = 2/12 terms vs the canary's 1/12 — the target
residual is budget-invariant/worse at 4x budget. Reward rose all run
(79.8/316.9/545.0/709.3) while evals regressed vs the 2M canary
(duty floor 0.98, zero DR-0 terms, Imax ≤0.82) → 08-21 MISALIGNMENT
branch: reward still pays for partial unloading the S-gate doesn't
price. **Rung-3 champion remains the 2M canary ckpt pair
(`holdheight-rung3-hha1`/`-s1`); the 8M ckpt is NOT promoted.**
NEXT: (a) DIG-IN flagged for the fallback implementation —
S-gate strengthening / min-load pricing is reward/env code + bank
rows (`test_task_semantics.py` must rank honest all-duty holds above
partial unloading UNDER the new pricing before any relaunch),
warm-start candidate = the rung-3 canary ckpts, NOT this run's; (b)
`-acq8m-s1` (finishing ~23:4x, benign wrap-up confirmed twice) gets
its own scoped triage but the joint call is already decided — the
tripwire is either-seed, so its cycle should verdict within scope
and NOT fund more budget on this recipe. More-budget on rung-3 is
CLOSED. Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
holdheight_rung3_acq8m_{gate,owncfg}/`, W&B `nuw6ttej`.)

Prior entry: 2026-08-25 ~23:3x (**STAND_HEIGHT RUNG-3 JOINT CALL:
PASS — the height ladder's in-scope rungs (1–3) are COMPLETE. The
stance follows the full hold/ramp/sine/pulse command mix over
[-40,20] mm at 15 mm/s, cross-seed, tripwire NOT fired.** This cycle
verdicted `holdheight-rung3-hha1` (seed 0) + `-s1` (seed 1), both
CANARY PASS - MECHANISM: DR-0 12/12 valid_plant per seed, ZERO
hold_min_load terms, h_err_end ≤2.8 mm, det Imax 0.70–0.82 A (inside
the rung-2 band), per-leg duty floor 0.98 (s0) / 0.95 (s1) vs the
0.85 tripwire; frame strips level/planted, micro-swings sub-visual.
The rung-2 trace dip family SHRANK again (0.87–0.89 → 0.95–0.98
floors) — the S-gate/min-load-pricing fallback stays holstered but
REGISTERED. Caveats: one s0 sto ep Imax 1.50 A (envelope ≤1.38 A) at
duty ≥0.98, judged current noise; own-DR 0.2 keeps EXACTLY 1/12 det
hold_min_load term per seed (persists from rung 2 — the open
hardening gap). Reward rising at both cutoffs (s0 quarters
20.5/50.2/79.2/176.4) → per the 08-21 ruling LAUNCHED the 8M
acquisition continuation pair, exact rung-3 recipe, warm from the
rung-3 SEED-0 ckpt (higher duty floor):
`cw-standwalk-stance-mesh2-holdheight-rung3-acq8m`/`-acq8m-s1`.
Gate: own-DR 0.2 12/12 zero min-load terms (the target residual),
DR-0 clean with the tripwire carried; FAIL branch (budget-invariant
own-DR term) fires the registered min-load-pricing fallback, not
more budget. Rungs 4–5 stay behind the stancemix port pair the
~23:1x cycle launched. SKILLS.md row added (ladder skill, both
ckpts). Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
holdheight_rung3_hha1{,_s1}_{gate,owncfg}/`, W&B kpgowofs/nws2zs8y.)

Prior entry: 2026-08-25 ~23:1x (**BOTH REGISTERED JOINT CALLS CLOSED
— (1) ACQ8M JOINT PASS: flat-start mesh rise SOLVED CROSS-SEED, recipe
promoted, stancemix port pair launched; (2) RUNG-2 JOINT PASS: full
[-40,20]mm height range clean cross-seed, rung-3 kind-mix pair
launched.** This cycle verdicted the two seed-1 twins and executed
both joint calls the ~23:0x seed-0 cycle delegated:

1. `tuckclock-acq8m-s1` (8M, seed 1) **ACQUISITION PASS** + **JOINT
   CALL: JOINT PASS.** Seed-1 flat-pinned probe (run on train-1)
   12/12 valid_plant, h_err_end 0.9-4.4mm, zero terms, roll_tail
   <=0.1° — the 2M flailing-fall mode (11/12 fell) fully resolved by
   budget; combined with seed-0's own 12/12 the pair lands 24/24 on
   the primary criterion. Non-flat gate 4/6+4/6 vs parent 5/6+4/6:
   sto parent-identical (2 rsi OC fells); det includes the lineage's
   FIRST flat gate pass; only slip = bridge det (one OC fell + one
   footprint-only miss that planted all six feet, success=True) —
   judged small-n noise, not a class regression. **PROMOTION
   EXECUTED:** `train.bc_anchor_flat_time_indexed=1` +
   `rise_ref_mesh_scripted.npz` is THE mesh rise recipe. **Stancemix
   port launched** as the registered on-PASS move:
   `cw-standwalk-stance-mesh2-stancemix-tuckclock1`/`-s1` (train-4/5,
   VERIFIED RUNNING, 2M canary pair) — exact `slowchain` respec with
   ONLY the two recipe keys changed (slowchain already carries the
   0.25s/8mm/foot_z/stratified chain and its FAIL diagnostic blamed
   precisely the primitive ref's torque-infeasible flat posture),
   warm from `stancemix_bcchain3_stdanneal`, std pinned -4 per the
   slowchain precedent (noise re-injection previously exonerated).
   Gate: flat probe genuine non-freeze tuck both seeds + hold
   >=5/6+5/6 zero-term + lower >=5/6 honest -> fund 8M; freeze/pin or
   hold/lower regression -> next lever is re-opened std, not budget.
   Rise residuals for later hardening: presses ride the current pin
   (2.37-2.64A) without tripping; bridge/rsi OC tail (parent-shared);
   own-DR (s0 10/12, s1 5/6+4/6); det leg-idx2 micro-swing asymmetry
   at no stability cost. SKILLS.md row added (seed-1 scoped).

2. `holdheight-rung2-hha1-s1` (2M canary, seed 1) **CANARY PASS** +
   **RUNG-2 JOINT CALL: PASS, PROCEED.** Seed-1: DR-0 12/12
   valid_plant, ZERO min-load terms, h_err_end 0.2-3.2mm, det duty
   1.0 all legs, det cur_max 0.62-0.76A (inside the rung-1 band);
   trace residual 1/6 sto ep at duty 0.89 — smaller than seed-0's 2/6
   at 0.87/0.89; cross-rung the cheat magnitude SHRANK (rung-1 s1:
   6/12 terms, floor 0.57). FALLBACK RULING (assume-and-go, in the
   verdict): the S-gate/min-load-pricing fallback does NOT fire on
   shrinking zero-term trace dips; instead **rung-3 carries a
   registered TRIPWIRE** — any per-leg duty <0.85 OR any
   hold_min_load term at DR-0 in either seed fires it immediately.
   **Rung-3 pair launched:** `holdheight-rung3-hha1`/`-s1`
   (train-2/3, VERIFIED RUNNING, 2M each) — full kind mix
   `["hold","ramp","sine","pulse"]` at 15mm/s (pure cfg sweep, all
   four kinds already in goal_task.py), warm from the rung-2 SEED-1
   ckpt (cleaner of the pair), gate = rung-2 gate + tripwire, joint
   pair. Own-DR hardening stays open (one det min-load term per seed
   at rung-2, better than rung-1's 2/6).

Fleet: 4 launches this cycle (cap), 4 pods busy training
(train-2/3/4/5), watcher owns their checkups. Next decision points:
rung-3 joint read (tripwire!), stancemix-tuckclock joint read (fund
8M vs re-open std), then STAND_HEIGHT rungs 4-5 unblock via the now-
solved rise. Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
holdheight_rung2_hha1_s1_{gate,owncfg}/`, `..._riseonly_bcchain3_
meshref_tuckclock_acq8m_s1_{gate,owncfg,flatprobe}/`, W&B `o8aq5c1a`
/ `xmkbyuuq`.)

Prior entry: 2026-08-25 ~23:0x (**FLAT-START RISE SOLVED on seed 0 —
`tuckclock-acq8m` PASS: the campaign-wide flat-start blocker is
broken. Plus rung-2 height-elevator seed 0 PASS with a trace caveat.**
This cycle verdicted its two assigned seed-0 runs; BOTH registered
joint calls land with their `-s1` twins' cycles:

1. `cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock-acq8m`
   (8M, seed 0) **PASS** — the robot stands up from lying splayed flat
   on the 3.5 kg mesh model. Flat-pinned pod probe
   (`goal.rise_flat_frac=1.0/partial=0/rsi=0`, DR-0, n=6+6): **12/12
   valid_plant, h_err_end 0.3–3.8 mm** vs the 2M canary's 0/12 at
   37.7–62.0 mm; roll peak/tail 1.0/0.8°, settled 12/12, zero
   falls/terms; video = genuine splay→tuck-under→level six-foot plant
   held to end. Non-flat standard DR-0 gate det 6/6 + sto 4/6 ≥
   meshref parent (5/6+4/6) — both gate clauses met for this seed.
   Residuals: cur_max still kisses 2.46–2.64 A in the tuck/press
   phase (structural ceiling, now non-terminal at DR-0); own-DR 0.2 =
   10/12 with 2 over_current rsi terms; one sto rsi fell. Reward rose
   all run (-15.4/-149.5/548.1/1290.9). **NEXT (for the `-acq8m-s1`
   cycle — s1 FINISHED its full 8M cleanly ~22:3x, its in-run final
   eval already showed rise f2/2 b2/2 c2/2; the watcher SUSPECT on it
   was the final W&B flush, benign):** run/read s1's flat probe, make
   the registered joint call; on joint PASS promote
   `train.bc_anchor_flat_time_indexed=1` + `rise_ref_mesh_scripted.npz`
   as THE mesh rise recipe and port into stancemix (warm-start from
   `ppo_goal_..._stancemix_bcchain3_stdanneal.zip` per the slowchain
   precedent, or per that arm's outcome). SKILLS.md row added.
   Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_
   bcchain3_meshref_tuckclock_acq8m_{gate,owncfg,flatprobe}/`, W&B
   h2zqn1ev. This also supersedes the tuckexempt-i0 dig-in's
   "SURVIVING DESIGN" note below — the script-progress floor redesign
   is MOOT if the joint call passes (tuckclock already tracks through
   the tuck).

2. `cw-standwalk-stance-mesh2-holdheight-rung2-hha1` (2M canary, seed
   0, full [-40,20] mm height range) **PASS with a letter-breach
   caveat** — DR-0 12/12 valid_plant, ZERO hold_min_load terms,
   h_err_end 0.2–3.7 mm, det duty all ≥0.98, det cur_max 0.75–1.06 A
   (band-adjacent to rung-1's 0.66–1.03 A), video level/planted.
   CAVEAT: 2/6 sto episodes lighten leg idx5 to duty 0.89/0.87
   (<0.9 gate letter) with 27–28 micro-swings, sto cur_max to 1.38 A
   — same lightening family as rung-1 seed 1, at trace magnitude,
   zero terms, sub-visual on video. Own-DR 0.2: 11/12, one det
   hold_min_load term — DR hardening still open. **NEXT (for the
   `-rung2-hha1-s1` cycle, still training):** registered joint call;
   recommendation PROCEED if s1 is comparably clean, but the
   pre-registered S-gate/min-load-pricing fallback condition ("EITHER
   seed reproduces min-load dips") is arguably met at trace level —
   fire it at the next rung if dips grow instead of shrink. Evidence:
   `logs/ckpt_eval/cw_standwalk_stance_mesh2_holdheight_rung2_hha1_
   {gate,owncfg}/`, W&B nngk147g.

No new launches this cycle: both refills are the twins' registered
joint calls (s1 runs finished/finishing, cycles imminent); amp DONE
(sim), cpg closed, walkcurr [operator]-blocked — no other runnable
topmost work.)

Prior entry: 2026-08-25 ~22:2x (**STAND_HEIGHT rung-1 JOINT CALL:
PROCEED — height-aware anchor proven cross-seed; rung-2 pair
LAUNCHED.** This cycle verdicted `holdheight-rung1-hha1-s1` (seed 1):
CANARY PASS - MECHANISM with a residual caveat — the anchor fix
rescued this seed from the gross flag-leg cheat too (12/12 DR-0
valid_plant, height_err_end 0.1–3.0mm, det Imax 0.62–0.94A, video
level/planted/quiet, 6/12 episodes fully honest all-duty-1.0 holds)
but NOT completely: 3/6 det + 3/6 sto still trip `hold_min_load` via
a SUBTLE leg-5 load-lightening (duty dips only to 0.57–0.9, 4–30
micro-swings, terminated episodes only; pre-fix was leg-2 duty
0.23–0.70, 10/12 terms). Warning sign: reward rose all run
(15.7/48.3/71.0/123.3) while `env/hold_load_factor` drifted DOWN late
(0.99→0.89 at locked std 0.018) — the optimizer slowly re-buying
partial unloading on this seed, so extending seed 1 is the wrong
move (08-22: misaligned residual, not undertrained). REGISTERED
JOINT PASS-RATE CALL (landed here): mechanism PROVEN cross-seed
(seed 0 fully clean 0/12 terms; seed 1 cheat magnitude collapsed),
strict zero-term criterion 1/2 → PROCEED up the ladder from the
CLEAN checkpoint. Launched rung-2 canary pair (full `[-40,20]`mm
range, hold+ramp, everything else identical, warm-start = rung-1
seed-0 ckpt `ppo_goal_..._rung1_hha1.zip`, hha=1 stays recipe
default): `cw-standwalk-stance-mesh2-holdheight-rung2-hha1`
(train-2) / `-s1` (train-3), both VERIFIED RUNNING, 2M canary each,
gate = rung-1 gate + no per-leg duty <0.9 + cur_max within rung-1
seed-0's 0.66–1.03A band, judged as a joint pair. If EITHER seed
reproduces min-load dips, the registered S-gate/min-load-pricing
fallback fires (also covers rung-1 seed 1's residual). Own-DR
hardening remains open on both rung-1 seeds (seed 0: 2/6 det trips
at DR 0.2). Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
holdheight_rung1_hha1_s1_{gate,owncfg}/`, W&B `gfksq1nx`.)

Prior entry: 2026-08-25 ~22:1x (**RISE, TUCK DIG-IN PAYS OFF —
`tuckclock1`/`-s1` CANARY PASS (mechanism-health; behavioral score
PARTIAL). The flat-time-indexed BC-anchor clock breaks the total-
freeze/press-up basin that killed the prior 5 anchor-plumbing arms.**
Triaged both seeds this cycle (seed-0's own prestage fired normally;
seed-1's never fired — checkpoint pulled + gate/owncfg/flatprobe run
by hand on train-1). Flat-pinned probe (det+sto n=6+6, DR-0, on-pod,
both seeds): 0/12 valid_plant each (h_err seed-0 37.7–59.3mm, seed-1
57.5–72.9mm, still short of the 79–87mm target) BUT for the first
time genuine six-leg motion in EVERY episode — duty>0 AND
swing_count>0 on every leg (vs the freeze family's byte-identical
duty=0/swing=0, and the meshref parent's own flat draw: duty=0.67
but swing=0, i.e. never lifts a leg). Video confirms a genuine
splay→tuck-under→partial-stand. Non-flat kinds (bridge/crouch/rsi)
hold AT/ABOVE the meshref parent on the standard DR-0 gate both seeds
(seed-0 5/6+5/6, seed-1 4/6+4/6, vs parent 5/6+4/6). Current still
pegs ~2.64A on nearly every episode — the same structural ceiling as
every prior arm, not new. SEED DIVERGENCE worth tracking: seed-0
lands in a stable-incomplete basin (zero falls, settled 3/6 det +
6/6 sto); seed-1 lands in an unstable-flailing one (11/12 flat
episodes end roll_class fell/leaning, one leg — idx1 — swinging
2–10×/episode while 3–4 others swing 0–2×). Both are the run's own
pre-registered PARTIAL branch (and the hypothesis's own named
"strongest alternative": clock tracked in action space but physics
lags on the 3.5kg mesh model) — NOT the FAIL branch's freeze/press-up
signature, so per the 08-21 ruling the reasoned move is extend
budget before touching any anchor knob, mirroring the meshref
parent's own 2M→8M acq8m precedent. LAUNCHED the pre-registered 8M
budget-extension pair, exact recipe unchanged, from scratch:
`cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock-acq8m`
(seed 0, train-0) / `-acq8m-s1` (seed 1, train-1), both VERIFIED
RUNNING. Gate: flat-probe valid_plant convergence (or at least
shrinking h_err / dropping seed-1 fall-rate) vs the 2M canary's own
baseline, non-flat still ≥parent. FAIL route (budget-invariant h_err/
fall-rate) points at a per-leg stability/symmetry term on the
flat-time-indexed target, not more budget. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_tuckclock1{,_s1}_
{gate,owncfg,flatprobe,flatprobe_det,flatprobe_sto}/`, W&B `3otxc8w4`/
`atmjubij`.)

Prior entry: 2026-08-25 ~22:0x (**STAND_HEIGHT height-AWARE anchor
rung-1 canary: seed 0 PASSES — flag-leg cheat GONE.**
`holdheight-rung1-hha1` (this cycle's assigned run) CANARY
PASS - MECHANISM: with `train.bc_anchor_hold_height_aware=1` +
`bc_anchor_coef=3.0` restored, the robot rides the ±15mm/8mm/s
commanded-height elevator on a clean six-foot stand. DR-0 gate 6/6
det + 6/6 sto valid_plant (needed 5/6+4/6), ZERO `hold_min_load`
terminations (parent recipe: 8/12), height_err_end 0.0–2.6mm, all six
legs duty 0.91–1.0 (no per-leg sacrifice; parent flagged a leg at
0.23–0.86), det Imax 0.66–1.03A vs the 2.0–2.63A failure signature —
modestly above the champion's static 0.67–0.71A, plausibly the honest
cost of a moving command. Video: level, planted, quiet; reward rose
all run (12.9/28.6/74.2/105.8). CAVEAT (outside the registered
canary gate): at own-DR 0.2, 2/6 det episodes still trip
`hold_min_load` with transient duty dips (0.4–0.66) and 2.5A spikes —
DR hardening is real remaining work for later rungs. The registered
JOINT pass-rate call belongs to `-hha1-s1`'s own cycle (it finished
at 2.03M, W&B `gfksq1nx`, reward also rising 15.7/48.3/71.0/123.3 —
evals prestaged); on joint PASS, the next rung of the STAND_HEIGHT
ladder launches with `bc_anchor_hold_height_aware=1` as the recipe
default. Watcher SUSPECT on `-hha1` was a false alarm (fired during
final flush; clean finish at 2.03M). Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_holdheight_rung1_hha1_{gate,owncfg}/`,
W&B `isxy1d5b`.)

Prior entry: 2026-08-25 ~21:4x (**STAND_HEIGHT height-AWARE BC-anchor
fix LANDED + its own rung-1 canary pair LAUNCHED — the exact NEXT
step the entry below (seed-0 cycle) named.** Verdicted this cycle's
own assigned run, `holdheight-rung1-s1`: CANARY FAIL - MECHANISM,
cross-verifying the seed-0 read below in full including the SAME
flag-leg fingerprint — leg index 2 duty_cycle 0.23–0.70 with 4–55
swings/episode (vs 0.88–1.0 and 0–5 on every other leg), 5/6 det +
5/6 sto `hold_min_load` terminations. Built + tested the fix the FAIL
branch pre-registered: `train.bc_anchor_hold_height_aware` (default
0 = legacy constant-`q_nom` target, bit-exact) in the HOLD/TRACK
BC-anchor block, `rl_move/sim/sim_env.py` — re-targets the anchor at
the `FixedFootBodyIK` pose that reaches the NEXT commanded height
(the same one-tick-ahead convention `train.bc_anchor_lower` already
uses) instead of the height-blind constant pose, gated to `mode ==
"hold"` and nonzero commanded height. 5 new tests in
`test_bc_anchor.py` (default-off bit-exact, zero-height no-op,
targets-the-commanded-height via IK-solve equality, track-mode-
excluded, feet-planted-descent chain) — all green; full
`test_bc_anchor.py` + `test_hold_height_cmd.py` green (84/84);
`STAND_HEIGHT.md`'s own preflight bank re-run green (4/4, unaffected
— confirmed the 10 red `test_task_semantics.py` walkcurr/hold-bank
tests hit while scoping this are pre-existing per RL_LOG 08-25 09:5x,
not caused by this change). Launched the fix canary pair — exact
`holdheight-rung1`/`-s1` recipe, only `train.bc_anchor_coef` restored
0.0->3.0 (the champion's own value) plus
`train.bc_anchor_hold_height_aware=1.0` added:
`cw-standwalk-stance-mesh2-holdheight-rung1-hha1` (train-0, W&B
`isxy1d5b`) / `-hha1-s1` (train-1, W&B `gfksq1nx`), both VERIFIED
RUNNING. Gate: DR-0 det>=5/6 + sto>=4/6 valid_plant, zero
`hold_min_load` terminations, `cur_max`/duty back within noise of the
champion's clean band (no flag-leg signature) — judged jointly as a
pass-rate pair against `holdheight-rung1`/`-s1`'s own numbers. On
FAIL at the same flag-leg signature even with the anchor restored:
the S-gate itself needs strengthening (the bank already ranks honest
6-foot tracking >2x above a flag but PPO's exploration never reaches
it from this init) — that is the registered fallback, and a genuine
dig-in. Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
holdheight_rung1_s1_{gate,owncfg}/`, W&B `roplbk1z`.)

Prior entry: 2026-08-25 ~21:2x (**STAND_HEIGHT RUNG-1 CANARY FAIL —
CROSS-SEED FLAG-LEG CHEAT.** `holdheight-rung1` (verdicted this cycle)
and its seed twin `-s1` (gate report peeked for the registered joint
pass-rate judgment; verdict belongs to its own cycle) both learn the
±15mm/8mm/s commanded-height elevator — height_err_end 0.3–8.8mm on
terminated episodes — but pay for it by FLAGGING ONE LEG: seed 0
unloads/waves leg 4 (duty 0.29–0.86, swing 3–6 det), seed 1 the SAME
cheat on leg 2 (duty 0.23–0.70). Different flagged leg per seed =
recipe-level exploration basin, not seed luck. DR-0 gate 0/6 det +
0/6 sto both seeds (needed 5/6+4/6, zero hold_min_load; actual 8/12
hold_min_load terms seed 0). Reward rose all run (quarters
15.6/11.9/34.4/75.7) and training min-load terms decayed 120→1-2/
window → 08-21 ruling: a basin/incentive problem to repair. The bank
ranks honest 6-foot tracking >2x above a flag, PPO just never finds
honest tracking from this init (hold_load_factor plateaus ~0.81 = a
~19% income haircut it happily eats; std annealed to 0.018 locks it
in). NEXT per the run's own pre-registered FAIL branch +
STAND_HEIGHT.md's height-blind-anchor note: implement the
height-AWARE hold BC anchor — IK-solved at the CURRENT commanded
height every tick, the exact mechanism `train.bc_anchor_lower`
already uses via `BodyOffset(height=g_next.height_ref)` — new cfg,
default off/bit-exact, bank rows + unit tests, then relaunch the
2-seed rung-1 canary. S-gate strengthening is the registered
fallback. Reward/env code → DIG-IN flagged. Also this cycle: the
watcher's SUSPECT on `holdheight-rung1-s1` was a false alarm (run
finished cleanly at 2.03M, W&B synced; checkup fired during the final
flush). Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_
holdheight_rung1{,_s1}_{gate,owncfg}/`, W&B `tp53jcv8`.)

Prior entry: 2026-08-25 ~21:5x (**TUCK DIG-IN RESOLVED WITH NUMBERS —
the reward was never misaligned; the honest TIMING was never taught.
New lever launched: `train.bc_anchor_flat_time_indexed`.** This cycle
picked up the 21:0x DIG-IN flag and ran the pricing question through
`probe_stance_pricing` (extended: `--ref` for the mesh ref, per-part
sums, and a new `replay_script` behavior — teacher-timed absolute-
clock replay; probe artifacts `logs/probe_tuck_pricing_*.json`).
FINDINGS: (1) under the LAUNCHED pricing (hot1a2+term3) every
reachable behavior is deeply negative — freeze -704, ramp-aligned
replay -706 WITH an over_current trip, learned policy -770 — and NO
hot dose fixes the ordering (grid a=2.2-2.55: stilt stays the paid
optimum via +385 ref-kernel income). (2) BUT the mesh scripted ref
replayed on ITS OWN CLOCK from flat earns **+2021, plant_ok, Imax
0.575A, zero over_current, under the exact launched pricing** — the
honest tuck sweep makes the press nearly effortless; the 2.64A
press-up was never necessary. The reward's optimum IS the gate
behavior; nothing in training ever EXECUTED that timing: state-aligned
pursuit stalls with the robot (freeze converges to low anchor loss),
and the legacy ramp-aligned clock starts ~2s INTO the 2.45s tuck
(env `rise_hold_min_s=0.5` vs ref `ramp_i0=245`), slamming the press —
that is the entire 15-arm anchor-plumbing campaign explained. (3)
Bonus bug found+documented: `reward.rise_score_strip_pen=1` is a
NO-OP as implemented (strips only `reward_task`, which is never
negative; the documented k_height-penalty strip never happened) — no
repair needed now since the probe shows ordering is correct with the
penalty live, but the code comment is wrong; noted so nobody
re-derives pricing from it. FIX (landed, snapshotted): sim_env
flat-start absolute-script-clock anchor targets — new cfg
`train.bc_anchor_flat_time_indexed` (default 0 = bit-exact; 3 new
unit tests, 65/65 test_bc_anchor green): pure-flat non-RSI rise
episodes anchor to ref row `round(step*dt/ref_dt)+1` (the target
advances by itself, so freeze ACCUMULATES loss instead of converging
on it); partial/crouch/rsi/bank keep state-aligned pursuit untouched.
Chained end-to-end on the launch-exact mesh stack: +2029 return,
81.5mm stand on a 79mm target, 0.58A, plant_ok=True. Launched the
pre-registered 2M canary pair `...meshref-tuckclock1`/`-s1` (single
lever vs the meshref parent — NO tuck_exempt/tuck_lookahead, those
are refuted).)

Prior entry: 2026-08-25 ~21:0x (**SCRIPT-INDEX FLOOR AXIS CLOSED —
`tucklook1`/`-s1` CANARY FAIL-MECHANISM, both seeds, cross-verified.
This cycle picked up the 19:5x/20:1x DIG-IN flag (idle fleet, no other
track had fundable work) and implemented the surviving design named
there: `train.bc_anchor_tuck_lookahead_s` (new cfg, default 0=off/
bit-exact, 3 new unit tests `test_bc_anchor.py`, 62/62 green,
snapshot `6014ed69`) — while the state-aligned match is still inside
the tuck (`< ramp_i0`), widen the pursuit lookahead itself to a fixed
1.25s SCRIPT-INDEX offset from the CURRENT match (never from achieved
height, so unlike the height-floor it cannot get stuck measuring a
frozen height) — composed with `tuck_exempt_i0=1` (floor off in-tuck,
unchanged) on the meshref recipe. RESULT: no better than plain
tuck_exempt alone. Flat-pinned probe on BOTH seeds: 0/6 det valid,
byte-IDENTICAL height_err_end (79.3/83.9/85.5/84.8/81.4/84.4mm) and
byte-identical curmax (0.53A)/duty(0)/swing(0) to tuckexempt0's own
flat probe — the widened lookahead changed NOTHING measurable about
the freeze. WORSE: the standard DR-0 gate, clean on the meshref parent
(det 5/6 + sto 4/6), collapsed to 0/6 det + 0/6 sto on BOTH seeds
(own-DR replicates on s1) — bridge/crouch/rsi starts that used to
plant cleanly now fail too (freeze-adjacent height misses or
over_current pins), so the lever also broke press-phase starts the
tuck-scoping was supposed to leave untouched. Video: same one-instant
splayed->tucked-under-body snap-fold as every prior freeze, then
total static hold, never a stand. Reward quarters both seeds decline
the same shape ([12.7,4.8,0.1,-61.1] / [14.2,4.6,-0.0,-46.3]) —
recipe-consistent, not seed noise. READ: either 1.25s (~25% of the
4.9s tuck) still under-doses the pose delta, or non-flat starts
transiently state-align into the tuck-index range too and the
widened target teaches an overshoot that destabilizes them, or the
near-collapsed policy std (0.018 by this point in the anneal) means no
anchor-target change escapes whichever basin PPO fell into early
regardless of magnitude — any of these makes "more bc_anchor
lookahead/floor plumbing" a bad next bet. **The anchor-mechanism axis
(pace, budget, dose, height-floor, tuck-exempt, script-index-floor —
every named lever) is now EXHAUSTED across ~15 canary arms this
campaign.** Per this run's own pre-registered FAIL branch and the
tuckexempt0 dig-in's fallback: the next lever is a direct TUCK-PHASE
REWARD/CURRICULUM term (price genuine sweep-to-footprint progress
directly, e.g. a foot-to-plant-footprint distance income scoped to
the tuck segment) rather than any further BC-anchor pursuit-shaping —
DIG-IN flagged (reward-semantics design + bank rows, not a quick
triage patch). Also folded in this cycle: the concurrent cycle's
`tuckrise45`/`-s1` verdicts (CANARY FAIL-MECHANISM, killed early,
ref-content axis refuted by construction) are corroborated by an
independent flat-pinned probe this cycle ran on the completed
`tuckrise45` checkpoint before reading that verdict: 2/6 det valid,
but both "valid" episodes are the same snap/drag-fold-to-tuck-pose
(swing_count=0 on every episode, all six legs, `drag_mm`>0) landing
AT the tuck/press boundary rather than genuine mid-tuck progress —
same conclusion, independent evidence, no re-verdict needed (already
recorded by the concurrent cycle). Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_tucklook1{,_s1}_
{gate,owncfg,flatprobe_det,flatprobe_sto}/`, W&B `6ewwghw3`/`zxnx3fb3`.)

Prior entry: 2026-08-25 ~20:1x (**MECHANICAL/CORROBORATING NOTE — this
cycle's own read independently reaches the same conclusion as the
19:5x entry below, plus one caught infra bug and one self-caught
redundant launch.** (1) Triaged the assigned run `tuckexempt0` (this
cycle's target) + its unclaimed seed twin `tuckexempt0-s1`: BOTH
CANARY FAIL - MECHANISM, byte-for-byte the tuckfloor0 total-freeze
signature (flat-pinned probe 0/24 combined valid_plant, duty=0.0 all
six legs, standard gate 0/6+0/6 both seeds, worse than the meshref
parent's 5/6+4/6) — closes the tuck-exempt-floor axis (ledger race:
the concurrent cycle also wrote a verdict on `-s1` ~1 min later,
same conclusion, its text is what persisted; no information lost,
RL_LOG keeps both lines). (2) Caught a genuine infra bug: a sibling
`tuckrise15` launch (concurrent cycle, in flight when this cycle
read the ledger) crashed immediately — `reward.rise_ref_path` pointed
at `rise_ref_mesh_tuckrise15.npz`, which was generated but never
committed (only `rise_ref_mesh_tuckrise45.npz` landed in snapshot
`fa1bccc5`); `MjxShardedVecEnv` worker died with `FileNotFoundError`
before any step, 0 training steps, W&B `58ohqh56`. Verdicted CANARY
FAIL - INFRASTRUCTURE. (3) Not yet having read this file's own 19:5x
entry (written concurrently), independently launched
`tuckrise45`/`-s1` (2M canary pair, the properly-committed 45mm ref,
single lever vs meshref parent) — then, on reading 19:5x below,
recognized it was redundant: the concurrent cycle had ALREADY probed
`--tuck-rise-mm` at 3 CPU-only doses and found achieved height never
lands inside the tuck on the 3.5kg mesh model at ANY dose/split
(swing-lift fights the rise, ~25-30mm compliance dead-band, ~1.5s
lag). This run's OWN reference file confirms it directly:
`h_rel_m` in `rise_ref_mesh_tuckrise45.npz` stays exactly 0.0 through
tick ~245 and only starts climbing at tick ~250 (achieved
`ramp_i0=258`, barely later than meshref's own 245) — the "45mm
across the second half of the tuck" lands AT the tuck/press boundary,
not inside the tuck. Killed both seeds early (~1.0-2.0M/2M steps,
train-0/train-1) rather than let a doomed canary finish; verdicted
CANARY FAIL - MECHANISM citing the concurrent cycle's own
construction-level refutation + this run's own h_rel_m evidence.
**Net: ref-content height-shaping is now refuted by TWO independent
lines of evidence (CPU-probe construction argument + this GPU
canary's own achieved-height trace) — do not fund another
`--tuck-rise-mm` dose.** The only surviving design per the 19:5x
entry — keying the BC-anchor floor's progress metric off the
SCRIPT-INDEX or COMMANDED height instead of ACHIEVED height — is
correctly DIG-IN flagged for a deep-model cycle (bc_anchor hot-path
code + bank rows + a canary is more than a triage-cycle change).
Re-checked all other tracks for spare-capacity refill (unchanged
since 19:2x): joystick DONE-gate met (deferred hardening explicitly
lives here, and is itself the same lesson, not a separate lever);
amp/cpg DONE/maintenance; walkcurr self-blocked on `[operator]`.
12/12 pods correctly idle at cycle end — genuinely nothing fundable
without duplicating in-flight work or jumping ahead of the DIG-IN
flag. Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_
bcchain3_meshref_tuckexempt0{,_s1}_{gate,owncfg,flatprobe_det,
flatprobe_sto}/`, W&B `46px3v47`/`zigatklw`/`58ohqh56`/`nt6ot7gn`/
`wbhsu4st`.)

Prior entry: 2026-08-25 ~19:5x (SUPERSEDES the entry directly below:
by the time this note landed the concurrent cycle had already gone
further — probed `--tuck-rise-mm` with 3 controlled CPU mints (no GPU
spent) and REFUTED ref-content editing by construction (on the 3.5kg
mesh model, no feasible tuck-segment content places ACHIEVED height
inside the tuck: swing-lift fights the rise, then a ~25-30mm
compliance dead-band, then a ~1.5s lag — height always lands in the
press regardless of dose/split), and pre-refuted the mid-tuck-
curriculum alternative too (rsi already samples tuck ticks uniformly
and those starts already press-pin in the parent). Anchor-floor
plumbing AND ref-content AND mid-tuck curriculum are now ALL closed;
the only surviving design is changing the anchor floor's PROGRESS
METRIC itself (script-index or commanded-height keyed, not achieved-
height keyed) — bc_anchor hot-path code + bank rows + a canary, and
correctly **DIG-IN flagged for the deep cycle** rather than being
rushed here. Full detail in `## Now` below (19:5x entry, W&B
j9k3h490); do not restart the ref-content axis, it is refuted, not
just "in flight.")

Prior entry: 2026-08-25 ~19:4x (**TUCKEXEMPT0/-S1 JOINT PAIR CLOSED —
CANARY FAIL - MECHANISM, cross-verified both seeds; anchor-floor-
scoping axis is now FULLY CLOSED.** This cycle read+verdicted the
seed-1 twin (`tuckexempt0-s1`): flat-pinned probe (det+sto n=6+6,
DR-0, run this cycle) 0/12 valid_plant, all 12 duty_cycle=0.0 on all
six legs (zero swings, Imax 0.53-0.54A) — byte-for-byte the same
total-freeze signature as `tuckexempt0` (seed-0, verdicted by the
concurrent cycle) and `tuckfloor0`/-s1 before it; standard DR-0 gate
also 0/6 det + 0/6 sto + own-DR(0.2) 0/6+0/6, WORSE than the meshref
parent's 5/6+4/6, triggering the gate's ALSO-FAIL clause too. MECHANISM
(agreed independently by both cycles): `rise_ref_mesh_scripted.npz`'s
own tuck segment holds height pinned at ~0mm for ~245 ticks, so the
state-aligned lookahead target is ALSO ~0mm there — removing the
height-floor (globally, `tuckfloor0`, or tuck-scoped,
`tuckexempt`/`tuckexempt0`) leaves near-zero pursuit gradient through
the tuck either way, so the policy settles into the reward-cheapest
freeze instead of tucking. Anchor-floor plumbing (dose, scope, removal)
is now fully refuted as a lever; the fix has to change WHAT the tuck
segment targets. **Already in flight (uncommitted, same clone — do not
duplicate):** `make_rise_ref_scripted.py` has a new `--tuck-rise-mm`
lever (raises the chassis smoothly across the second half of the tuck
so the reference's height profile is monotone, not flat) and a first
mint `rise_ref_mesh_tuckrise15.npz` already exists on disk — this is
the pre-registered next rung; whichever cycle is mid-flight on it owns
landing+launching the canary, not a second parallel attempt. MECHANICAL
NOTE: a benign double-verdict race also occurred on `tuckexempt0-s1` —
the concurrent cycle wrote its own (agreeing) verdict ~19:46, this
cycle's `ops.sh verdict` overwrote the ledger's single entry ~19:47
with a fuller version citing the same evidence/conclusion; RL_LOG.md
keeps both lines (append-only), no information lost, no re-verdict
needed. Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_
bcchain3_meshref_tuckexempt0{,_s1}_{gate,owncfg,flatprobe_det,
flatprobe_sto}/`, W&B `46px3v47` / `zigatklw`.)

Prior entry: 2026-08-25 ~19:2x (MECHANICAL NOTE, no science change:
this cycle's assigned triage target, `...-meshref-tuckfloor0`, was
already fully verdicted (CANARY FAIL - MECHANISM, see the 19:0x entry
below) and its fix already coded/tested/snapshotted by the time this
cycle read the ledger — but a race with the concurrent cycle meant
BOTH cycles independently believed the pre-registered tuck-exempt
canary pair had not yet landed and both launched it: the concurrent
cycle's `...-meshref-tuckexempt0`/`-s1` (train-0/train-2, created
19:08:39/19:14:37) is the pair of record (seed-0 already FINISHED,
awaiting its own triage cycle; seed-1 still running). This cycle's
own copy `...-meshref-tuckexempt-i0`/`-i0-s1` (train-1/train-3,
created ~2 min later, bit-identical cfg) was recognized as the
duplicate, killed cleanly on both pods, and verdicted CANARY FAIL -
INFRASTRUCTURE (no information lost, no pod time wasted beyond ~10
min). No other standwalk lever is fundable right now without
duplicating in-flight work: rise is the sole open rung and its only
live mechanism (tuck-exempt floor) is already running under the
concurrent cycle's names; hold/lower are PASSed; the footlow2raw18
warmmix pair and stancemix-slowchain are both already verdicted FAIL.
Other tracks checked for spare-capacity refill: joystick's DONE gate
is already met (08-23, `stotight45-seed13`) with further mesh/100Hz
hardening explicitly deferred to this track per its own 08-25 ~12:1x
entry; amp and cpg are both DONE/maintenance-only; walkcurr is
deliberately self-blocked pending an `[operator]` ruling per its own
founding rule (not idle-capacity-actionable). Net: 11 of 12 pods
correctly idle this cycle — genuinely nothing else is runnable
without duplication or violating another track's own blocking rule.)

Prior entry: 2026-08-25 ~19:0x (**tuckfloor0/-s1 JOINT PAIR CANARY
FAIL - MECHANISM — removing the BC-anchor height-floor everywhere
does NOT teach the tuck; it breaks pursuit broadly. Coded + tested +
launched the pre-registered fix: a TUCK-EXEMPT floor.** Both seeds
cross-verify: flat-pinned probe (det+sto n=6+6, DR-0) 0/12 valid,
0/12 over_current (Imax 0.52-0.56A, well under the 2.64A pin) — but
duty_cycle=0.0 on all six legs and swing_count=0 for the FULL 15s
episode, height_err stuck 79-86mm. Contact sheets: one instant snap
from the splayed spawn to a folded pose, then total static freeze —
not the hoped tuck-then-press. Worse, the standard DR-0 gate that was
clean on the meshref parent (det 5/6 + sto 4/6 valid) collapsed to
0/6 det + 0/6 sto on BOTH seeds: bridge/crouch/rsi starts now show a
mix of the SAME 2.64A press-up pins plus brand-new duty=0 freezes on
starts that used to work. own-DR(0.2) replicates. Training reward
DECLINED across quarters both seeds (~[13,-0.3,-20,-46]) — a genuine
FAIL per the 08-21 ruling, not misaligned-continue. This lands exactly
on the pair's own pre-registered alternate FAIL branch: non-flat kinds
collapsing into freezes means the floor's press-phase anti-freeze role
(its original 08-12 purpose) was load-bearing, and removing it
everywhere threw that out along with its flat-segment defect. FIX
(this cycle): new cfg `train.bc_anchor_min_h_tuck_exempt_i0` (default
0 = legacy, bit-exact) in `sim_env.py`'s rise state-aligned BC-anchor
block — gates the height-floor OFF only while the state-aligned
matched index is still inside the reference's OWN tuck segment
(`ref["ramp_i0"]`, a fixed file property), restoring the exact legacy
floor unchanged once the match reaches/passes ramp_i0. 4 new unit
tests (`test_tuck_exempt_*` in `test_bc_anchor.py`), 59/59 green,
snapshot `exp/cw-standwalk-tuckexempt-i0`. Launched the 2-seed 2M
canary pair `tuckexempt0`/`-s1` (exact tuckfloor0/meshref recipe,
`min_h_ahead_mm` restored 0->8 + `min_h_tuck_exempt_i0=1`) — VERIFIED
RUNNING on train-0/train-2; tuckexempt0 (seed 0) already finished and
has its own triage cycle spawned (own it there, do not re-triage).
Evidence: `logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_
meshref_tuckfloor0{,_s1}_{gate,owncfg,flatprobe_det,flatprobe_sto}/`,
W&B `2t8z8o4i` / `dvwfnqqm`.)

Prior entry: 2026-08-25 ~18:1x (**FLAT-START ROOT CAUSE FOUND — the
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

Addendum 2026-08-25 ~18:2x (seed-0 verdict recorded — the exposure
pair is now CLOSED, both seeds CANARY FAIL-MECHANISM): `flatmix70`
(seed 0) flat-pinned probe 0/12 valid, all over_current pinned exactly
2.64A, zero swings, h_err_end 24-70mm (`logs/ckpt_eval/
flatmix70_fpin_{det,sto}/`, W&B jkpuza6s); standard DR-0 gate det 0/6
(4 flat + both rsi in the press-up basin) / sto 4/6, episode-for-
episode the seed-1 story. The two triage cycles independently
converged on the same measured root cause (anchor height-floor skips
the height-flat tuck ticks 0-245) — treat it as cross-verified. Both
arms of the funded fix pair are confirmed running with the single
lever verified in their live commands (`min_h_ahead_mm=0`, lookahead
0.25 unchanged): `tuckfloor0` train-0 (VERIFIED RUNNING, W&B 2t8z8o4i,
launched by the seed-0 cycle) + `tuckfloor0-s1` train-1 (launched by
the seed-1 cycle). Joint pair gate as registered in both run docs.)

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

**SEGFIX JOINT CALL: REAL DIVERGENCE, 08-26 ~09:1x — the composed
mode_seq segment-window widen (6-8s->9-11s) FIXES seed1's severe
flat-in-composition collapse (2/12->12/12 valid, matched flat-pinned
composed probe) but REGRESSES seed0 (12/12->9/12, 3 NEW
hold_low_height terms) on the identical config change.** Full
before/after numbers, video confirmation, and the recommendation
(do NOT promote the widened window; `acq8m`/`acq8m-s1` stay the
standing stage-1 checkpoints pending root-cause) are in the
Last-updated banner at the top of this file — not duplicated here.
DIG-IN flagged for the seed0 regression's root cause. Own-scope
verdict (`-segfix-s1`) recorded; `-segfix` (seed0) belongs to a
concurrent cycle. SKILLS.md row added. Evidence: `logs/ckpt_eval/
cw_standwalk_stance_mesh2_standheight_rung5_acq8m_segfix_s1_{gate,
owncfg,seqprobe,seqprobe_flat}/`, `..._segfix_{seqprobe,
seqprobe_flat}/` (sibling, cited), `..._acq8m_seqprobe_flat/` +
`..._acq8m_s1_seqprobe_flat/` (matched baselines). W&B `i50v00p9`.

Prior entry, 08-26 ~07:5x (**STAND_HEIGHT RUNG-5 JOINT CALL CLOSED —
the composed lower fix holds cross-seed under the CORRECTED motor
contract; rung-5 is closed for its own question, flat-start rise
carries forward as the pre-existing open residual.** Closing the joint call the prior
entry left open: `-s1`'s own gate/owncfg/seqprobe were confirmed
genuinely pre-fix (`report.json.motor_contract.safety.max_delta_q_deg
== 1.5`) this cycle — deleted those artifacts and re-ran all three on
train-0 under the landed fix (`report.json.motor_contract` now reads
`0.375` on all three). **Composed seqprobe (the registered instrument)
post-fix: lower/det 6/6 + lower/sto 6/6, BOTH zero-termination
(herr 9.4–10.7mm)** — the exact fix this arm was funded for (2M
canary's `-s1` lower/det was 0/6, ALL over_current-pinned) replicates
under the checkpoint's TRUE trained contract, matching seed-0's own
already-corrected 6/6+6/6 (herr 14.1–14.9mm). hold clean both seeds
(seed0 6/6+5/6, seed1 6/6+6/6). **rise reveals a real cross-seed
divergence invisible under the buggy contract**: seed0 10/12 (2
over_current only) vs seed1 7/12 (5 fails, mostly a `hold_low_height`
stall — robot tucks but stays low, never completes the post-rise
transition — concentrated on flat starts: 0/2 det, 1/3 sto flat).
Video (fresh contact sheet + `rise_det_5` + `lower_det_0`, all
re-pulled post-fix): upright six-foot-planted stands throughout every
mode, no fall/tip; the rise fails are stall-low, not collapse.
**Verdict: rung-5's own named question (does composing the
height-cmd hold segment into rise→hold→lower sequencing preserve
skill) is answered YES — the lower-phase fix is real and cross-seed —
CLOSED. Flat-start rise is NOT solved by this arm's extra budget and
now has sharper cross-seed evidence (10/12 vs 7/12) that it is a
seed-sensitive, budget-invariant residual** — this is the same
pre-existing item already tracked below (Next 0.5 / RUNG-9: mint a
mesh-native `rise_ref` or edit the flat segment of the existing one),
not a new blocker. **Next: open stage-2 design** (rise→walk→lower
composition) using either seed's acq8m checkpoint as the stance
teacher; the walking source is still the track's pre-registered
fallback (`stotight45-seed13`, primitive-family) pending a mesh-era
joystick champion. SKILLS.md row added (joint-scope). Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_standheight_rung5_acq8m{,_s1}_
{gate,owncfg,seqprobe}/` (both post-fix; `-s1`'s pre-fix artifacts kept
as `*_prefix_bugged` for audit), W&B `i78gd6k3`/`auf0f70c`.

Prior entry:

**RE-VERIFICATION IN FLIGHT (launched this cycle, 08-26 ~07:1x, no
ledger entry — direct pod_eval/eval_checkpoint invocations, not a
`launch_run.py` launch): the promoted THE-mesh-stancemix-recipe
champion's own headline evidence (`stancemix_tuckclock_scratch8m` +
`-s1`, promoted 08-26 ~04:3x) was ALSO measured pre-fix (confirmed:
its committed `report.json.motor_contract.safety.max_delta_q_deg ==
1.5`) — it is the foundation every later standwalk arm (including
rung-5 above) warm-starts from, so it is the highest-value re-check.
Re-running gate+owncfg+flatprobe for BOTH seeds under the fix on
their original pods (train-6 seed0, train-7 seed1; code synced).**
Old (pre-fix) artifact dirs moved aside to `*_PREFIX` (not deleted —
comparison value). Expected output:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_tuckclock_
scratch8m{,_s1}_{gate,owncfg,flatprobe}/` (fresh, post-fix). Whoever
picks this up next: diff the flat-pinned valid_plant counts and
DR-0 hold/lower zero-term claims against the `_PREFIX` dirs — if
unchanged, the champion promotion stands as-is (the current-ceiling
residual is a genuine training-time characteristic, confirmed
independent of the eval bug); if the current-ceiling/over_current
tail shrinks or vanishes, the "not a zero-fall closure" caveat on the
promotion should be revisited (possibly upgrades to a cleaner PASS).
Not yet read as of this write — launched, not triaged.

Prior entry:

**STAND_HEIGHT RUNG-5, 08-26 ~07:1x — seed-0 acq8m ACQUISITION PASS
own-scope: the composed-lower residual the 2M canary flagged for
seed0 is CLOSED at 8M (6/6 det+sto, was 2/6 det).** Full detail in the
banner at the top of this file (including a live pod_eval slew-
contract bug found+fixed this cycle — every number below is post-fix).
**Open: the joint call with `-s1` (seed 1, own 2M canary residual was
the OPPOSITE shape — over_current pin, not high-herr drift) — its
gate/owncfg/seqprobe are already DONE on train-0 but CONFIRMED
PRE-FIX (read-only check this cycle: `report.json.motor_contract.
safety.max_delta_q_deg == 1.5`, the legacy value, not the 0.375 this
checkpoint trained under) — do NOT trust those numbers for the joint
call; re-run `-s1`'s gate/owncfg + the manual seqprobe under the fix
before closing (left untouched this cycle per the off-limits
instruction — `-s1` belongs to another cycle). If `-s1`
also clears its own residual: rung-5 is CLOSED, open stage-2 design
(rise→walk→lower composition, needs a walking source per the track's
Stage-2 text). If `-s1` still fails on the over_current shape: rung-5
is PARTIAL (one seed clean, one still current-pinned) — the next
lever is the height-cmd-segment `bc_anchor_coef` loosening (new
default-off cfg + bank rows + unit tests), not a 3rd seed.**

Prior entry: 08-26 ~06:0x — first canary pair BOTH CANARY PASS
(caveated) — mechanism works, residual is seed-dependent lower-phase
softening; funded an 8M continuation before calling rung-5 closed.
`cw-standwalk-stance-mesh2-standheight-rung5`/`-s1` (2M, warm from
this cycle's freshly-promoted scratch8m mesh-stancemix champion,
`goal.mode_seq_stance=1` + `goal.mode_seq_hold_height_cmd=1` +
`hold_height_cmd_frac=1` + the height-aware BC anchor from rung-1)
both show reward rising over the run through the now-familiar
mid-training valley (seed0 2.4→peak~76@1M→trough −180@1.2M→144
final; seed1 6.5→54→trough −216→66 final). The gate's own
pre-registered custom probe (`--cfg-set goal.mode_seq_stance=1
mode_seq_hold_height_cmd=1 hold_height_cmd_frac=1`, det+sto 6+6) shows
REAL height tracking on both seeds (hold herr_end 0.8–3.4mm det /
1.0–3.3mm sto seed0, 1.2–10.3mm seed1 — a moving [-40,20]mm@15mm/s
target, not flat-ignored) and no majority over_current/fall on any
mode. Rise mildly softened both seeds (4/6 det+4/6 sto success, 2
term each). **Lower is where the two pre-registered seeds DISAGREE**
(the gate's own PARTIAL trigger, mapped onto the ledger's binary
canary vocabulary as a caveated PASS): seed0 keeps lower alive with
ZERO terminations but drifts to a high herr on det (4/6 exceed the
15mm success bar, 18–20.5mm; sto stays inside, 6/6 success up to
13.1mm); seed1 has the OPPOSITE shape — lower/det tracks TIGHTLY
(herr 0.3–2.0mm) but trips the over_current safety on ALL 6/6 det
episodes right at/near the end (0/6 success by the herr rule; sto is
clean 6/6). Both read as the campaign's long-documented current-
margin fragility (cur_max pinned 2.4–2.64A near the safety ceiling)
newly stacked onto the height-cmd+mode_seq composition — not a new
mechanism defect, and video (contact sheets, both seeds) shows
upright, all-six-planted stands throughout, no fall/tip/splay/
press-pin. Secondary, likely-cosmetic finding: seed1's ISOLATED
(non-composed, cold "plant" reset) hold-only gate/owncfg probe is far
more fragile (4/6 and 3/6 hold_min_load terms) than seed0's (0–1)
even though seed1's hold is clean INSIDE the composed seqprobe —
probably an artifact of the isolated probe's own reset distribution,
not the real stage-2 entry condition (rise always precedes hold in
practice). Acting on the cheaper, no-new-code lever first: launched
`cw-standwalk-stance-mesh2-standheight-rung5-acq8m`/`-acq8m-s1` (8M,
`--init-from-source` off each seed's own 2M canary checkpoint,
recipe unchanged) to see whether the still-rising Q4 reward trend
closes the lower-phase residual before building a height-cmd-
segment-specific `bc_anchor_coef` loosening (the gate's other named
PARTIAL lever). On PASS (both seeds, lower clears its own
established band with no majority term): treat rung-5 as cleared and
open stage-2 design (rise→walk→lower composition). On FAIL at the
same signature: budget is refuted for this residual, and the
anchor-coef-during-height-cmd lever becomes the next arm (new
default-off cfg + bank rows + unit tests, dig-in scope). Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_standheight_rung5{,_owncfg,
_seqprobe}/`, `..._s1_{gate,owncfg,seqprobe}/`; W&B
`7sg3akct`/`z103ioia`.

Prior entry:

**HOLD-HEIGHT (commandable stand height), 08-25 ~21:4x — rung-1
canary pair `holdheight-rung1`/`-s1` BOTH CANARY FAIL-MECHANISM;
height-AWARE BC-anchor fix built+tested and its own canary pair
LAUNCHED.** The mechanism-health canary dropped the hold BC-anchor
entirely (`train.bc_anchor_coef` forced 0.0, since the existing
anchor's target — the constant, height-BLIND `q_nom` — would fight a
moving height command) and warm-started from the deployed hold
champion with `goal.hold_height_cmd_frac=1.0` (rung-1 elevator,
+/-15mm at 8mm/s, hold+ramp only). Both seeds regressed the
champion's SOLVED quiet-stand skill, not just added height-tracking:
DR-0 gate (the height-cmd cfg rides into the eval via pod_eval's own
`--cfg-set` carry-forward) hold det 1/6 valid_plant + sto 1/6 (seed 0:
det 1/6 sto 2/6), 5/6 det + 5/6 sto (seed 0: 5/6 det + 3/6 sto)
episodes trip the `hold_min_load` safety termination 3-8s into the
15s episode; `cur_max` climbs to 2.0-2.63A (near the 2.64A
torque-saturation ceiling) vs the champion's clean 0.67-0.71A on the
IDENTICAL static-hold gate, drag 75-191mm vs ~0mm. Video: no visible
fall/tip (roll_peak 0.6-2.8deg, all six feet grounded every frame in
the contact sheets) — a genuine but visually-subtle foot-underload
regression, not a collapse. Training reward DECLINED across both runs
(`optimization/reward_per_tick` ~0.42->0.24), a genuine fail per the
08-21 ruling. ROOT CAUSE (matches STAND_HEIGHT.md's own pre-registered
fallback): the height-blind anchor was doing real pose-regularization
work beyond just being height-blind — removing it entirely let PPO
drift into a noisier, higher-current stance even at commanded
height=0. FIX (built+tested this cycle): `train.bc_anchor_
hold_height_aware` (default 0 = legacy constant-`q_nom` target,
bit-exact) in the HOLD/TRACK BC-anchor block, `rl_move/sim/
sim_env.py` — re-targets the anchor at the `FixedFootBodyIK` pose
that reaches the NEXT commanded height (the same one-tick-ahead
convention `train.bc_anchor_lower` already uses), gated to `mode ==
"hold"` and a nonzero commanded height so it composes cleanly with
the existing tilt-comp block and stays a no-op for every non-height-
cmd hold lineage. 5 new tests in `test_bc_anchor.py` (default-off
bit-exact, zero-height no-op, targets-the-commanded-height via
IK-solve equality, track-mode-excluded, feet-planted-descent chain) —
all green; full `test_bc_anchor.py` + `test_hold_height_cmd.py` green
(84/84); `STAND_HEIGHT.md`'s own preflight bank re-run green (4/4,
unaffected). Launched the fix canary pair — exact `holdheight-rung1`/
`-s1` recipe, only `train.bc_anchor_coef` restored 0.0->3.0 (the
champion's own value) plus `train.bc_anchor_hold_height_aware=1.0`
added: `cw-standwalk-stance-mesh2-holdheight-rung1-hha1` (train-0,
W&B `isxy1d5b`) / `-hha1-s1` (train-1, W&B `gfksq1nx`), both VERIFIED
RUNNING. Gate: DR-0 det>=5/6 + sto>=4/6 valid_plant, zero
`hold_min_load` terminations, `cur_max` back within noise of the
champion's 0.67-0.71A band — judged jointly as a pass-rate pair
against `holdheight-rung1`/`-s1`'s own numbers. On PASS: promote to
an 8M acquisition budget and treat rung-1 as cleared, move to rung 2
(full `[-40,20]` range). On FAIL at the same signature: the height-
command mechanism itself (not just the anchor) needs a different
income/gate design — next lever would be pricing the `hold_min_load`
margin directly during height transitions, a genuine dig-in. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_holdheight_rung1{,_s1}_
{gate,owncfg}/`, W&B `roplbk1z` / (seed-0's own id, concurrent cycle).

**RISE, 08-25 ~19:5x — tuckexempt-i0 (seed 0) CANARY FAIL-MECHANISM;
ref-content option then REFUTED BY CONSTRUCTION PROBE (no GPU spent);
next lever = anchor progress-metric code (dig-in flagged).**
`tuckexempt-i0` (phase-gated floor exemption, floor OFF only while the
state-aligned match is inside the ref's tuck segment) reproduced
tuckfloor0's FULL collapse: flat-pinned probe 0/12 with the duty=0
snap-fold freeze (0.52A, h_err 79-87mm, zero tuck motion on strips),
standard DR-0 gate 0/12 (parent meshref 5/6+4/6) with deep starts
either twitching or pressing into tilt/over_current, own-DR
replicates. ROOT CAUSE: the exemption keys on the matched ref INDEX,
but bridge/crouch/rsi low states also state-align into the
height-flat tuck segment, so they lost the floor too — and floor-less
pursuit targets there are near-stationary (half-pace tuck), making
freeze anchor-optimal. Joint pair close on `-i0-s1`'s cycle
(concurrent `tuckexempt0/-s1` pair is the same idea from the
triage-overlap race). THEN, per the pre-registered FAIL branch
(ref-content edit), built `--tuck-rise-mm` into
`make_rise_ref_scripted.py` (default 0 = bit-exact, verified) and ran
three controlled CPU mints: (1) rise ramp in tuck 2nd half FIGHTS the
sinusoidal swing lift (feet in air can't lift the body) — achieved h
flat until press; (2) sweep-split fix (sweep done by 0.6·tuck, then
20mm planted-feet rise) — absorbed silently at 0.53A, zero lift:
~25-30mm commanded-excursion compliance dead-band before liftoff;
(3) 45mm dose — achieved h at tuck end still 0.2mm; achieved height
lags commanded excursion by ~1.5s and ALWAYS lands in the press
(evidence ref: `rise_ref_mesh_tuckrise45.npz`, ramp_i0_ach 258 vs
tuck end 250, first 8mm at t=5.5s). CONCLUSION: on the 3.5kg mesh
model no feasible tuck content places ACHIEVED-height progress inside
the tuck, so the achieved-height-keyed floor (min_h_ahead_mm) can
never track through it — ref-content editing is refuted as the lever,
and the mid-tuck curriculum option is pre-refuted too (rsi already
samples tuck ticks uniformly, j∈[0, i0+0.9(n-1-i0)), and exactly
those rsi starts press-pin in the parent). SURVIVING DESIGN: change
the anchor floor's PROGRESS METRIC to script progress — target tick
= max(matched_j + min_ticks_ahead, height-floor tick) or key the
floor on COMMANDED height (needs a new npz key) — i.e. the floor
should measure progress along the script, not achieved height; a
large index-ahead (~1.0-1.5s, vs the 0.25s lookahead that froze)
keeps the anti-freeze property without requiring height. That is
bc_anchor hot-path code + test_bc_anchor bank rows + a 2-seed canary:
DIG-IN flagged for the deep cycle rather than rushed here. Evidence:
`logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_tuckexempt_i0_{gate,owncfg,flatprobe_det,flatprobe_sto}/`,
W&B j9k3h490. (MCP fb_20260825T191814_a38da6 corroborates: the
flat-start defect reproduces on the true full-STL mesh, not just the
MJX twin.)

Prior entry:

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

-1.7 ANCHOR-LEAK DIG-IN RESOLVED; REPAIRED PAIR RUNNING (08-26
    ~18:0x). The -1.6 dig-in landed: stance→walk interference is a
    shared-Adam UPDATE-isolation defect (populated all-zero grads on
    the gated-out core + momentum step), not a gradient-masking or
    routing bug — full mechanism, probe numbers, and the dual1
    reconciliation in the Last-updated banner. Fix
    `train.bc_anchor_isolate_update` (default 0 bit-exact) landed at
    `2f585a97` with defect+fix unit tests. `cw-standwalk-stance-
    mesh2-stage2-dualbc1-anchor2`/`-anchor2-s1` (2M canary pair,
    train-0/1) re-run the anchor1 recipe with only the fix ON; the
    joint gate's LEAK-FIX/FULL-PASS/FAIL-A/FAIL-B branches (see
    banner/ledger) name the next owner in every outcome. Nothing else
    on the dual-core-BC-init lineage is fundable until that pair
    reads back. Open follow-up owned by FAIL-A if it fires: the same
    zero-grad momentum channel inside PPO's own update
    (single-family recurrent minibatches under mode_seq=0.75).

-1.6 STAGE-2 FIRST CELL FULLY CLOSED, BOTH MECHANISMS FAIL (08-26
    ~16:4x). Both the bare RL fine-tune (`modeseq1`/`-s1`) AND its
    pre-registered stance-only/walk-off `bc_anchor` fallback
    (`anchor1`/`-anchor1-s1`) are now CANARY FAIL - MECHANISM,
    cross-seed replicated, DR-0 + own-DR(0.5), det+sto — full evidence
    in the Last-updated banner and prior banner below. The anchor
    fallback is WORSE than the thing it was meant to fix: hold/lower
    never reach isolated failure on both det+sto in either seed, and
    walk collapses from a weak-but-positive crawl to 0/6 success
    everywhere with two DIFFERENT severe new pathologies per seed
    (leg-sacrifice freeze vs. high-slip shuffle-in-place) — evidence
    the coef=3.0 stance-anchor gradient leaks into walk/shared-value
    territory, not just fails to reach hold/lower. **Per both gates'
    own text: do NOT fund a third anchor dose/config.** REAL NEXT
    STEP (dig-in scope, not a triage-cycle launch): read
    `bc_anchor.py`'s dual-core wiring and the per-tick
    gradient-masking tests to find where/why stance-tick gradients
    reach walk-tick parameters (or a shared value head) despite
    `bc_anchor_walk=0.0` — candidates: a shared trunk upstream of the
    two heads, a shared critic/value function the huge anchor loss
    dominates, or an optimizer-state-level interaction (Adam moments
    shared across the whole network even when per-tick loss masking
    is correct). Only after that root cause is named should a new
    mechanism (true parameter-level isolation, a second optimizer,
    or a smaller anchor coefficient with a verified-isolated gradient
    path) be designed and gated. Until this dig-in lands, standwalk
    stage-2 has no fundable arm on the dual-core-BC-init lineage; the
    stage-1 mesh stance checkpoints (`acq8m`/`acq8m-s1`) remain the
    best available stance teachers and stotight45-seed13 remains the
    walking source for whatever composition mechanism the dig-in
    recommends next.

-1. (context, superseded by -1.5 above) STAND_HEIGHT RUNG-5 CLOSED
    (08-26): the composed rise->hold(height-cmd)->lower lower-phase
    fix is cross-seed-verified under the corrected motor contract
    (both seeds 6/6+6/6 zero-term). The open design item was
    **STAGE-2** (rise->walk->lower composition): pre-register the
    walking-source x mechanism matrix and launch as a batch — DONE,
    see -1.5. Caveat carried forward, not a blocker: flat-start rise
    remains unsolved and is confirmed seed-sensitive (10/12 vs 7/12 on
    the acq8m pair) — stage-1's own "zero falls/tips" gate text is
    still not fully closed by any single arm; the acq8m checkpoints
    are the best available stance teacher candidates but carry that
    residual into stage-2's own session-level eval.

-0.5 ROOT-CAUSE + LEVER for the flat-in-composition rise residual
    above (08-26, this cycle, code-read not guesswork — confirmed the
    isolated `riseonly-bcchain3-meshref-tuckclock-acq8m{,_s1}` flat
    PASS (12/12, 11/12) is NOT contract-bug-contaminated, its
    flatprobe already reads `motor_contract=0.375`, so flat rise is
    genuinely solved standalone and the composed-seqprobe regression
    is a real composition effect, not stale evidence). Read
    `goal_task.py::_sample_mode_seq_stance`/`_seq_segment_traj`: a
    mode_seq segment's length is drawn `U(mode_seq_segment_s_min=6.0,
    mode_seq_segment_s_max=8.0)`, but this run's OWN rise schedule
    (`goal.rise_ramp_s=6.0` + the hardcoded 1 s pre-ramp hold) needs
    >=7.0 s to even REACH the commanded height once — when rise draws
    as the sequence's first segment (the eval's forced case) roughly
    half the U(6,8) draws land under 7.0 s, cutting the ramp off
    before a flat start (the only start_kind needing the FULL
    physical belly->stand climb; bridge/rsi/crouch start partway up
    against the same fixed 0->amp schedule) can finish, and the
    still-low height rides into the next segment where
    `hold_low_height` fires — a probabilistic, start_kind-conditional
    timing truncation, not a skill or reward defect, and the exact
    same class of segment-timing artifact this file's own lower-phase
    entries already diagnosed (`min_tail` truncating a late `lower`).
    **LEVER (launched this cycle, no code change — `goal.mode_seq_
    segment_s_min/_max` are pre-existing shared cfg keys, already used
    as launch-time overrides elsewhere — `distill_gru.py`,
    `verify_modeseq_teachers.py`):** `cw-standwalk-stance-mesh2-
    standheight-rung5-acq8m-segfix`/`-segfix-s1`, 2M continuation off
    each seed's own acq8m checkpoint, identical recipe +
    `goal.mode_seq_segment_s_min=9.0 goal.mode_seq_segment_s_max=11.0`
    (safely clears the 7.0 s rise-schedule floor with margin; only
    changes THIS launch's cfg, not any shared default). Gate: PASS if
    the composed seqprobe's flat-start rise sub-count improves (fewer
    `hold_low_height` terms on flat draws) with hold/lower staying at
    the acq8m level (no new majority term) — confirms the
    segment-timing mechanism and gives a promotable rung-5 recipe
    variant; FAIL (unchanged/worse flat sub-count) refutes the timing
    hypothesis and points back to a genuine skill-interference cause
    (would need a matched multi-teacher/KL mechanism, dig-in scope).

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
0.5 UPDATE 08-25 ~19:5x: the "edit the flat segment of the existing
   ref" option below is now REFUTED (see Now entry: achieved height
   physically cannot appear inside the tuck on the mesh model; three
   controlled mints). The runnable next item is the anchor
   progress-metric redesign (script-index / commanded-height keyed
   floor) — bc_anchor code + bank rows + 2-seed canary, dig-in
   flagged.
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
4. NEW (08-25, operator MCP request fb_20260825T195117_3dce6e):
   commandable standing-height curriculum, built ALONGSIDE (not
   instead of) the rise dig-in above. Full design + reward reuse
   rationale + preflight bank (GREEN, this cycle) in
   `rl_docs/tracks/standwalk/STAND_HEIGHT.md`; key table in
   REWARD.md §4d. Reuses `hold` (not a new goal kind) — new cfg
   `goal.hold_height_cmd_*` (default off, bit-exact), `rl_move/sim/
   goal_task.py`'s `_hold_height_schedule`. Rungs 0-3 (solid-start
   commandable height) are the scope; rungs 4-5 (recover-then-track
   from non-solid, compose with rise/lower) stay OUT OF SCOPE until
   the rise mechanism above is solved (no honest non-solid-but-
   recoverable start distribution exists without it). UPDATE 08-25
   ~21:2x: the pre-registered rung-1 2-seed canary
   (`holdheight-rung1`/`-s1`, WITHOUT `train.bc_anchor_coef`) ran and
   FAILED on a cross-seed flag-leg cheat — height tracking itself is
   good (0.3–8.8mm) but each seed sacrifices a different single leg
   (leg 4 / leg 2) and trips hold_min_load; its own pre-registered
   FAIL branch fired (see Last-updated entry). Next arm on this line:
   implement the height-AWARE hold BC anchor (IK at the commanded
   height per tick, `bc_anchor_lower`'s
   `BodyOffset(height=height_ref)` mechanism, new default-off cfg +
   bank rows + unit tests), then relaunch rung-1 2-seed; S-gate
   strengthening is the registered fallback. DIG-IN flagged
   (reward/env code, not a triage patch).

## Landmines

- Sim only — hardware stand/plant transfer stays operator-owned.
- No stage-2 arm may warm-start from a primitive checkpoint.
- The joystick track owns generic mesh walking; this track owns
  rise/lower + the unification. Coordinate via STATUS, don't
  duplicate its mesh conversion arms.
