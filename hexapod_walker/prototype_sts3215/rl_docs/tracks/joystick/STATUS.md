# joystick - RL from the programmatic gait to joystick control

Last updated: 2026-08-25 ~00:3x (**cw-arch-tf64-joyfullcurr13-v7-hz100-acq1
FAIL: the 100Hz tf64 (2L/d128/8h/ff256) architecture question is ANSWERED
YES (reward matches/beats the matched-step MLP control -- ep_rew_mean 811,
quarters -692/515/671/747, crossed zero well inside the pre-registered
window, walk_direction_valid~0.99, walk_loadslip_ratio~0.58, all comfortably
inside bar), corroborating the prior dig-in's overturn of the "attention
pathology" read. But `walkcurr/frontier` stayed pinned at b0 for all 76
cert rounds and DR-0 gate (0/6 det+sto gait_valid), own-DR(0.5) (0/6+0/6)
and the held-out 60s joygate (48/48 falls, gait_valid_frac 0.0) all agree:
legs [3,5] are structurally sacrificed and the robot topples (term_reason
tilt_pitch, video-confirmed real roll-over on both det_0/det_3 strips, not
a metric artifact). THIRD independent lineage (after walkcurr's from-
scratch diet and hist64-scratch's from-scratch MLP) to show the identical
leg-sacrifice-then-fall fingerprint under a totally different architecture/
diet -- raises the priority of, but does not newly open, the still-unclaimed
cross-lineage root-cause item (`OPERATOR_QUESTIONS.md` 08-24 ~22:0x, updated
this cycle). Does NOT close the 100Hz-transformer line (architecture is
fine) and does not license another reward-magnitude arm on this ladder --
next lever is the leg-sacrifice root cause itself (DIG-IN, unclaimed), not
another architecture/diet variant. No relaunch this cycle: the two live
siblings (`joyfullcurr15-v8-hz100-r2`, `certfreeze-v9`) already cover the
diet-scope and stop-cert-semantics questions. Evidence: `logs/ckpt_eval/
cw_arch_tf64_joyfullcurr13_v7_hz100_acq1_{gate,owncfg,joygate}/`, W&B
`kbctcsua`.)

Previous entry (2026-08-24 ~22:5x (**hz100-r2 FAIL, CONFOUNDED by the
already-fixed V7 diet-scope bug -- bit-for-bit match to certfreeze-
v7's frontier-stuck-at-b1/leg-3-lock signature, not a rate-conversion
finding; relaunched clean at V8+100Hz as -joyfullcurr15-v8-hz100-r2.**
Plain English: the 100 Hz rate-conversion arm
(`cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-r2`, `--walk-
curriculum-version 7`) trained a full healthy 40M steps (ep_rew_mean
2317, quarters 1740/2785/2347/2952, GPU fps 6067 -- the 100 Hz control
loop itself trains stably) but `walkcurr/frontier` shows only 1
promotion (stuck at b1 the whole run, `b1/stop_speed_m_s` pinned
0.033-0.055 vs the 0.015 cert bar) and video is bimodal: some episodes
walk cleanly (det_0 prog 0.83, det_3 prog 1.48) while most others lock
leg index 3 rigid (DR-0 gait_valid 2/6 det 1/6 sto; own-DR(0.5) 2/6
det 3/6 sto). Held-out 60s joygate FAILs with real falls: 8/48 (cap
<=2/48), dir_err med 45.2deg (allow 40), slip/m 2.732 OK (cap 2.9).
This is the EXACT signature already root-caused on
`...certfreeze-v7` (25 Hz): the `WALKCURR_BUCKETS_V7` diet-scope bug
(turn/reversal stress extras leak into the still-front-cone b1 entry
cert bucket, poisoning it) applies its extras from `front45_20s`
instead of `side90_20s` -- already fixed as V8, but AFTER this 100 Hz
run had launched with the stale V7 diet. Verdicted FAIL (ledger),
not informative about the 100 Hz rate itself. **Note found DURING
this same cycle's pod-eval infra fix (see below): `pod_eval.py`'s
2700s/3600s pass timeouts were calibrated for the 25 Hz/15s baseline
and SILENTLY LOSE THE ENTIRE gate pass (no report.json at all, `rc=
-1`) for any control.hz=100+episode-seconds=60 run -- fixed via
`eval_timeout_scale()` (scales both timeouts by hz/episode-seconds
vs baseline, 5 unit tests, `test_pod_eval_timeout.py`), snapshot
`podeval-timeout-scale-fix`. `hz100-r2`'s gate pass was manually
re-run + copied back after the fix landed (the original attempt was
silently lost mid-run); `cw-arch-hist64-joyfullcurr13-v7-hz100-
scratch-s0-r1` and `cw-arch-tf64-joyfullcurr13-v7-hz100-canary2`
launched before the fix and are at the same risk -- check for a
missing/thin gate report before trusting their triage.** Relaunched
the identical warm-start/reward/100Hz stack with the single lever
`--walk-curriculum-version 7->8` as
`cw-arch-hist16-dep1-c1-joyfullcurr15-v8-hz100-r2` (VERIFIED RUNNING
train-1; first attempt without a pod pin landed on a legacy-64M-shm
pod and was correctly refused by `_check_shm_budget`, no wasted GPU
budget). **CAVEAT ON THIS NEW ARM'S OWN READ (found by a CONCURRENT
cycle's `certfreeze-v8` verdict, same cycle window)**: V8's diet-scope
fix does let the frontier clear b1/b2 but then plateaus at b3 with an
UNRELATED, deeper freeze<->cert semantics bug (`_sample_walk_curr`
draws stop and wz independently, so ~half of "stop" segments in any
wz-carrying bucket are correctly exempted from the cert-time freeze
supervisor while the cert's own accumulator still counts them,
capping `stop_speed_m_s` around 0.023-0.042 regardless of policy
quality) -- DIG-IN flagged, fix unbuilt. Since `joyfullcurr15-v8-
hz100-r2` inherits the SAME `--walkcurr-cert-cfg-set goal.walk_stop_
freeze_s=0.4` mechanism, expect it to ALSO plateau at b3 for this
same reason -- that part of its own pre-registered gate text will
likely read as an uninformative repeat, not new 100 Hz evidence. The
part of its read that STAYS informative regardless: whether V8's
diet-placement fix (not the cert-freeze mechanism) removes the
leg-3 lock at 100 Hz too, i.e. DR-0 det gait_valid and held-out
joygate fall-count vs `hz100-r2`'s own 2/6 and 8/48 -- triage on
THOSE numbers, not the frontier/b3 cert reading, until the freeze<->
cert semantics fix lands. Champion unchanged (`stotight45-seed13`);
DONE gate stays met per 08-23. Evidence: `logs/ckpt_eval/
cw_arch_hist16_dep1_c1_joyfullcurr13_v7_hz100_r2_{gate,owncfg,
joygate}/`, W&B `a7aanvq3`; `rl_move/tests/test_pod_eval_timeout.py`.)

Previous entry (2026-08-24 ~23:0x (**certfreeze-v8 PARTIAL: the bucket-
scope fix works exactly as designed (frontier leaves b1, reaches b3,
side90 opens) but then plateaus AT b3 with the IDENTICAL stuck-stop-
cert signature v7 showed at b1 -- ROOT-CAUSED to a freeze/cert
semantics mismatch, DIG-IN flagged, not fixed this cycle.** Plain
English: `walkcurr/promotions` climbed 1->2->3 in the first ~1000 cert
rounds (b0/b1/b2 all instant off the warm start, matching the V8
design intent that front-cone buckets stay clean) but then sat at
frontier=3 (side90_20s) for the remaining ~76/77 cert rounds spanning
nearly the whole 40M-step budget -- `walkcurr/b3_side90_20s/pass`=0
every single round, `stop_speed_m_s` pinned 0.023-0.042 (cap 0.015),
never trending down. This is the run's own pre-registered if-false
branch firing, just one bucket later: bucket placement was not the
mechanism, the stop-settle bar itself is too tight for ANY bucket
that also trains the wz/reversal diet. Held-out 60s joygate: falls
4/48 (cap <=2, v7 was 3/48 -- flat/slightly worse), dir_err med
47.49deg (allow 40), slip/m 2.268 (cap 2.9, clean). DR-0 gate's named
clause ("det gait_valid 6/6 no sacrifice") IS met (clean, prog 0.89,
slip/m 1.35); sto/own-DR still show leg sacrifice (not gate-named).
Zero raw falls anywhere (4 passes / 24+48 episodes), video-clean
six-leg cycling both owncfg contact sheets checked (det0/det3).
**ROOT CAUSE (code-read, not inference)**: `_walk_stop_freeze_override`
(the cert-time supervisor `--walkcurr-cert-cfg-set
goal.walk_stop_freeze_s=0.4` relies on) explicitly EXEMPTS any tick
where `wz_ref != 0` -- correct, a real turn-in-place command should
not be forced to hold still. But `_sample_walk_curr`'s stop and wz
draws are INDEPENDENT rng calls on the same resampled segment
(`stop_frac` for vx/vy, `wz_zero_frac` separately for wz), so in any
V8 bucket carrying the diet (side90_20s onward), roughly HALF of the
nominal "stop" segments (vx=vy=0) also draw wz!=0 and are therefore
exempted from the freeze -- while the cert's own `stop_v_sum`/
`stop_ticks` accumulator (`_walk_probe_summary`) counts every
vx=vy=0 tick toward `stop_speed_m_s` regardless of wz. The
freeze-on/freeze-off numbers already on record from the original
stopfreeze-probe (0.0133 frozen vs 0.0326 unfrozen, on b1 which has
wz_max=0 and is therefore 100% frozen) bracket b3's observed
0.023-0.042 almost exactly as a half-frozen/half-unfrozen average.
This is a genuine, structural freeze<->cert semantics mismatch: for
any bucket that trains wz on top of a linear stop, the cert as
written can NEVER clear 0.015 regardless of policy quality, because
half its "stop" ticks are (correctly) exempted from the very
supervisor meant to force the pass. **NEXT (specified, not built --
DIG-IN)**: align the cert's stop-tick accumulation with the freeze's
own wz-exemption (only count ticks where BOTH linear speed AND wz are
~0 as "stop" for `stop_speed_m_s`/`stop_gate` purposes), or build a
separate, explicit turn-in-place drift/stillness gate if in-place
skate during a "stop-but-turning" segment is itself something worth
grading -- this is a mechanism-semantics design call. **BUILT + BANK-
PROVED + LAUNCHED, same cycle** (this was a pure cert-measurement
alignment, not a reward-pricing change, so `test_task_semantics.py`'s
reward-ranking bank does not apply -- confirmed it has zero
`stop_speed`/`stop_metric`/`stop_gate` references at all, `-k "stop or
walkcurr"` subset re-run clean of any NEW failure from this change,
its 27 pre-existing reds are all unrelated `walkcurr_pf_*`
reward-ranking tests on the walkcurr TRACK's own `goal.walk_pure`
mechanism, nothing this fix touches): `stop_speed_pure_m_s`
(walk_task.py, purely additive, excludes ticks where `|wz_ref|>1e-3`,
the SAME threshold `_walk_stop_freeze_override` uses) +
`WALKCURR_BUCKETS_V9` (byte-identical to V8 except every gated bucket
adds `stop_metric="stop_speed_pure_m_s"`) + `walkcurr_cert.
walkcurr_bucket_pass` reading `spec.get("stop_metric", "stop_speed_m_s")`
(bit-exact when absent). 12 new tests (`test_walk_curriculum.py` 4,
`test_walk_stop_settle_metric.py` 2, `test_launch_run_control_hz.py`
8), full existing banks re-green (`test_walk_curriculum.py` 52/52,
`test_walkcurr_mjx.py` 19/19, `test_walk_stop_freeze/grace/current.py`
14/14). **Also found+fixed a real gap while launching**: the
`--allow-legacy-control-hz` escape hatch CURRENT_TRUTHS documents as
already landed with the 08-24 100 Hz ruling did not actually exist in
`launch_run.py` (`_with_default_control_hz` had no such parameter --
every explicit non-100 `--cfg-set control.hz` was an unconditional
REFUSE), so certfreeze-v8's own already-precedented legacy-rate
pinning could not be reproduced through the normal launch/respec path.
Built it (`launch --allow-legacy-control-hz` / `respec
--allow-legacy-control-hz` / `backlog add --allow-legacy-control-hz`,
threaded through drain's subprocess too; default False = bit-exact
unconditional-refuse, unchanged). **Launched
`cw-arch-hist16-dep1-c1-joyfullcurr16-certfreeze-v9`** (single lever
`--walk-curriculum-version=9`, pinned to the SAME legacy
`control.hz=25` as v8 via the new escape hatch to keep this isolated
from the separate, concurrently-running 100Hz rate-conversion question
`joyfullcurr15-v8-hz100-r1/r2`), VERIFIED RUNNING train-5 -- live pod
log confirms the mechanism fires correctly (`b1 front45_20s` precert:
`stop=0.0125 stop_pure=0.0125 pure_frac=1.00`, bit-exact as designed
since that bucket carries no wz diet). Gate: PASS = frontier promotes
past b3 to >=b5 (rear135 opens) AND joygate falls <=2/48 AND DR-0 det
gait_valid 6/6 no sacrifice; PARTIAL = frontier moves but joygate/
DR-0-gait doesn't fully clear; FAIL = frontier still stalls at b3
(metric fix didn't touch the mechanism, or turn-in-place genuinely
induces real drift the freeze can't suppress) -- points next at a
dedicated turn-in-place drift/stillness gate instead of a metric fix.
Champion unchanged (`stotight45-seed13`); DONE gate stays met per
08-23; this V6/V7/V8/V9 ladder remains operator-ordered hardening
(`fb_20260823T220651_5c66e3`), not gate-blocking. Evidence: `logs/
ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr14_certfreeze_v8_{gate,
owncfg,joygate}/`, W&B run `tvps19cg`, `logs/experiments/
cw-arch-hist16-dep1-c1-joyfullcurr14-certfreeze-v8/wandb_history.csv`;
snapshot `0b383a1c` (tag
`exp/cw-arch-hist16-dep1-c1-joyfullcurr16-certfreeze-v9`).)

Previous entry (2026-08-24 ~22:1x (**DIG-IN VERDICT: the tf64 "attention
pathology" read is OVERTURNED — both transformer canaries re-classed
CANARY PASS; the 2M canary bar itself was miscalibrated; 38M
matched-gate continuation `cw-arch-tf64-joyfullcurr13-v7-hz100-acq1`
RUNNING on train-6.** Plain English: the transformer was declared
broken because its reward fell for all 2M steps while "the MLP
sibling rises to 746" — but that compared the transformer at 2M
against the MLP at 38M. At MATCHED step counts the MLP is
statistically identical to both tf canaries (MLP −738.9 / height_err
94.3mm / loadslip 5.39 at 2M vs canary2 −739.5/95.4/5.54 and r1
−734/99/5.76), and the MLP kept falling to ~−1460 by 7M, crossing
zero only ~12–14M. The "identical collapse signature" across 1L/d64
and 2L/d128 is the architecture-INDEPENDENT early reward valley of
the V7 stack at 100Hz — which is also why 4x width/depth changed
nothing. Checkpoint-level trace found nothing wrong with the
mechanism: attention near-uniform (init-like, entropy 4.11–4.14 vs
ln64=4.16, no NaN/degeneracy), pos_embed at init as is NORMAL at 1230
optimizer steps (the healthy MLP's actor-trunk grad_rms 7.2e-7 is
even smaller than the tf trunk's 2–9e-6), healthy feature variance
(0.69/dim), and per-frame gradient sensitivity correctly dominated by
the newest frame (14x) — framing/causality sound, matching the green
test bank. LESSON (binding for future canaries on this stack): a
from-scratch 100Hz "reward must improve by 2M" bar FAILS the
known-good architecture too — from-scratch canary reads need a
MATCHED-STEP control trajectory, not an end-of-run comparison.
Follow-up now running: `...-acq1` continues canary2's 2L/d128
checkpoint 38M more (40M total, = MLP budget), gated on the MLP's own
matched-step trajectory (upturn by 15M, cross 0 by ~18M, frontier
past b0; still-monotone-down at 15M = REAL architecture evidence and
closes the line). Caveat: the MLP itself FAILED its 40M gate via the
{0,2,5} 3-leg-sacrifice drag (see next entry) — so acq1's
architecture read weights the (a)–(c) shape/turn/frontier clauses;
its end-state gait clauses share whatever 100Hz reward-misalignment
fix that dig-in produces. Evidence: W&B rel6d200/heklqc5l notes +
attached analysis artifacts.)

Previous entry (2026-08-24 ~22:0x (**cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1
FAIL: the from-scratch 100Hz MLP arm learns real forward motion but
converges to a chronic 3-leg-sacrifice drag, never a valid six-leg
gait — CONFIRMS the reward/PPO stack is learnable from scratch at
100Hz (unlike the sibling transformer canaries above), but this
checkpoint itself does not clear the gate.** Plain English: this is
the "sibling MLP" already cited by the tf64 canary verdicts as proof
100Hz isn't inherently unlearnable — reward rose the whole 40M run
(quarters -1089.3/121.1/471.0/402.5, ep_rew_mean 876.7) and the b0
rung's own cmd_prog_frac climbed from ~0 to 1.0-1.5+ (real forward
progress, not noise). But `walkcurr/frontier`/`promotions` stayed 0
the entire run (80 cert rounds, zero promotions past b0), and
`b0_bridge_10s/pass`=0 every single round — most tellingly, `falls`
pinned at exactly 1/round for ~50 consecutive cert rounds (steps
~7450-11331) with NO declining trend, i.e. a stable bad plateau, not
slow convergence toward a pass. Held-out joygate (n=48, seed 90000):
FAIL — 12/48 falls (25%), `gait_valid_frac=0.0` (zero of 48 episodes
had a valid six-leg gait), dir_err med 53.9deg (cap 40); slip/m med
2.096 is the only passing check, and only because the sacrifice-drag
barely displaces the body. Per-leg duty is REPRODUCIBLE across both
the joygate and the separate own-DR(0.5) owncfg pass (det AND sto):
legs {0,2,5} pinned near-zero duty (0.0-0.06, fully sacrificed) while
{1,3,4} carry the gait at 0.93-0.97 duty — a structural split, not
per-episode noise. Own-DR(0.5): det/sto gait_valid 0/6 both, slip/m
15.6/9.2 (vs cap 2.9), roll peak 19-21deg. Video (owncfg contact
sheet) confirms: body tilted, legs splayed/dragging rather than
cycling. In-training periodic self-eval (`eval/walk/survived_frac`)
reads 0 at every logged point from step 4871 to the end (two 0.5
blips only) — the policy has been falling in its own held-out-style
probe for ~3/4 of training with zero improving trend. Per the
08-21/08-22 rules this is the MISALIGNED shape (progress-linked
metrics rising, survival/gait-validity flat-bad the whole back half),
not undertrained-continue — the progress/speed reward terms are
satisfiable by a 3-leg drag that the falls/gait-validity terms don't
suppress at 100Hz's 4x-denser per-tick pricing. The separate DR-0
gate pass (dr-scale 0.0) hung and timed out after 45min with zero
output (rc=-1, no artifacts) — an infra anomaly, not counted against
the verdict, flagged for a dig-in rather than re-run blind. **No
relaunch from this cycle**: falls are plateaued, not slowly
improving, so blind continuation is not well-supported; the V7->V8
walkcurr-diet fix (built for certfreeze-v7's failure) does NOT apply
here since V7/V8 only diverge at side90_20s+ and this run never
promoted past b0/bridge. The decisive next fork is already in flight
on sibling arms this cycle does not own: `cw-arch-hist16-dep1-c1-
joyfullcurr13-v7-hz100-r2` (warm-start at 100Hz) will show whether
starting from a competent walker avoids the same {0,2,5} sacrifice,
and `cw-arch-hist16-dep1-c1-joyfullcurr14-certfreeze-v8` is separately
testing the V7-diet-scope fix on the warm-start lineage. **DIG-IN
flagged**: the identical {0,2,5}-sacrificed/{1,3,4}-active split
appears in both this run's joygate and its own-DR owncfg pass, and
partially overlaps walkcurr's chronic {0,2,3,5}-sacrifice signature
on a totally different architecture/diet/track — worth a dedicated
root-cause pass (obs/action leg indexing, per-leg reward calibration,
or a structural sim asymmetry), since fixing it could unblock both
this ladder and (if in-bounds) the walkcurr rung-1 campaign. Evidence:
`logs/ckpt_eval/cw_arch_hist64_joyfullcurr13_v7_hz100_scratch_s0_r1_
{owncfg,joygate}/`, W&B `c4s7i0e2`.)

Previous entry (2026-08-24 ~21:5x (**cw-arch-tf64-joyfullcurr13-v7-hz100-canary2
CANARY FAIL - CAPACITY RULED OUT, ATTENTION-SPECIFIC PATHOLOGY
CONFIRMED, DIG-IN FLAGGED.** Plain English: the escalation arm
(2L/d128/8h/ff256, matching the proven pre-100Hz `tf-r1-hard1` config,
per canary-r1's own pre-registered "under-capacity -> escalate width/
layers" branch) reproduces the IDENTICAL decline shape as the 1L/d64
canary despite 4x+ more width/depth — reward quarters
-17.7/-192.7/-376.7/-611.9 (r1: -20.4/-197.1/-384.1/-609.4, nearly
indistinguishable), ep_rew_mean -739 (r1: -734). `walkcurr/frontier`
pinned at 0 through all 4 cert rounds, `frontier_pass`=0,
`promotions`=0 (zero real curriculum practice ever unlocked);
`env/height_err_mm` rises 0->104mm (progressive crouch collapse);
`env/walk_loadslip_ratio` rises 0.2->6.17 (2x over the 3.0 cap);
`env/walk_direction_valid` falls 0.95->0.52 with only a partial late
uptick to ~0.69-0.73. Per the canary's own decision tree this is
decisively the FAIL-same-signature branch, not the PASS branch —
width/depth changed nothing about the trajectory. **Per the gate's
own text: STOP dosing tf width/layers; no further transformer arm
until an architecture root-cause trace runs** (attention-weight/
gradient-norm inspection; whether the 0.64s hist64 window's
positional handling is broken specifically at 100Hz control cadence,
not parameter count) — DIG-IN flagged this cycle, unclaimed. DR-0/
joygate reads were not required at this 2M canary bar and were not
waited on; training telemetry alone matches r1's shape point-for-
point and is fully decisive. Evidence:
`logs/experiments/cw-arch-tf64-joyfullcurr13-v7-hz100-canary2/`, W&B
`rel6d200`.)

Previous entry (2026-08-24 ~20:5x (**tf64-small-canary-r1 CANARY FAIL -
MECHANISM: the small-transformer 100Hz architecture arm collapses,
isolated to the transformer trunk itself, NOT the 100Hz rate or reward
stack.** Plain English: the operator-ordered 1-layer/d64/4-head/ff128
causal transformer (0.64s hist64 window at control.hz=100, V7
certfreeze recipe) trained clean to its full 2M canary budget (no
crash/NaN, checkpoint+video synced) but got WORSE the whole way:
reward fell every quarter (-20.4/-197.1/-384.1/-609.4, ep_rew_mean
-734 final), walkcurr frontier never promoted past b0 in any of 4
cert rounds, height_err_mm grew 0->99mm, walk_loadslip_ratio grew
0.2->5.76 (cap 3.0), walk_direction_valid fell 0.94->0.63. Video:
rollout_25 (early) is an ordinary static stand; rollout_49 (late) is a
lower, asymmetric crouch. Ran the held-out joygate too (not required
at 2M, but decisive): gait_valid_frac 0.02 (1/48), slip/m 20-22 (cap
2.9), dir_err ~79deg (cap 40), leg-0 sacrificed in 47/48 episodes — a
near-permanent single-leg lock (zero falls; it drags/circles, never
topples). **Decisive isolation**: the sibling from-scratch MLP arch on
the IDENTICAL 100Hz/V7-curriculum/reward stack
(`cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1`, same family,
37.7-38.5M/40M this cycle) shows HEALTHY rising reward the whole time
(quarters -1089.3/121.1/484.5/384.4, ep_rew_mean=746) — proving the
100Hz rate + reward stack IS learnable, and isolating this canary's
collapse to the tf64-small transformer trunk specifically (under-
capacity or an attention-specific pathology), not a rate-conversion or
reward-alignment defect. Caveat: this canary silently ran on CPU torch
(train-8's CUDA install was wiped by the pod recreation; already
flagged 08-24 ~18:5x as acceptable for a 2M canary) — not implicated
in the behavioral collapse since CPU vs GPU changes wall-clock, not
learning dynamics. Per the run's own pre-registered gate text (FAIL ->
under-capacity suspected -> escalate tf width/layers, not seeds):
launched `cw-arch-tf64-joyfullcurr13-v7-hz100-canary2` (2L/d128/8h/
ff256, matching the proven `cw-arch-tf-r1-hard1` config that DID learn
to walk at 40M on the pre-100Hz stack), single-lever width/depth
escalation, same V7/hist64/100Hz recipe otherwise, on a CUDA-verified
+ shm-fixed pod. Evidence:
`logs/ckpt_eval/cw_arch_tf64_small_joyfullcurr13_v7_hz100_canary_r1_
{gate,owncfg,joygate}/`, W&B `heklqc5l`.)

Previous entry (2026-08-24 ~20:0x (**certfreeze-v7 FAIL: its own repair
poisoned the entry-level cert bucket, frontier stuck at b1 the whole
40M-step run -- scope-fixed as V8 and relaunched.** Plain English: V7
was meant to add in-place-turn + full-reversal command diversity to the
WIDE-heading buckets (side90+) so training would practice the held-out
joygate's stress_mix distribution before those buckets certify. Its
code instead applied the diet "from front45_20s onward", which
includes front45_20s/60s -- still front-cone, not widened-heading, and
the exact rung (b1) whose stop-settle cert (`stop_speed_m_s<=0.015`)
gates every later promotion. Training-side telemetry: `walkcurr/
promotions=1` (only the initial b0->b1 promotion, at cert_round 1 of
80); `pre_b1_pass=False` every round after; b1's own `stop_speed_m_s`
pinned 0.020-0.033 the entire run (a full reversal inside the same 20s
window that must also settle to a stop leaves residual momentum the
window can't absorb). Reward fell every quarter (694->641->525->438)
-- per the 08-21 ruling, a genuine stuck/regressing signal. Held-out
joygate: FAIL (falls 3/48 vs cap 2/48, dir_err med 51.9deg vs allow
40). DR-0 gate: det 6/6 clean but noisy (prog med 0.67); sto 5/6 with
a full leg-0 sacrifice + near-stall episode (video: robot parked,
marching almost in place). own-DR(0.5): det 5/6 with a 2-leg [3,4]
sacrifice+stall (video: two legs rigid while body wiggles), sto clean
6/6. **Fix**: `WALKCURR_BUCKETS_V8` (walk_task.py) is byte-identical to
V7 except the stress-diet extras (wz_max=0.3, reversal_frac=0.15)
start at `side90_20s` (the first genuinely widened-heading rung)
instead of `front45_20s` -- bridge/front45_20s/front45_60s stay
bit-exact V6. `test_walk_curriculum.py` 48/48 (+5 new V8 tests,
incl. a bit-exact-vs-V6 check on front45_60s), `test_walkcurr_mjx.py`
19/19; snapshot `f0ce6f79`. Relaunched
`cw-arch-hist16-dep1-c1-joyfullcurr14-certfreeze-v8` (same stopcur2
warm start / cert-only-freeze / reward recipe as v7, single lever
`--walk-curriculum-version 7->8`) on train-5, **deliberately pinned to
v7's own legacy `control.hz=25`** (`--allow-legacy-control-hz`; see
`OPERATOR_QUESTIONS.md` 08-24 ~20:0x) rather than silently inheriting
the launcher's new-default `control.hz=100` -- a concurrent cycle is
separately mid-debugging this exact lineage family's 100 Hz
warm-start rate conversion (naive + hist-stride-transplant both dead
at init precert), and letting that unrelated, still-open defect leak
into this single-lever diet test would make a FAIL uninterpretable
(diet-scope fix vs. rate mismatch). VERIFIED RUNNING train-5. Gate:
PASS = frontier promotes past b1 to >=b3 AND joygate falls <=2/48 AND
DR-0 det gait_valid 6/6 no sacrifice; FAIL = frontier still stalls at
b1, pointing next at the stop-settle grace WINDOW itself rather than
bucket placement. DONE gate stays met via `stotight45-seed13`
(08-23); this V6/V7/V8 ladder remains operator-ordered full-circle
hardening on top of that, not gate-blocking. Evidence:
`logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr13_certfreeze_v7_{gate,
owncfg,joygate}/`, W&B `flziz48k`.)

Previous entry (2026-08-24 ~19:1x (**cw-amp-joy60-s29-ft1 PARTIAL (metric
bug found+fixed) + scratch-s0 relaunched clean on a 4.0G pod.** Plain
English, two threads: (1) the AMP-M5-champion joystick fine-tune
ordered by the operator (MCP 20260824T175033Z) read as a catastrophic
joygate FAIL (slip/m 176.8 vs cap 2.9) but that was mostly a bug in
`eval_checkpoint.py`'s `slip_per_m` — it divided by
`max(along_dist_m, 0.05)` unconditionally, so 8/24 held-out episodes
that were whole-episode zero-commanded-speed draws (hold/turn-in-place
`stress_mix` archetypes) turned ~9-11m of ordinary marching-in-place
foot travel into slip_per_m~150-230, swamping the median even though
every one of the 16 translating episodes was healthy. Fixed (mirrors
the `progress_ratio` guard already one line above it; `slip_per_m` is
now `None`/excluded when `cmd_dist_m<=0`, `slip_m_total` still reports
raw meters either way; regression test in `test_sim_env.py`) — does
NOT retroactively touch the 08-23 DONE-gate declaration (that champion
was healthy even under the old buggy formula, so its held-out episodes
weren't hitting the zero-cmd_dist case). Corrected reading: translating
slip/m=3.12 (beats parent 3.679 and the PARTIAL bar), dir_err
47.16deg (still >40), gait_valid_frac 0.833 (4/12 sto episodes
sacrifice a leg — a new finding, not previously visible). Separate,
ungated finding: this recipe zeroed every walkcurr-lineage
stop-shaping reward term and visibly does not stand still on a
zero-speed hold command (keeps marching) — the walkcurr V7
stop-freeze mechanism is the fix if hold/stop robustness needs
funding later. Verdict PARTIAL, not gate-blocking (DONE gate already
independently met via `stotight45`; AMP is maintenance-only). (2) The
100Hz from-scratch arm's THIRD launch attempt (`scratch-s0`) also died
pre-training — same shm-fleet defect the tf64-canary hit below, this
time on train-2 (also legacy 64M). Relaunched unchanged (n-envs 3072)
pinned to train-1 (verified 4.0G, free) as `scratch-s0-r1`, now
VERIFIED RUNNING (fps 3600+, healthy PPO stats) — the actual first
live attempt at the from-scratch-100Hz-learnability question. Also
landed `mjx_sharded_vec_env._check_shm_budget` (fails fast with the
safe `--n-envs` spelled out instead of N silent SIGBUS workers).
Evidence: `RL_LOG.md` 08-24 19:0x-19:1x, W&B `mpj70aqg`/`qx5s4c3l`
(FAILED)/scratch-s0-r1. Full detail: `CURRENT_TRUTHS.md` Policy and
eval facts.)

Previous entry (2026-08-24 ~18:5x (**hist64-rr1 CONFIRMS the transplant
class stays closed (2nd repro, prog 0.056 vs 0.052, 0 GPU budget spent
— fail-closed before `learn()`); tf64-small-canary's crash was NOT an
architecture finding, it was a FLEET INFRA DEFECT, now partially
fixed.** Plain English: the small-transformer 100 Hz canary never ran
a single PPO step — all 24 vec-env workers SIGBUS'd inside the first
`env.reset()`. Root cause: `obs.history_frames=64` at `n_envs=3072`
makes the shared-memory obs array alone ~54MB, and pod train-8 was
still on the k8s-default 64M `/dev/shm` — the 08-10 dshm-4Gi fix
(COMMANDS.md gotcha 13c) was only ever HALF-applied fleet-wide.
`df -h /dev/shm` audit across all 12 pods: train-1/5/7/9/10/11 fixed
(4.0G), train-0/2/3/4/6/8 still legacy (64M). Recreated train-6 and
train-8 (both idle) via `coreweave_pods_mjx_scaleout.yaml` +
`bootstrap_train_pod.sh` — verified 4.0G on both; relaunched
`cw-arch-tf64-small-joyfullcurr13-v7-hz100-canary-r1` on the fixed
train-8, confirmed training past reset cleanly (2 iters/98k steps,
healthy PPO stats) where the parent instantly crashed. **train-0/2/3/4
remain legacy 64M** (recreate opportunistically next time one is
idle). Full writeup + the apply-file gotcha (it also applies
unauthorized train-12..15 blocks — caught + deleted, no lasting
effect): `rl_docs/COMMANDS.md` gotcha 13c. Evidence:
`RL_LOG.md` 08-24 18:45-18:51, W&B `6hvipngm`/`tvd8vdh8`/canary-r1.)

Previous entry (2026-08-24 ~18:2x (**100 Hz RATE-CONVERSION LINEAGE:
`-hist64` DIED AT INIT PRECERT TOO, WORSE than the plain `-r2` drop —
UNVERDICTED, flagged DIG-IN, not a class-stop yet.** Plain English:
after attempt-1 (`...-v7-hz100`) died fail-closed at init (b0
prog=0.203 < 0.50 bar), a concurrent cycle built `--hist-stride-
transplant` (scatter the 25 Hz parent's hist16 first-layer columns to
rate-matched slots of a densified hist64 stack, so at init the policy
theoretically reads exactly its trained 40 ms-spaced frames) and
relaunched as `...-v7-hz100-hist64` keeping `--walkcurr-cert-at-init`
on. It died at the SAME precert gate, but WORSE: b0 prog=0.052 (vs
0.203 for the naive same-old-hist16-just-faster-ticking `-r2` arm),
0 falls, slip/m 3.65. The transplant's own offline smoke test (a
`DummyVecEnv` synthetic + the real parent checkpoint at exact launch
shapes) verified action-equivalence within 7e-6 fp noise BEFORE
launch — so either that equivalence check doesn't cover something the
real MJX walk env does differently (frame ordering, obs scaling,
value-net width, or a `history_frames` cfg key read inconsistently
between the offline check and the actual env), or the theory itself is
wrong about what "equivalent to the parent" should score under the
precert bucket sampler. **Root cause NOT isolated — do not retry
another `--hist-stride-transplant` dose/stride blind, and do not
conclude the mechanism is a dead end either; both are live
possibilities pending a trace.** Meanwhile `...-v7-hz100-r2` (cert-
at-init/precert dropped, frontier starts honest at b0, no transplant)
is VERIFIED RUNNING train-0 and training normally — that arm is
unaffected by this finding and remains the live 100 Hz read. Ledger
state fixed this cycle (`hist64` was stuck INTENT after its own dead
process went unreconciled — `launch_run.py checkup` confirmed DEAD,
status set to FAILED with log tail via `update --set`); no
interpretive verdict written pending the trace. Evidence: pod
train-2 log `/tmp/train_cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-
hist64.log` (already rotated off the pod once a new run lands there —
copy it first), W&B `ppjeexty`, code `rl_move/sim/train_ppo_sim.py`
`hist_stride_transplant` + `train_ppo_mjx.py` `--hist-stride-
transplant` (snapshot `7c7175e2`).)

Previous entry (2026-08-24 ~18:0x (**NEW OPERATOR-ORDERED CANDIDATE
LINEAGE: `cw-amp-joy60-s29-ft1` VERIFIED RUNNING train-4** (MCP order
20260824T175033Z). The AMP M5 champion `..._phasehz11_s29.zip` was
manually run through the corrected 60 s joygate (fastprofile_v1,
operator-run): FAIL but stable — n=48, ZERO falls, gait_valid 1.0, no
sacrificed legs, slip 3.679 (cap 2.9), dir_err 48.15 (cap 40); det
near-gate, stochastic/command-churn is the miss. This run fine-tunes
s29 warm-start with its exact trained recipe (25 Hz, phase_obs 1.1 Hz,
obs_body_vel=2, yaw_cmd 0.3, phase_run_on_yaw, fault_health, fast
motor contract 1500/80/5deg, stress_mix 4.0 s jitter 0.5, AMP style
0.5) changing ONLY episode-seconds 15->60 to match the gate horizon;
8M steps, phase hardening, track joystick (cross-track by operator
order; watcher joygate fires on finish, held-out seed 90000, DR-0+
own-DR det+sto, PASS = 0 falls + gait valid + slip<=2.9 + dir<=40).
Independent of the certfreeze/V7 ladder below — champion unchanged.)

Previous entry (2026-08-24 ~17:5x (**certfreeze SEED-PASS-RATE GRID
CLOSED, 3/3 uniform FAIL — the base run's verdict was NOT seed luck.**
`-s1`/`-s2` (the n=3 batch launched alongside the base arm) both
finished and read exactly like the base: `walkcurr/frontier` promotes
b0->b5 cleanly in all 3 seeds (precert b0/b1 pass every time) — the
cert-only-freeze curriculum-unlock is real and seed-robust — but the
held-out 60s joygate fails all 3 (falls 6/48 base, 7/48 s1, 8/48 s2,
all over the <=2/48 cap) and the new DR-0 leg-3(+4) sacrifice lock
appears in all 3 (base det gait_valid 3/6, s1 4/6, s2 2/6, always
sac-ing leg index 2 = "leg 3"). Uniform, not split — per the grid's
own pre-registered read this decisively rules out seed noise as the
explanation and confirms the b2+ heading-widening V6 practice diet
itself (not freeze mechanics, not current-charge dose, not seed) is
the damage source. No further certfreeze-recipe arms of any seed;
`cw-arch-hist16-dep1-c1-joyfullcurr13-certfreeze-v7` (wz turning +
15% instant-reversal added to every V6 bucket, already built +
VERIFIED RUNNING train-1 before this grid closed) remains the correct
standing next lever and needs no seed batch of its own until its own
single read comes back — if it fails too, its own pre-registered
branch escalates to the heading-BAND-WIDTH axis, not more diet
diversity. Champion unchanged (`stotight45-seed13`); joystick DONE
gate stays met via that champion; this ladder remains operator-ordered
full-circle hardening (`fb_20260823T220651_5c66e3`). Evidence:
`logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr12_certfreeze_{s1,s2}_{gate,owncfg,joygate}/`,
W&B runs `5nsinbnj` (s1) / `k7jfofa9` (s2). Prior banner below.)

Previous entry (2026-08-24 ~17:3x (**certfreeze VERDICTED FAIL — the
cert-only-freeze repair does NOT rescue the joygate; 3/3 independent
arms (full training freeze, cert-only freeze, k=2.0/k=6.0 current-
charge doses) now show the SAME ~6-7/48 over_current regression,
decisively pointing at the b2+ heading-widening PRACTICE DIET itself,
exactly the gate's own pre-registered if-false branch.** Plain
English: making the stop-hold freeze cert-time-only (training clean,
cert/precert env frozen) keeps the frontier-promotion win (b0->b5
again) but does NOT fix the safety regression — held-out 60s joygate
falls 6/48 (dr0 4/24, dr0p5 2/24), all six `over_current`, vs parent
stopcur2's 1/48 and the gate's own <=2/48 bar. **NEW finding**: this
run's own DR-0/own-DR gate (fixed-heading, not just the joygate) also
regressed hard and shows a mechanism candidate — det gait_valid
crashed 6/6 (stopcur2 parent, clean) -> 3/6, with **leg 3 SACRIFICED
in half the det episodes** (progress_ratio ~0.53 vs ~1.0-1.6 clean),
reappearing at DR-0 from a lineage that was previously clean there.
Since stopcur2 (front-cone-only training) is clean and certfreeze
(same weights + b2-b5 wide-heading training) is not, the wide-heading
practice itself — not the current-charge dose, previously blamed on
stopcur6 — can induce this lock fresh. Root cause (band-width vs.
command-diversity within the band) is NOT yet isolated — flagged for
a deep dig-in once the repair below reads, regardless of its verdict.
**REPAIR BUILT + LAUNCHED same cycle**: new `WALKCURR_BUCKETS_V7`
(`--walk-curriculum-version 7`, tag `exp/cw-arch-hist16-dep1-c1-
joyfullcurr12-certfreeze-v7`, snapshot 8a4261ba) — byte-identical
ladder to V6 except every non-bridge bucket now also draws in-place
turning (wz +-0.3 rad/s, 50% zero) and a 15%-chance full instant
reversal per resampled segment, the bucket-diet analogue of the
held-out joygate's stress_mix family (sweep_circle/square/flip_180)
that the plain V6 sampler never drew at all. V1-V6 tables lack the
new keys so `.get(..., 0.0)` defaults make this bit-exact for every
existing lineage (`test_v6_sampler_bit_exact_when_stress_fields_
absent`); `test_walk_curriculum.py` 49/49 (+6 new), `test_walkcurr_
mjx.py` 19/19, `test_task_semantics.py` unaffected (211 tests, 1
pre-existing known-red unrelated to this change). Launched
`cw-arch-hist16-dep1-c1-joyfullcurr13-certfreeze-v7` (single lever
vs certfreeze: V6->V7 only, same stopcur2 warm-start, same
k_walk_stop_current=2.0, same cert-only-freeze frontier assist;
VERIFIED RUNNING train-1). Gate: PASS = frontier still promotes past
b1 AND joygate falls <=2/48 AND DR-0 det gait_valid stays >=5/6 with
no leg-3 sacrifice; PARTIAL = joygate improves without clearing, or a
genuine trade against frontier/DR-0-gait; FAIL = joygate stays
>=4/48 over_current and/or leg-3 lock persists unchanged, which would
point the next lever at heading-BAND-WIDTH itself (a staged/narrower
band-opening ladder) rather than command diversity within a band.
Champion unchanged (`stotight45-seed13`); the core joystick DONE gate
stays met per 08-23 via that champion; this V6/V7 ladder remains
operator-ordered full-circle hardening (`fb_20260823T220651_5c66e3`).
Evidence: `logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr12_
certfreeze_{gate,owncfg,joygate}/`, W&B run `ffhhv474`. Prior banner
below.)

Previous entry (2026-08-24 ~13:4x (**freeze40-stopcur6 DIG-IN COMPLETE —
verdict FAIL (both its own pre-registered FAIL branches fired), with
TWO corrections to the prior triage's story.** (1) **CFG DRIFT
discovered: the "k=6.0 twin" actually TRAINED at
`reward.k_walk_stop_current=2.0`** — the respec inherited freeze40's
reward cfg and only changed `--init-from` (ledger `extra_args` + W&B
`config.reward_cfg` both confirm k=2). The earlier "dose-invariant
across k=2.0/k=6.0" claim is therefore WRONG; the correct claim is
**INIT-invariant**: the training-time-freeze regression reproduces
from a second, differently-trained warm-start (stopcur6's k=6-trained
weights), with BOTH freeze runs training at k=2.0. The class-stop on
training-time freeze (`--cfg-set goal.walk_stop_freeze_s>0`) still
stands — its controlled evidence is freeze40's 2x2 plus this
second-init reproduction, not a dose sweep. (2) **New B-control run
this dig-in** (twin ckpt + freeze-OFF joygate, same held-out seeds
90000, `logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr11_freeze40_
stopcur6_joygate_freezeoff/`): **3/48 falls (0 at DR-0, dir_err dr0
36.65deg == parent 36.02) vs freeze-ON 6/48 (3 at DR-0, dir 43.8) vs
parent 1/48.** Unlike freeze40 — whose DR-0 det falls persisted with
freeze OFF — THIS checkpoint's DR-0 regression (falls AND dir_err) is
entirely EVAL-mechanism-caused; its weights damage expresses only
under own-DR(0.5) (1 -> 3 over_current stop-episode falls with the
mechanism off). Same training-data-corruption root cause, milder/
DR-shifted expression; the certfreeze repair (cert-only freeze,
training freeze off, eval freeze off) removes both expressions.
(3) **Bonus fact for the leg-lock lever: 40M steps at the REDUCED
k=2.0 dose did NOT undo the leg-3 lock** (own-DR det gait_valid 2/6,
numerically identical to the k=6-trained parent) — the lock is baked
into the stopcur6 WEIGHTS, not sustained by ongoing k=6 reward
pressure, so dose-reduction-only is refuted as a lock repair; a
future lock arm needs its own mechanism and should prefer the clean
stopcur2 lineage. Frontier win reconfirmed (b1->b5, 4 promotions/0
rollbacks, b1 cert 72/79). Verdict + artifacts on the run; certfreeze
(train-1) remains the live repair arm. Prior banner below.)

Previous entry (2026-08-24 ~13:3x (**freeze40 DIG-IN COMPLETE — verdict
FAIL, root cause ISOLATED by a 2x2 control, mechanism repaired and
relaunched as `cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze`
(VERIFIED RUNNING, train-1).** Plain English: the stop-freeze
genuinely unlocks the curriculum (first-ever b0->b5; b1 cert held all
run) but applying it during TRAINING damages the policy itself. The
2x2 on the same held-out joygate seeds (90000) settles where the
damage lives: A freeze40ckpt+freeze-ON-eval **7/48** falls; B
freeze40ckpt+freeze-OFF **4/48**; C stopcur2 parent+OFF **1/48**; D
parent+ON **2/48** (artifacts on controller: `logs/ckpt_eval/
cw_arch_hist16_dep1_c1_joyfullcurr11_freeze40_joygate_freezeoff/`,
`..._joyfullcurr9_stopcur2_joygate_freezeon/`). B repeats A's exact
DR-0 det falls (det6/det11, both over_current) with the mechanism
disabled — the DR-0 regression is in the WEIGHTS; the eval-time
mechanism on healthy parent weights costs ~1 fall (C->D, noise-band).
dir_err regression is eval-mechanism-only (B dr0 33.75deg == parent
33.93; A 41.78). Fall videos (reproduced DR-0 det pass, train-1
`..._freeze40_joygate_dr0_video/`): over_current STRAIN terminations
(det6 progressive crouch/splay, det11 upright cycling then current
trip), not topples. ROOT CAUSE: the freeze discards the policy's
action on frozen ticks, so PPO trains on executed!=proposed
transitions — the policy drifts unpunished during holds and strains
after resume (plus a b2-b5 diet shift). This ALSO answers the
stopcur6-twin banner's open question just below: the falls are NOT
eval-time freeze-release events (B falls with freeze OFF), so
per-tick release traces are moot — trace the TRAINING-data corruption
instead, which the repair removes. REPAIR (landed, tag
`exp/cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze`, 24/24 tests):
new `--walkcurr-cert-cfg-set K=V` applies cfg overrides to the
walkcurr CERT/PRECERT env ONLY (default empty = bit-exact).
certfreeze arm = warm-start stopcur2, training freeze OFF
(goal.walk_stop_freeze_s=0.0), cert-env freeze ON (0.4); verified at
init: precert b0 PASS + b1 PASS (stop 0.0124 < 0.015) at step 0.
Gate: frontier past b1 AND joygate in parent band (falls <=2/48, dr0
dir_err ~<=36deg; its joygate prestage correctly evals freeze-OFF
since the flag is not a --cfg-set). If it STILL regresses ~4+/48,
the damage is the b2+ practice diet itself -> next lever is mixing
stress_mix commands into bucket training, not freeze mechanics. The
freeze-arm launch hold is LIFTED for cert-only-freeze arms; training-
time freeze (--cfg-set goal.walk_stop_freeze_s>0) stays CLOSED as a
class — evidence: freeze40 (k=2.0) + freeze40-stopcur6 (k=6.0), same
regression both doses. The stopcur6 twin stays another cycle's to
verdict; this 2x2 is its dig-in evidence. Prior banner below.)

Previous entry (2026-08-24 ~13:2x (**`cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40-stopcur6`
finished, UNVERDICTED — the second twin CONFIRMS the freeze40 finding
is a property of the FREEZE MECHANISM ITSELF, not a k=2.0-current-
charge-dose artifact, and DECISIVELY CLOSES one of the two open
sub-questions.** Plain English: same freeze cfg
(`goal.walk_stop_freeze_s=0.4`) as `-freeze40`, but warm-started from
`stopcur6` (k=6.0 current charge, the dose with the pre-existing
leg-3 rigid-lock trade under own-DR) instead of `stopcur2`. Full
40M-step training run, all 5 prestage passes now read (DR-0 gate,
own-DR(0.5), session [informational, obs-mismatch as expected], the
held-out 60s joygate, and W&B history). Results, read jointly with
`-freeze40`:
- **Frontier promotion: TRUE, matches freeze40.** `walkcurr/frontier`
  climbed b0->b5 (4 promotions, 0 rollbacks across 80 cert rounds —
  even cleaner than freeze40's 13-rollback path), so b2-b5 got real
  PPO practice on this dose too. b1's own stop cert held (72/79
  rounds, 91% <=0.015 m/s, no downward trend) — the freeze's core
  win (unstick the curriculum) reproduces across BOTH current-charge
  doses.
- **Held-out joygate: REGRESSED, matches freeze40's direction and
  magnitude.** falls **1/48 (stopcur6 parent) -> 6/48 (freeze40-
  stopcur6)** — freeze40 itself went 1/48 (stopcur2 parent) -> 7/48.
  Per-episode `term_reason` audit: 5/6 falls are `over_current`
  (dr0/det/0, dr0/sto/11, dr0p5/sto/{0,4,10}) and 1/6 is `tilt_roll`
  (dr0/sto/6, roll_peak 26.2deg, sac=[]) — the SAME dominant
  over_current signature as freeze40 (5/7 there), at a DIFFERENT
  current-charge dose (k=6.0 vs k=2.0). dir_err med 47.09deg (dr0
  43.8/dr0p5 49.56) vs stopcur6's own ~45deg. **This rules out
  "current-charge dose specifically interacting with the freeze" as
  the mechanism — the regression is dose-invariant, so the freeze's
  resume-transition (frozen command -> fresh policy action, timer
  resets to 0 the instant a new command begins per
  `test_walk_stop_freeze.py::test_resumed_walking_resets_the_timer`)
  is the prime suspect, not an interaction with the current-charge
  reward term.**
- **Leg-3 lock / own-DR gait_valid: UNCHANGED, not reduced — the
  run's OWN pre-registered FAIL/PARTIAL branch, decisively closed.**
  own-DR(0.5) det gait_valid = **2/6** (sac=[3,4], video-confirmed on
  `walk_det_1.mp4`/`.png`: leg 3 held rigidly folded/aloft in an
  identical pose across the whole clip while the other 5 legs cycle)
  — IDENTICAL to `stopcur6`'s own already-recorded 2/6, not "reduced/
  gone" as the gate's if-true branch hoped. DR-0 (gate pass) det also
  matches the parent's own 4/6 exactly (sac=[3]). **The freeze does
  NOT touch the current-charge leg-lock pathology at all — confirmed
  orthogonal, not just "still present," by an exact numeric match to
  the untouched parent.**
- **Net read (joint with freeze40): the freeze mechanism has now
  been tested at two current-charge doses (k=2.0, k=6.0) with
  IDENTICAL qualitative behavior on all three axes** (frontier: both
  promote cleanly; joygate: both regress ~6-7x, both dominated by
  over_current; leg-lock: both leave the dose-specific pathology
  numerically unchanged). This is strong evidence the freeze
  mechanism itself — not a training-time interaction with any
  particular current-charge dose — is the joygate regression's cause,
  and that leg-lock needs its own separate fix regardless of the
  freeze (both of the run's own pre-registered if-false branches
  confirmed). What is NOT yet answered by either twin (needs the
  dig-in's full toolkit, not another triage read): whether the
  over_current spike specifically clusters at the freeze-RELEASE tick
  (the mechanistic story implied by the immediate timer-reset code
  path) versus somewhere else in the stop/resume cycle — that needs
  per-tick current traces around freeze-release events on an
  eval-only isolated run (freeze on vs off, same checkpoint, same
  command seed), not a from-scratch relaunch. Leaving this run
  UNVERDICTED per the model-tiering rule (this is a triage cycle);
  both twins are DIG-IN evidence for whichever cycle traces the
  resume-transition. Do not launch any further freeze-mechanism arms
  until that dig-in reads. Evidence: `logs/ckpt_eval/
  cw_arch_hist16_dep1_c1_joyfullcurr11_freeze40_stopcur6_{gate,
  owncfg,joygate}/`, `logs/experiments/cw-arch-hist16-dep1-c1-
  joyfullcurr11-freeze40-stopcur6/wandb_history.csv`, W&B run
  `z7kv7bsw`. Prior banner below.)

Previous entry (2026-08-24 ~12:4x (**`cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40`
finished, UNVERDICTED — DIG-IN flagged: the freeze mechanism DOES
promote the ladder past b1 for the first time ever, but the held-out
joygate got WORSE, not better, exactly the run's own pre-registered
if-false branch.** Plain English: real 40M-step training with the
stop-hold freeze (`goal.walk_stop_freeze_s=0.4`) on top of stopcur2
finally unstuck the curriculum — `walkcurr/frontier` climbed b0->b1->
b2->b3->b4->b5 (cert_round 1/2/3/9/13, 5 promotions, 13 rollbacks
along the way), so buckets b2-b5 (front45_60s/side90_20s/side90_60s/
rear135_40s — the actual point of the operator's full-circle order)
got REAL PPO practice for the first time in the whole V6 lineage. b1's
own stop cert held up fine through training (88% cert-round pass rate,
0.012-0.016 m/s band, no downward trend, no regression). But the
standard held-out 60s randomized joygate — the safety net the run's
own gate text named — got WORSE than its stopcur2 parent, not
neutral: falls **1/48 (stopcur2) -> 7/48 (freeze40)**, dir_err med
41.8/51.7 (dr0/dr0p5) vs stopcur2's ~42-45, own-DR(0.5) sub-pass alone
is 5/24 falls. Per-episode `term_reason` audit: 5/7 falls are still
`over_current` (the exact failure mode stopcur2's current-charge was
built to fix, reappearing despite the SAME k_walk_stop_current=2.0
still being active) and 1 fall is `tilt_roll` with `sacrificed_legs:
[3]` — the same leg-3 signature stopcur6 showed at k=6.0, now
appearing at k=2.0+freeze under the harder randomized command mix.
gait_valid_frac 0.9375 (was 1.0 at DR-0 for stopcur2). Standard DR-0/
own-DR gate+owncfg evals were still running on the pod at triage time
(60s x12x2 episodes each, not done yet) — not needed to make the call:
the joygate result alone is a >=7x regression vs the named parent,
squarely the run's own pre-registered "if the freeze introduces a new
fall mode: dig into the resume-transition (frozen command -> fresh
policy action) before any wider deployment" branch. Leaving this run
UNVERDICTED per the model-tiering rule (this is a triage cycle) —
frontier promotion is genuinely good news, joygate regression is a
genuine new pathology, and the correct read (does the freeze mechanism
itself cause over_current/leg-sacrifice at the freeze-release
transition, or is this a training-time interaction with the
current-charge under the fuller command mix) needs the full toolkit
(per-episode video, isolate freeze-on/off at eval-time on this exact
checkpoint) rather than a triage-cycle guess. The concurrent
`-freeze40-stopcur6` twin (k=6.0 dose, another cycle's pod) will give
a second data point on the same question. Do not launch any further
freeze-mechanism arms until this dig-in reads. Evidence: `logs/
experiments/cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40/
wandb_history.csv` (walkcurr/frontier, b1 stop cert trend),
`logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr11_freeze40_joygate/
gate_verdict.json` + per-pass `report.json` (term_reason audit), W&B
run `yi0s9g6x`. Prior banner below.)

Previous entry (2026-08-24 ~09:1x (**stopfreeze-probe-stopcur6 PASS: the
structural stop-hold GENERALIZES across current-charge dose.** Plain
English: same eval-only (~15s, no PPO) precert dry run as the original
stopfreeze-probe, this time reading the stopcur6 checkpoint (k=6.0
current charge, the dose with the known leg-3 rigid-lock trade under
own-DR) instead of stopcur2. Result: b1 front45_20s stop_speed_m_s
0.0133 (settled 0.0044, settled_frac 0.80) — under the 0.015 cap,
matching stopcur2's post-freeze read to 3 decimals exactly. This is
informative beyond just "it works twice": two checkpoints trained
under different reward prices converge on the IDENTICAL residual
number once the freeze overrides their command stream, which points
at the freeze's post-grace floor being a hold-mechanics/momentum-decay
property, not something either checkpoint's policy quality
contributes to. **REFILL (this cycle):** launched
`cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40-stopcur6` (respec of
`freeze40`, same freeze cfg, same 40M budget, but warm-started from
`stopcur6.zip` instead of `stopcur2.zip`; VERIFIED RUNNING,
train-0), the real-training twin of `freeze40` for this dose. It
answers two things the precert probe cannot: (1) does
`walkcurr/frontier` promote past b1 the same way under real training +
the full randomized joygate mix, and (2) does the freeze's forced hold
at stop also reduce/remove stopcur6's own-DR leg-3 sacrifice pathology
(plausible since the lock looks like a stop-adjacent isometric fight
the freeze would preempt) — orthogonal to `freeze40`'s own stopcur2
read, which has no leg-3 pathology to test against. Gate: held-out
joygate falls stay <=1/48 (not regressed by the freeze), own-DR det
gait_valid recovers toward 6/6 (leg-3 lock reduced/gone), frontier
promotes past b1. If-false (leg-3 lock persists unchanged): freeze
fixes the speed-cert floor but is orthogonal to the current-charge
leg-lock pathology, which still needs its own fix (a per-leg-fairness/
smoothness term, per the stopcur6 verdict's own open item). Evidence:
`rl_docs/runs/cw-arch-hist16-dep1-c1-joyfullcurr10-stopfreeze-probe-
stopcur6.md`, W&B run `yaq7sy9z`. Prior banner below.)

Previous entry (2026-08-24 ~08:5x (**BUILT + VALIDATED the structural
stop-hold lever the stopsettle-probe's own gate text named, and it
WORKS on the first try.** Plain English: the audit banner just below
(stopsettle-probe, INFORMATIVE) closed the entire stop-speed/stop-
current REWARD-PRICING lever for good and named the specified next
step: "an explicit freeze/hold controller on stop commands." This
cycle built exactly that: `goal.walk_stop_freeze_s`
(`sim_env._walk_stop_freeze_override`, default 0.0 = off/bit-exact,
5/5 new unit tests in `test_walk_stop_freeze.py`) is a hook in the
shared `_step_begin` pre-physics path (used by every task/goal, every
backend — CPU sim, MJX warp, eval scripts — so one hook covers the
whole stack). Once a walk/quadwalk-mode stop segment has been
commanded for more than the threshold, it DISCARDS the policy's own
proposed command for that tick and re-issues the PREVIOUS tick's own
safe command instead — an actual physical hold, not a price. Turn-
in-place (`wz_ref != 0`) is exempted exactly like the reward charges.
Landed via `cw-arch-hist16-dep1-c1-joyfullcurr10-stopfreeze-probe`
(another `--walkcurr-precert-only` eval-only dry run, ~15s, no PPO,
reading the UNCHANGED `stopcur2` checkpoint): with the freeze on,
`walkcurr/pre_b1_pass` flips **0 -> 1** — `stop_speed_m_s` drops
**0.0326 -> 0.0133** (under the 0.015 cap for the first time in the
entire V6 ladder, 6+ arms and ~280M cumulative training steps), with
zero cost elsewhere (`prog_frac` 1.06, falls 0, roll 2.8deg, all
in-band). This is a genuinely different kind of fix from every prior
arm: it needed **zero additional training** — a structural
supervisory override on an already-trained checkpoint clears a bar
that reward-shaping alone never touched at any dose. **REFILL (this
cycle):** launched `cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40`
(warm-start from `stopcur2`'s own checkpoint, same reward cfg
including the proven `k_walk_stop_current=2.0`, `+goal.walk_stop_
freeze_s=0.4`, real 40M-step budget, VERIFICATION FAILED is expected
launcher-poll noise per the stopsettle-probe/stopfreeze-probe
precedent — check the ledger/pod log directly, not just the
launcher's own status field) to answer the two things the static
precert probe cannot: (1) does `walkcurr/frontier` finally promote
past b1 and start practicing b2-b9 (side90/rear/full-circle — the
actual point of the operator's full-circle order, never once
practiced across the whole lineage), and (2) does the freeze survive
real training dynamics and the full randomized joygate command mix
(turn/stop/resume sequences far richer than the scripted precert
probe) without a new fall mode at the freeze-release transition.
If-false (frontier still stuck, or a new fall mode appears at
resume): dig into the resume transition before any wider deployment.
Evidence: `logs/experiments/cw-arch-hist16-dep1-c1-joyfullcurr10-
stopfreeze-probe/`, W&B run `u3go1oj8`; code: `sim_env.py`
`_walk_stop_freeze_override`, `walk_task.py` (3 reset-site inits),
`test_walk_stop_freeze.py` (5/5), tag `exp/cw-arch-hist16-dep1-c1-
joyfullcurr10-stopfreeze-probe`. Prior banner below.)

Previous entry (2026-08-24 ~08:4x (**`joyfullcurr10-chg2`/`-chg4` BOTH
FAIL, and a same-cycle cert-methodology audit CLOSES the
stop-speed-charge-dose lever for good, by both price AND
measurement.** Plain English: after stopcur2 proved the actuator-
CURRENT charge fixes over_current falls but leaves the walkcurr b1
"stop creep" cert untouched, this cycle doubled/quadrupled the
stop-SPEED charge on top of it (`k_walk_stop_charge` 1.0->2.0/4.0,
`k_walk_stop_current` held at the proven 2.0). Neither moves the cert
(`stop_speed_m_s` finishes 0.0416/0.0426, cap 0.015, same 0.027-0.048
band every dose 0/1/2/4x has landed in across 80 cert rounds each) and
BOTH actively regress what stopcur2 had just fixed, monotonically with
dose: held-out 60s joygate falls 1/48 (stopcur2) -> 4/48 (chg2) ->
8/48 (chg4, slip/m now over the 2.9 cap too); own-DR(0.5) det gait
5/6 (stopcur2) -> 2/6 (chg2, 3/6 video-confirmed leg-3 rigid-lock,
the SAME shortcut stopcur6 found at k=6.0, now reappearing at a lower
dose once the SPEED charge is doubled) -> an actual video-confirmed
tilt_roll FALL (chg4). Reward quarters decline in both (chg2
805/826/740/664, chg4 713/722/572/394, chg4 steeper) — aligned per
08-21, not undertrained; the charge bites, there is no basin left to
buy the cert with more of the same price.
**Same-cycle audit (`joyfullcurr10-stopsettle-probe`, INFORMATIVE,
diagnostic-only, no training):** built `goal.walk_stop_settle_s`
(default 0.0, additive-only metric, bit-exact-when-absent, 4/4 new
tests `test_walk_stop_settle_metric.py`) to test whether the cert's
`stop_speed_m_s` — which has always averaged EVERY stop tick from the
very first one of a commanded-stop segment — differs from a version
that excludes the same 0.4s grace window the reward's own stop charges
already exempt (`reward.walk_stop_grace_s`, built for exactly this
transient in the joyfullcurr7 dig-in). A `--walkcurr-precert-only` dry
run (no PPO, exits in ~15s) on the stopcur2 checkpoint's b1 bucket
read `stop_speed_m_s=0.0326` vs the new `stop_speed_settled_m_s=0.0311`
(`settled_frac=0.80`) — only a 5% drop, both ~2x the cap. **This
definitively closes the audit**: the residual creep is a genuine
POST-grace steady-state floor, not a cert-methodology artifact
(excluding the reward's own exempted window barely moves the number)
and not primarily a decel transient (80% of counted ticks were already
past it). Combined with the dose ladder's price-insensitivity, the
stop-speed-charge mechanism class is closed by BOTH price and
methodology — no further stop-charge dosing and no cert-measurement
fix is worth attempting. **Next specified-but-unbuilt lever**: a
structural anchor/hold gate on loaded-foot position during stop ticks
(reusing the existing `walk_anchor_gate` pattern, which today only
fires while `s_ref > 1e-3` — extend it, or an analogous new gate, to
price POSITION DRIFT of loaded feet during stop instead of
instantaneous body speed) rather than any further speed/current
pricing. Champion unchanged (`stotight45-seed13`); the core joystick
DONE gate stays met per 08-23; this V6 ladder remains operator-ordered
hardening (`fb_20260823T220651_5c66e3`). Evidence:
`logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr10_{chg2,chg4}_
{gate,owncfg,joygate}/` (chg4 owncfg `walk_det_1.mp4/.png`
video-confirms the fall), W&B runs `zi9qtmgu`/`nq6a26vc`/`xfx92c1i`.
Prior banner below.)

Previous entry (2026-08-24 ~06:3x (**`cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2`
VERDICTED PARTIAL (dose sibling of stopcur6, read jointly) — the k=2.0
stop-current charge keeps the over_current win with only a MILD version
of stopcur6's leg-sacrifice trade, but still does not touch the b1
cert.** Plain English: same mechanism as stopcur6 (charge sustained
per-servo current above headroom on stop ticks) at a lower dose.
Held-out 60s joygate falls: **1/48** (matches stopcur6's 1/48 exactly,
both crushing joyfullcurr6's 8/48 bar), the one remaining fall still
over_current but now a rare tail event, not the dominant mode; slip/m
2.178 (cap 2.9), dir_err 42.83deg (allow 40, narrowly misses, in line
with the whole ladder's history). DR-0 gate is fully clean (det 6/6,
sto 5/6 gait_valid, 0 terms) — no leg-3 sacrifice at DR-0, unlike
stopcur6's 4/6. Own-DR (0.5) shows the SAME leg-3 pathology stopcur6's
triage flagged, just milder: det 5/6 gait_valid, one episode (det/1)
sacrifices leg 3 (slip/m 6.63) exactly like stopcur6's fingerprint,
but only 1/6 vs stopcur6's 2/6-5/6-ish rate — **the leg-3 sacrifice
trade is dose-dependent and k=2.0 sits in a meaningfully safer regime
than k=6.0, not a clean zero**, worth remembering for any future
current-charge dose increase. On the walkcurr side: in-training
`walkcurr/b1_front45_20s/stop_speed_m_s` plateaus 0.027-0.048 m/s
across the full 40M run (79 cert rounds, no downward trend) —
identical band to joyfullcurr6/7/8 and to stopcur6 — because
`env/walk_stop_current_max_a` stayed 1.1-1.4A the whole run, under the
1.5A headroom the charge fires above: it is a no-op against a gentle,
low-current sustained creep, which is exactly what the cert measures.
`walkcurr/frontier` never leaves b1; buckets b2-b9 (side90/rear/
full-circle, the actual point of the operator's order) stay
unpracticed. Reward quarters 841/932/899/864 (peaks Q2, declines) =
plateau, aligned per 08-21 for the cert metric specifically (the
over_current axis is a genuine, separate win, not undertrained).
**Joint read with stopcur6**: both doses solve over_current
identically (1/48 each) and both leave the cert bar completely
unmoved — the current-charge mechanism's job is done (over_current is
fixed), and no further current-charge dose is worth spending; k=2.0 is
the safer operating point given the leg-3 finding.
**REFILL (this cycle): the old "do not dose up k_walk_stop_charge"
caution (recorded when over_current was still unsolved and a hard
brake was the feared failure mode) no longer applies now that a
proven, separate mechanism prices the fight directly** — launched
`cw-arch-hist16-dep1-c1-joyfullcurr10-chg2`/`-chg4`
(`k_walk_stop_charge` 1.0->2.0/4.0, `k_walk_stop_current` held at the
proven, safer 2.0, same base parent `ppo_goal_cw_arch_hist16_dep1_c1.zip`),
both VERIFIED RUNNING (train-0, train-1), to attack the cert-blocking
creep directly. Watch per-leg `sacrificed_legs` closely on both — a
higher speed charge could plausibly find the SAME leg-3-lock shortcut
to cheapen its own metric (locking a leg reduces measured body speed
too), which would be a new, orthogonal finding, not a repeat of the
current-charge story. If-false at both (creep dose-insensitive again):
close the stop-speed-charge-dose lever for good and audit whether the
0.015 m/s cert bar is achievable at all given actuator/contact
settle-time physics. Champion unchanged (`stotight45-seed13`); the
core joystick DONE gate stays met per 08-23; this V6 ladder remains
operator-ordered hardening (`fb_20260823T220651_5c66e3`). Evidence:
`logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr9_stopcur2_{gate,
owncfg,joygate}/`, W&B run `ft3d73yx`. Prior banner below.)

Previous entry (2026-08-24 ~06:2x (**`cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur6`
VERDICTED PARTIAL — the stop-CURRENT charge (k=6.0, 2x the speed-charge
cap) WORKS on its primary target but trades in a new pathology.**
Plain English: pricing sustained per-servo current on stop ticks
(instead of body speed) was hypothesized to fix the over_current
safety-trip falls that dominated joyfullcurr7/8 — it does, hard: held-out
60s joygate falls crashed **14/48 -> 1/48**, the best result in the
entire V6 stop-pricing lineage (previous best joyfullcurr6 8/48), with
slip/m 2.317 (cap 2.9) and dir_err 45.39deg. But two of the ledger's
other named bars stay unmet: (1) the V6 curriculum's own b1 cert
(`stop_speed_m_s <= 0.015`) never clears — `walkcurr/frontier` stuck
at 1 the entire 40M steps, `b1_front45_20s/stop_speed_m_s` oscillating
0.034-0.047 with no downward trend, because a current charge prices a
different physical quantity than the cert's body-speed metric and was
never expected to move it; (2) a NEW, video-confirmed leg-3 sacrifice
pathology appears, worst under domain randomization — own-cfg
(DR 0.5) det gait_valid crashed 5/6 (stopgrace parent) -> **2/6**,
DR-0 gate det also dropped 6/6 -> 4/6, always the same leg (index 3)
held rigidly aloft the whole clip (`walk_det_1.png/.mp4` vs the clean
6-leg `walk_det_0.png`). Reward quarters 848.8/908.1/855.4/811.4
(peaks Q2, declines) = aligned per 08-21, not undertrained. Read:
at 2x the speed-charge cap, immobilizing one joint is a cheaper way to
avoid the current charge than lowering peak current gait-wide,
especially exposed under DR. **NEXT**: joint dose read against the
k=2.0 sibling `cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2` (owned by
a concurrent cycle, unverdicted at this writing) decides whether the
lower dose keeps the over_current win without the leg-sacrifice trade;
if k=2 also sacrifices a leg, the fix needs a per-leg-fairness/
smoothness term alongside the current charge, not a dose retune, and
that would be a new DIG-IN. Champion unchanged (`stotight45-seed13`,
the core joystick DONE gate stays met per 08-23); this V6 full-circle
ladder remains operator-ordered hardening (`fb_20260823T220651_5c66e3`).
Evidence: `logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr9_stopcur6_
{gate,owncfg,joygate}/`, W&B run `eponx2n1`. Prior banner below.)

Previous entry (2026-08-24 ~04:4x (**joyfullcurr8 DIG-IN resolved: the
specified stop-CURRENT mechanism is BUILT, BANK-PROVEN and TRAINING.**
New `reward.k_walk_stop_current` (walk_task.py, tag
`exp/cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur`): on stop ticks only
(same s_ref/turn-in-place scoping as the speed charge), charges
per-servo current quadratically above a 1.5A headroom threshold
(`walk_stop_current_a`; trip is 2.5A sustained 0.8s), sharing the
0.4s `walk_stop_grace_s` timer so braking current pays little. Level
charge, not current-rate: the SafetyLayer trips on SUSTAINED level
through a ~0.1s LPF, so the over_current falls are a held isometric
fight against contact, not a spike a rate term would see. Bank:
`test_walk_stop_current.py` 5/5 — includes a scripted fight pose
(knees -40° into contact) that REPRODUCES the over_current
termination exactly and pays ~-4.7/tick while the relaxed plant
stance pays ~0 and stays the global optimum under the full
joyfullcurr stack; default-off bit-exact; grace 4/4 + stopcharge 3/3
unaffected. Launched the dose pair in one batch (08-22 batching
order): `joyfullcurr9-stopcur2` (k=2.0, train-0) and
`joyfullcurr9-stopcur6` (k=6.0 — prices one trip-level servo at
6/tick, strictly above the speed charge's 4.0/tick cap so no frozen
hard-brake can be reward-optimal, train-1), both VERIFIED RUNNING,
same base parent checkpoint, speed charge k=1.0 + grace 0.4
inherited. Pre-registered if-false at BOTH doses: the fight is not
reward-driven — audit the 0.015 m/s cert bar and the sim current
model before any further stop-pricing arm.) Previous entry
(joyfullcurr8-stopgrace FAIL, 04:1x): (**`cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace`
FAIL, verdicted — the settle-grace fix (built+launched last cycle
to answer joyfullcurr7's over_current finding) softens slip/direction
but does NOT touch over_current at all; the mechanism's own
pre-registered if-false branch fires.** Plain English: ramping the
stop-speed charge in over the first 0.4s (instead of applying it at
full strength immediately) was supposed to remove the pressure to
brake hard right after a stop command, which joyfullcurr7 showed was
tripping the actuator current-limit safety cutoff on every single
fall. It didn't: b1's stop cert still fails every certification round
through the full 40M budget (`stop_speed_m_s` plateaus 0.026-0.031
m/s, same shape as joyfullcurr7, never below the 0.015 cert bar;
`walkcurr/frontier` stuck at b1, buckets b2-b9 side90/rear/full-circle
still never practiced). Held-out 60s joygate: falls=**14/48** — worse
than the joyfullcurr6 bar (<=8/48) and flat vs joyfullcurr7's 13/48 —
and per-episode `term_reason` audit shows **100% of falls are still
over_current** (dr0 6/12+4/12, dr0p5 2/12+2/12), unchanged in kind
from joyfullcurr7. The grace window IS a real, non-null lever though:
slip/m improved 2.474->2.248 (well under the 2.9 cap) and dir_err
improved a lot, 45.67->**40.17deg** (allow 40.0 — a hair's-breadth
miss, closest any arm in this ladder has come). Standard
non-adversarial DR-0/own-DR walk gate stayed healthy (gate-side det
6/6 + sto 5/6 gait_valid, own-DR det 5/6 + sto 6/6, zero terminations
either report) — no general walking regression, this is narrowly the
harder joygate's stop-transition handling. Training reward quarters
858.4/941.5/906.3/880.4 (peaks Q2, declines) — aligned FAIL per 08-21
(plateau, not still-rising). **Root cause, confirmed exactly as
joyfullcurr7's own hypothesis pre-registered for this branch**: a
SPEED-based charge, however it's timed/ramped, prices the wrong
physical quantity — it changes WHEN in the stop segment the policy
feels pressure to decelerate, not the PEAK ACTUATOR EFFORT it spends
doing so, so it structurally cannot move a current-limit safety trip.
**NEXT (specified, not built — DIG-IN flagged, this is a new
reward-mechanism design+bank-proof task, not a triage-cycle-budget
edit)**: replace/augment the stop charge with a direct actuator
current (or current-RATE/jerk) charge on stop segments — servo
current is already read elsewhere in this file (`servo_current`
state, existing `reward.k_current` regularizer) so the plumbing
exists; the new piece is pricing it specifically and harshly enough
during commanded-stop segments, bank-proven under
`test_task_semantics.py` before any relaunch. Do NOT dose up
`k_walk_stop_charge` or `walk_stop_grace_s` further on this same
speed-based mechanism — both the dose axis (joyfullcurr7) and the
transient-timing axis (this run) are now tested and neither moves
over_current. Champion unchanged (`stotight45-seed13`); the joystick
DONE gate itself stays met per the 08-23 declaration — this V6
full-circle ladder is operator-ordered hardening
(`fb_20260823T220651_5c66e3`), not a re-open of the core mission
gate. Evidence: `logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr8_
stopgrace_{gate,owncfg,joygate}/`, W&B run `bm6vwexb`. Prior banner
below.)

Previous entry (2026-08-24 ~02:0x (**`cw-arch-hist16-dep1-c1-joyfullcurr7`
FAIL, verdicted — the stop-charge fix (landed for joyfullcurr7, see
banner below) is a REAL PARTIAL fix, not a no-op, but plateaus short
of the cert bar and trades away actuator safety margin.** Plain
English: last cycle's fix (charge the robot for still moving when
told to stop) DID reduce the creep speed a lot (0.045 -> ~0.026-0.031
m/s by ~40% of the run, vs joyfullcurr6's flat 0.041-0.047 the whole
way) but PLATEAUED there for the remaining ~35M steps, never reaching
the 0.015 m/s cert bar — `walkcurr/frontier` stayed pinned at 1 (b1),
identical to joyfullcurr6, so buckets b2-b9 (side90/rear/full-circle,
the actual point of the operator's order) were never practiced again.
Worse, the held-out randomized joygate got WORSE than joyfullcurr6
(13/48 falls vs 8/48; dir_err med 45.67 vs allow 40); newly
root-caused via per-episode `term_reason`: **every single fall in
both passes is `over_current`** (an actuator current-limit safety
trip), not roll/tilt — the per-tick linear stop-charge creates
pressure to decelerate hard the instant a stop is commanded, and the
joygate's harder, more frequent stop transitions turn that pressure
into current spikes that trip the safety limit. The standard
non-adversarial DR-0/own-DR walk gate stayed healthy and
video-comparable to joyfullcurr6 (6/6 det gait_valid 0 term, clean
six-leg gait) — this is a narrow, specific regression on the harder
panel, not a general walking collapse. ep_rew_mean quarters
828/928/898/864 (peaks Q2, declines) = aligned FAIL per 08-21, not a
keep-training case (the plateau, not still-descending, says
dose-insufficient-at-equilibrium).
**NEXT (specified, not yet built — flagged for a cycle with a clean
`walk_task.py`, see Now-banner note)**: do NOT blindly raise
`k_walk_stop_charge` dose — the over-current finding says more dose
intensifies the hard-brake incentive and likely worsens falls, not
the plateau. Build a **settle-grace** on the stop charge instead:
exempt or linearly ramp-in the charge over the first
`reward.walk_stop_grace_s` (~0.3-0.5s, new cfg key, default 0 =
off/bit-exact) after a stop command begins — the physically-necessary
deceleration transient pays nothing or a fraction, only SUSTAINED
post-transient creep pays the full charge. This addresses both
findings at once: removes the hard-brake pressure that's tripping
over_current, while keeping (or strengthening) the pricing on the
actual cert-bar violation (sustained creep). Needs: a small
`_walk_stop_grace_s` timer/EMA analogous to the existing
`_walk_idle_ema` state, a `test_task_semantics.py` bank addition
(stopcharge-with-grace: still_charged>creep_charged still holds,
grace window itself pays less than an equal-duration sustained creep
at the same speed), then relaunch `cw-arch-hist16-dep1-c1-joyfullcurr8`
from the SAME base parent (`ppo_goal_cw_arch_hist16_dep1_c1.zip`, not
this regressed checkpoint) with the grace mechanism replacing the
plain charge. NOT attempted this cycle: `rl_move/sim/walk_task.py`
had a live uncommitted concurrent edit in flight (a different
mechanism, walkcurr's foot-contact/idle-termination work) at
triage time — editing the same file simultaneously risked losing
either change; check `git diff --stat walk_task.py` is clean before
starting. Champion unchanged (`stotight45-seed13`); the joystick
DONE gate itself stays met per the 08-23 declaration — this V6
full-circle ladder is operator-ordered hardening
(`fb_20260823T220651_5c66e3`), not a re-open of the core mission gate.
Evidence: `logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr7_{gate,
owncfg,joygate}/`, W&B run `6l0afxr1`. Prior banner below.)

Previous entry (2026-08-24 ~00:1x (**`cw-arch-hist16-dep1-c1-joyfullcurr6`
VERDICTED FAIL (dig-in complete) — root cause CORRECTED, stop-charge
fix landed, relaunch queued.** The dig-in finalized the flagged run:
joygate FAIL (8 falls/48, 5 at DR-0, dir_err 40.69>40, gait_valid
0.917 — the parent lineage passed this gate 0 falls, so 40M steps of
b1-only churn REGRESSED the base gait), own-DR walk FAIL (det 1/6
over_current termination, gait_valid 3/6 det, dir_err med 42.1; sto
5/6 gait_valid, slip 1.61), checkpoint kept append-only, NOT a
champion candidate; champion unchanged (stotight45-seed13).
**ROOT-CAUSE CORRECTION to the banner below:** the earlier triage
blamed `walk_freeprog_score`'s 0.06 stop cap — but `k_walk_freeprog`
was NEVER ACTIVE in this run (default 0.0, not in the cfg-set list,
no freeprog keys in the W&B history). The real defect, verified in
code: on commanded-stop ticks (s_ref ~ 0) EVERY walk-mode term except
the Gaussian velocity kernel is guarded by `s_ref > 1e-3`
(prog gate, k_walk_course, park-duty, step/drag, idle), and the
kernel's stillness-vs-creep margin is shallow (2.0/tick still vs
1.45/tick at the converged 0.04 m/s creep) — the V6 cert stop bar
(mean stop-tick speed <= 0.015 m/s incl. decel transient,
walk_task.py stop_v_sum accounting) had NO matching reward optimum,
so PPO's converged creep was priced as near-optimal. **FIX LANDED
(2026-08-24):** new `reward.k_walk_stop_charge` (default 0 = off,
bit-exact; scale `reward.walk_stop_scale_m_s` default 0.015 = the
cert bar, cap `walk_stop_charge_cap` 4.0) charges stop-tick body
speed linearly against the cert's own scale — stillness pays 0, the
observed creep pays ~2.7k/tick, driving through the stop pays the
cap; turn-in-place ticks exempt; NOT in the walk_charge_ramp trio.
Bank-proven under the run's exact reward stack
(test_task_semantics.py stopcharge bank: still_charged > creep_charged
by >300/ep, creep > walk-through-stop, stillness itself untaxed;
walk bank 62 PASS, test_walk_curriculum.py 38/38). Relaunch
`cw-arch-hist16-dep1-c1-joyfullcurr7` from the SAME parent
(`ppo_goal_cw_arch_hist16_dep1_c1.zip`, NOT the regressed
joyfullcurr6 checkpoint — phasedir9-vs-9b precedent) = joyfullcurr6
recipe + k_walk_stop_charge=1.0. Infra nit for the watcher: the
prestaged session eval crashed on obs dim (1152 vs 72 — harness
didn't apply obs.history_frames=16); gate DR-0 pass was still
running at verdict time, artifacts will land in
`logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr6_gate`. Prior
(superseded on the freeprog point) banner below.)

Previous entry (2026-08-23 ~23:5x: **`cw-arch-hist16-dep1-c1-joyfullcurr6`
FINISHED its full 40M-step budget STUCK AT THE SECOND RUNG OF A
10-BUCKET LADDER — DIG-IN FLAGGED, left unverdicted.** Plain English:
the operator-ordered full-circle joystick curriculum (fb_20260823T220651_5c66e3)
was supposed to climb bridge -> front45 -> side90 -> rear135/180 ->
full-circle over the run; it promoted ONCE (b0->b1 at step 540k, round
r1) and then FAILED bucket b1 (`front45_20s`) on its `stop` check on
EVERY SINGLE certification round from r2 through the end of training
(~80 rounds spanning the full 40M steps) — buckets b2-b9 (side90
onward, the entire point of the operator's order) were NEVER PRACTICED
AT ALL. Training reward converged early (ep_rew_mean quarters 913.9 /
1062.0 / 1054.1 / 1042.6 — flat after Q1, not still rising), so this
is NOT an undertrained-keep-going case per the 08-21 ruling; it is a
genuine reward<->eval mismatch. Root cause (pinned via W&B history,
no extra training cost): b1's cert gate requires `stop_speed_m_s <=
0.015` m/s during the bucket's commanded-stop segments, but the
run's own `walkcurr/b1_front45_20s/stop_speed_m_s` metric sat flat in
[0.036, 0.047] for all 79 logged cert rounds (no downward trend) —
2.4-3x over the cert threshold, essentially motionless numerically
but never converging to true stillness. The recipe's active
stop-pricing term is `walk_freeprog_score` (`k_walk_freeprog`,
inherited from the parent, not touched by this run's cfg-set list),
whose own stop-command formula charges speed against
`walk_freeprog_cap_m_s` (default **0.06** m/s) — 4x LOOSER than the
cert's 0.015 m/s bar — so the optimizer has genuinely converged to a
local optimum (creep ~0.04 m/s) that is priced as "good enough" by
the reward but fails the gate. The hypothesis's own text says
`k_walk_cmd_track` (the ONE reward term in the codebase with a
dedicated, separately-configurable `stop_speed_m_s` parameter,
`walk_cmd_track_score` in `walk_task.py`) was "omitted to keep the
coupled-change count down" — that omission is now the leading
suspect for the fix, not a free variable. **Fork this decides:**
before spending any more budget on the V6 ladder (side90/rear/full-
circle are all still completely untested), either (a) tighten
`walk_freeprog_cap_m_s` to <=0.015 for stop segments (or add a
dedicated low-cap stop charge), or (b) turn on `k_walk_cmd_track`
with `stop_speed_m_s=0.015` matching the cert bar, re-prove whichever
lever against the WALKCURR-family bank discipline (this track hasn't
had one for cmd_track/freeprog stop-pricing specifically — check
`test_task_semantics.py` before relaunch), then relaunch from the
SAME parent (`ppo_goal_cw_arch_hist16_dep1_c1.zip`) rather than
continuing the frozen-at-b1 checkpoint (phasedir9-vs-9b precedent:
continuing a converged-in-the-wrong-basin checkpoint does not escape
it). Checkpoint IS saved
(`ppo_goal_cw_arch_hist16_dep1_c1_joyfullcurr6.zip`, plus the
promotion artifact `..._joyfullcurr6_promo_b1.zip`), gate/own-DR/
joygate evals were prestaged and were still running on
hexapod-mjx-train-4 as of this writing (`ops.sh waitlog` on
`/tmp/eval_cw-arch-hist16-dep1-c1-joyfullcurr6*.log`) — read those
before writing the formal verdict (this run's b0-only behavior may
still cleanly pass the OLD front-cone-only DONE gate even though the
new full-circle order's own bar was not met). Evidence:
`logs/experiments/cw-arch-hist16-dep1-c1-joyfullcurr6/wandb_history.csv`
(`walkcurr/b1_front45_20s/stop_speed_m_s` column), pod train log
`/tmp/train_cw-arch-hist16-dep1-c1-joyfullcurr6.log` (grep `walkcurr
cert`). Prior banner below.)

Previous entry (2026-08-23 (**DONE-GATE DECLARED MET (assume-and-go,
cycle c0823-seed37-triage) — track goal achieved, work is
maintenance-only.** q_20260822T1730Z sat unanswered for a full day
across many cycles that kept re-verifying (2 held-out command-seed
bases, 4 training seeds, an on-distribution command-training lever)
without ever executing the promotion the evidence already supported —
that non-decision was itself becoming a parked line. The pre-
registered gate (60s randomized joystick script, MuJoCo, zero falls,
directions followed, slip/m <= ~2.9 teacher band, n>=12 det+sto, DR-0
+ own-DR) is met as written: `stotight45` passes on 4/4 training seeds
(17/23/13/29) and 2/2 held-out command-seed bases, n=48 each, zero
falls, gait_valid 48/48, slip 2.41-2.78, dir_err 36.4-39.4deg (allow
40). **Champion promoted: `stotight45-seed13`**
(`ppo_goal_cw_dep_bcgait4_phasedir9_stotight45_seed13.zip`, slip/m
2.407, dir_err 36.4deg — widest margins of the 4 passers). No untried
margin lever remains (`cmdmix` CLOSED 0/3 PASS below). Honest residual
caveats that do NOT block the declaration (the gate text doesn't name
them): thin own-DR sto margins on some seeds, a legacy
`bc_anchor_knee_abs=1.0` dialect + `walk_phase_obs` (+2 dim) contract
the hardware runner must match, det progress a touch softer than the
pre-noise-floor `longrun17` checkpoint, and training itself was
fixed-forward-only (the gate's reverses/turns/stops are emergent
generalization, measured not assumed). Full evidence trail in
`CURRENT_TRUTHS.md` and the banners below. Operator override welcome
(`OPERATOR_QUESTIONS.md` q_20260822T1730Z); absent one this stands.
Fleet registered-goal effort now concentrates on `amp`/`cpg`. Prior
banner below.)

Previous entry (~19:5x: **CMDMIX LEVER CLOSED — on-
distribution command training is ANTI-PRODUCTIVE on BC-anchored
recipes (batch 0/3 PASS, 2 FAIL).** The 3-basin batch (each seed's
best dose retrained with the gate's own stress_mix command family,
training rng only) landed on its pre-registered worst branch:
`cmdmix45-seed13` FAIL (joygate slip 2.407→3.023 over the 2.9 cap,
dir 36.4→41.26 over the 40 allow), `cmdmix50-seed23` FAIL and worst
(slip 2.543→3.595, dir 35.31→43.67), `cmdmix55-seed17` INFORMATIVE
(only evaluator-keeper: pass=true but every margin shrank — slip
2.515→2.817 w/ own-DR 2.897 grazing the cap, dir 34.97→36.73, 15s det
prog 0.70→0.55). Zero falls / gait 48/48 on all three — pure margin
regression, reward rose on all three while gates worsened: the
distribution change itself is the misalignment (mid-episode churn
fights the BC anchor); emergent transfer from fixed-command training
already beats practicing the transitions. No continuation, lever
CLOSED (4th arm `cmdmix45-seed29` FINISHED ~19:5x: FAIL, joygate
slip 3.043>2.9 / dir 39.45, zero falls, same pure margin regression —
batch FINAL 0/4 PASS, 3 FAIL / 1 INFORMATIVE). **CHAMPION UNCHANGED:
stotight45-seed13 (slip 2.407/dir 36.4).** With dose ladder, ramp,
loadslip family, anneal-continue, and now command-mix all measured-
closed, no untried margin lever remains on this lineage; DONE-gate
pass stands reproduced 4/4 seeds, formal gate-green awaits operator
ack (q_20260822T1730Z). Prior banner below.)

Previous entry (~19:4x: **MEASURED-PLANT GATE BREAK stance
half CLOSED.** The corrected-cfg tibia-150 session-gate rerun flagged
by the `plant150-3-rsifix` PASS is done: rsifix stance x
`cw-dep-bcgait1-plant150-1` walk (`actions.max_height_mm=137`,
`bus.servo_params=loaded`) passes ALL hard gates — no falls (the
prestage's rise over_current FELL was the default harness pairing
vref1_r1 + stale cfg, not the checkpoint), rise 170.3 mm, sit
descends — plus 5/7 soft incl. fwd_heading 8.8 deg (plant150-1's own
earlier -10.6 miss now clean with this partner); only
track_right/track_back miss, the documented all-model weakness.
Strip watched: clean rise/cruise/sit/re-stand. Evidence:
`logs/ckpt_eval/rsifix_plant150pair_session_corrected/`. The
tibia-150 download pair = rsifix + plant150-1, both halves
session-clean on hard gates. Prior banner below.)

Previous entry (~19:3x: **STOTIGHT DOSE SEARCH COMPLETE — all four
verdicts in, ladder closed, champion final.** The two in-flight reads
landed: `stotight60` (seed17 × -6.0) INFORMATIVE — KNEE FOUND,
margins regress well beyond noise (slip 2.515→2.823, dir 34.97→37.68,
own-DR slip 2.873 grazing the cap) though the evaluator still passes
and det holds (0.70/2.01); seed17's endpoint is -5.5.
`stotight55-seed13` INFORMATIVE — dose and basin do NOT stack (slip
2.407→2.594, dir 36.4→39.72, own-DR-alone dir 40.4); with
stotight50-seed13 this brackets seed13's knee AT -4.5. **CHAMPION
CANDIDATE FINAL: seed13@-4.5 (`stotight45-seed13`, slip 2.407/dir
36.4).** Per-seed knees: seed13→-4.5, seed17→-5.5, seed23→-5.0,
seed29→-4.5. No further stotight arms. Also cleaned a stale triage
leak: `phasedir9-seed42` (old -3.2-era seed arm) FAIL,
reward-collapsed, question superseded. **NEXT LEVER LAUNCHED (Next
item 5, 3-arm batch, ~19:2x): `cmdmix45-seed13` / `cmdmix55-seed17` /
`cmdmix50-seed23`** — every passer trained on ONE fixed forward
command yet is graded on turns/stops/reverses; these train each
best-per-seed recipe on the gate's own command family
(walk_cmd_mode=stress_mix, resample 4.0s ±0.5, training rng only,
held-out gate seed base 90000 untouched). Pre-registered batch read:
3/3 PASS = general lever; 1/3 = basin lottery again; FAILs = command
churn breaks the BC-anchored basin. Prior banner below.)

Previous entry (~19:1x: **DOSE TRANSFER SPLIT — the -5.0
rung CLOSES seed23's gap but WORSENS seed29's.** `stotight50-seed23`
VERDICTED PASS on its pre-registered bar: joygate slip 2.543/dir
35.31° (vs seed23@-4.5's 39.4°), own-DR-alone dir 36.94° — the 40.36°
over-allowance gap that motivated the arm is closed by 3.4°; 0
falls/48, gait 48/48, no det trade (15s DR-0 det prog 0.69/slip
1.80), clean six-leg sheet. `stotight50-seed29` VERDICTED
INFORMATIVE: evaluator still passes (0 falls/48, gait 48/48) but slip
went the WRONG way — combined 2.748 vs its -4.5 reading 2.704, own-DR
sto slip 2.986 over the 2.9 cap on that pass alone, det weakest of
the batch (0.63/2.31). Reward rose and converged on both — honest
basin answers. **Conclusion: dose response is seed-basin-specific and
does NOT track gap type** (seed23's dir gap closed; seed29's slip gap
widened). Per-seed bests now fully mapped: seed13@-4.5 (CHAMPION
CANDIDATE, slip 2.407/dir 36.4), seed17@-5.5 (best dir 34.97),
seed23@-5.0 (slip 2.543/dir 35.31), seed29@-4.5 (slip 2.704). Blanket
per-seed dose sweeps are DONE — no further stotight dose arms except
the two in-flight reads (`stotight60` knee search, `stotight55-seed13`
stack test; evals running ~19:1x). Prior banner below.)

Previous entry (~18:5x: **THE DOSE LADDER IS SEED-SPECIFIC —
`stotight50-seed13` (best basin seed13 × -5.0) VERDICTED INFORMATIVE:
still passes the 60s DONE-gate (0 falls/48, gait 48/48, clean six-leg
video) but with WORSE margins than seed13's own -4.5 reading — slip
2.407→2.63, dir 36.4→38.21 (own-DR slip 2.763/dir 39.59); the
pre-registered PASS bar (slip ≤ 2.407) was missed and even the
INFORMATIVE ceiling (2.569) exceeded. Det did NOT soften (15s DR-0
det prog 0.79/slip 1.61, strong) and reward rose all run — reward and
gate agree; the -5.0 rung's gains are seed17-basin-specific, not
universal. **Champion candidate unchanged: seed13@-4.5 (slip
2.407/dir 36.4) keeps the fattest margins.** Follow-up batch
launched: `stotight50-seed23` / `stotight50-seed29` — does the deeper
floor transfer to the two seeds holding the named hardening gaps
(seed23 own-DR-alone dir 40.36; seed29 own-DR slip 2.736), or is the
ladder purely a seed17 phenomenon? Prior banner below.)

Previous entry (~18:4x: **LADDER STILL PAYING AT -5.5 —
`stotight55` (seed17, log-std-final -5.0→-5.5, final std ~0.004)
VERDICTED PASS on the 60s joystick session gate with the BEST
direction-following of any passer: slip 2.515 (parent 2.569), dir
34.97° (parent 38.02°, near the teacher floor ~35), own-DR pass slip
2.542/dir 33.7, 0 falls/48, gait 48/48, no sacrificed legs; det
improved yet again (15s DR-0 det prog 0.74/slip 1.63 vs parent
0.69/1.72) — five monotone rungs (-3.6/-4.0/-4.5/-5.0/-5.5) and the
det trade has never materialized past -4.5. Slip step is shrinking
(-0.102 → -0.054/rung) but the 3° dir jump is well outside the prior
rung's 0.6° step — not saturated. Follow-ups launched per the PASS
branch: `stotight60` (seed17, -6.0 — knee search continues) and
`stotight55-seed13` (best basin × this dose — champion-candidate
margins; seed13@-5.0 still training on train-4). Prior banner
below.)

Previous entry (~18:2x: **DEEPER NOISE FLOOR WIDENS EVERY
MARGIN — `stotight50` (seed17, log-std-final -4.5→-5.0) VERDICTED
PASS on the 60s joystick session gate: slip 2.569 (parent 2.671),
dir 38.02° (best of any passer), own-DR slip 2.623 (vs 2.736), 0
falls/48, gait 48/48; and the feared det-progress trade did NOT
appear — 15s det prog 0.69/slip 1.72 vs parent 0.65/1.83, det
IMPROVED. Dose ladder -3.6/-4.0/-4.5/-5.0 still monotone. Follow-ups
launched per the gate's own PASS branch: `stotight55` (seed17,
-5.5 — find the knee) and `stotight50-seed13` (best-basin seed ×
deeper dose — fattest-margin champion candidate, target slip <
seed13@-4.5's 2.407). Prior banner below.)

Previous entry (~18:1x: **DONE-GATE PASS REPRODUCES ON
EVERY TESTED SEED — recipe 4/4, seed-robustness question CLOSED.**
`stotight45-seed13` and `-seed29` both VERDICTED PASS on the full
60s randomized joystick session gate, completing the n=4 sample:
seed17 (original) slip 2.671 / dir 38.6; seed23 2.78 / 39.4; seed13
2.407 / 36.4 (widest margins of any passer); seed29 2.704 / 39.05.
All four: pass=true, 0 falls/48, gait_valid 48/48, no sacrificed
legs, videos watched det+sto both DR (clean upright six-leg gait);
training reward rose all run with std annealed to 0.011 on every
seed — reward and gate agree everywhere. Seeds 13 and 29 were the
lineage's two historically WORST basins at the -3.2 dose (1/4 pass
rate there) — the -4.5 noise floor converted a seed lottery into a
reproducible recipe; recipe property, not seed luck. The
seed-reproduction bar implied by q_20260822T1730Z is met; formal
gate-green + champion promotion remain operator-confirmed. Honest
residual margin gaps if the operator wants hardening: own-DR sto
margins (thinnest: seed29 slip 2.736/2.9, dir 39.4/40; seed23
own-DR-alone dir 40.36 a hair over on its own) and the 15s-rung det
progress trade (~0.85x clone) shared by the -4.5 passers. Prior
banner below.)

Previous entry (~17:5x: **THE DONE-GATE PASS REPRODUCES ON
A SECOND SEED** — `cw-dep-bcgait4-phasedir9-stotight45-seed23`
(identical stotight45 recipe, only seed 17→23) VERDICTED PASS on the
full 60s randomized joystick session gate: pass=true, 0 falls 48/48,
gait_valid 48/48, no sacrificed legs, slip 2.78 (cap 2.9), dir 39.4°
(allow 40); clean six-leg video at both DR scales. Margins thinner
than seed17 (own-DR-alone dir median 40.36°, a hair over the
allowance on its own; combined gate passes).)

Previous entry (~17:3x: **DONE GATE PASSED FOR THE FIRST
TIME** — `cw-dep-bcgait4-phasedir9-longrun17-stotight45` (fresh
reinit of the longrun17 recipe, only `--log-std-final` -3.2 -> -4.5,
final std 0.011) passes the full randomized 60s joystick session
gate: versioned evaluator pass=true, n=48 held-out episodes, ZERO
falls, gait_valid 48/48, no sacrificed legs, combined slip 2.671
(cap 2.9), dir_err 38.6deg (allow 40), and every mode individually
under caps (worst: own-DR sto slip 2.859). Sto slip fell
monotonically across the -3.6/-4.0/-4.5 dose grid (own-DR 15s-panel
3.00/2.87/2.48; siblings near-miss the session gate at 40.7deg dir /
2.94 slip). Videos watched (video-joygate rerun, det+sto, both DR):
clean upright six-leg alternating gait. Honest caveats: det softened
vs longrun17 (session det slip 2.30->2.55, dir 34.7->37.6, still
under caps; 15s rung prog 1.02x->0.85x clone) — the expected
noise-floor trade; own-DR sto margins thin. Checkpoint
`ppo_goal_cw_dep_bcgait4_phasedir9_longrun17_stotight45.zip` pulled
to controller, md5 9fb86d18 pod==controller. PROMOTION + formal
gate-green declaration flagged for operator confirmation
(OPERATOR_QUESTIONS q_20260822T1730Z); q_20260822T1520Z's assumed
answer (sto gap is a policy property fixable by dose) is CONFIRMED.
Same cycle: RAMP LEVER CLASS CLOSED — the 4-arm
allowramp2{slow,wide}-seed{23,29} grid verdicted 3x worse-on-both-
axes + 1 noise-edge nominal improver (slow29); no dose generalizes
across seeds; judgment closure, not the literal 0-for-4 trigger —
see Next item 3.)

Previous entry (08-22): (`-longrun17-cont1` VERDICTED FAIL on its
pre-registered prediction-if-false: the +4M continuation with std
held at 0.041 RETAINED longrun17's full det DONE-gate pass (det slip
2.38/2.65 vs cap 2.9, dir 34.5/37.6 vs 40, 0 falls 48/48) but sto
stayed out (slip 3.93 DR-0 / 4.36 own-DR, dir 50-51deg) while
training reward rose the whole run (91.6->225.6). CONTINUATION/
BUDGET LEVER CLOSED for the sto axis — sto is a reward/eval
divergence, not undertraining. Live probe: the stotight
log-std-final dose grid (-3.6/-4.0/-4.5, all finished, triage
pending); if sto slip stays ~4 across doses, next is train-time
sto-robustness (perturbation training or sto-aware pricing). Also
settled this day: recipe seed pass rate 1/4 (seed17 only;
13/23/29 FAIL) — "seed lottery too thin to farm"; allowramp
generalization REFUTED on seeds 23+29.)

Previous entry (08-22, operator-ordered FORMAL SESSION-GATE
reading on `-longrun17`, after the operator live-accepted the
checkpoint under real joystick input on the Mac viewer: the full 60s
randomized DONE gate (`eval_joystick_gate.py`, held-out stress_mix,
n=12 det+sto at DR-0 AND own-DR 0.35) is **FAIL overall but
DET-ONLY PASSES EVERY AXIS at both DR scales** — zero falls 48/48,
gait_valid 48/48, det slip 2.30 (cap 2.9), det dir_err 34.7deg DR-0
/ 37.4deg own-DR (allow 40). The sto half alone fails it (slip
4.0, dir 51-52deg -> combined medians 3.325/45.7). The
operator-live-vs-gate delta is RESOLVED as det-vs-sto, NOT command
distribution: the det policy follows held-out reverses/stops/turns
it never trained on (trained forward-only fixed 0.08!) and beats the
phase clone's own session reading ~5x on slip (15.9->3.3) and ~20deg
on direction. PROMOTION NOT EXECUTED (operator conditional was
pass-gated); sto-calibration question filed
(OPERATOR_QUESTIONS q_20260822T1520Z, assumed answer: gate stands,
sto gap is a policy property -> sto-robustness arm after
longrun23/29 settle the seed pass rate). Presentation caveat
(fb_20260822T145428): the phasedir lineage deliberately trains with
the legacy `bc_anchor_knee_abs=1.0` dialect — do not present it as a
clean current-convention imitation line. Artifacts:
`logs/ckpt_eval/longrun17_joystick_session_gate_v1/`.

Previous entry (08-22, CORRECTION to the entry below it:
`phasedir9-longrun17`'s FAIL verdict was written before its own eval
finished syncing — a premature-verdict race, not a real reading.
Recomputed from the synced report + this run's own W&B summary
(triple-confirmed), `-longrun17` (seed17) is actually a **DET-mode
PASS** of the rung-A clone-relative gate — the first in 34+ arms —
at both DR-0 (progress 1.02x clone, slip 0.74x, speed 0.069 m/s) and
its own DR-0.35 (0.94x/1.01x/0.067, thinner). `-longrun13` (seed13)
stays correctly verdicted FAIL/worse (0.792x/1.286x). So the "budget
lever is closed end-to-end" conclusion below is WRONG for seed17 and
right for seed13 — a genuine seed-dependent divergence on an
identical recipe, not a settled answer either way. Ledger set to
`PASS (partial)` pending: reproduce `-longrun17` independently before
any promotion (`-longrun29` seed29 RUNNING), and root-cause the
seed13/seed17 divergence. **BC-anchor/phase-lock family-boundary
DIG-IN RESOLVED same day** (per-tick trace,
`rl_move/sim/trace_bc_cadence.py`,
`logs/ckpt_eval/pd9seed17_bc_cadence_trace`): NO cadence gap exists —
policy, clone AND raw teacher all cycle at 0.76 s == bc_target ==
the 0.75 s phase clock (TripodGait.period=0.75 == 1/walk_phase_hz by
construction). The "swing_s_mean ~30% slower" premise was a
contact-segmentation ARTIFACT: the clone's 0.25 s "swing" is double
lift-offs per cycle (24-36 lifts/leg/15 s vs ~19 cycles, lift-to-
lift 0.44 s vs 0.76 s) splitting its swings; the policy single-
swings cleanly at 0.372 s vs the raw teacher's realized 0.345 s
(+8%). Supervision is honest in the eval regime (policy-vs-bc_target
MSE 0.00136; the clone's own is 0.00549; xcorr ~1.00 at the same
~3-tick servo lag). LEVER CLASS CLOSED: anchor dose, walk_phase_hz,
phase-lock plumbing all exonerated. The seed divergence is therefore
NOT a supervision defect — it is the known init/seed-basin lottery
on a reward surface that is ~flat across honest and drag basins at
annealed-low std; the residual rung deficit on failing seeds is
loaded-foot slip during stance at MATCHED gait timing/stride/duty
(seed17 slip 2.85/m vs clone 1.89). Next budget = measure the
recipe's seed pass rate (longrun29 + longrun23), then promote the
best det passer. Superseded text below kept for the lineage record;
treat its "FORK RESOLVED/EXONERATED end to end" language as void.

Previous entry (08-22, phasedir9-seed17 VERDICTED FAIL-as-
reproduction): pd9-stdanneal's 2M near-pass (0.873x progress, 1.08x
slip) did NOT reproduce on a second seed. FOLLOW-UP `phasedir9-
longrun13`/`-longrun17` (fresh re-inits, NOT continuations — same
stacks/seeds, --steps 2M->4M, anneal still ending ~1.2M so ~2.8M
steps run converged vs ~800k): longrun13 (from the GOOD seed) got
WORSE (progress 0.873x->0.792x clone, slip 1.08x->1.286x); longrun17
(from the BAD seed) — SEE CORRECTION ABOVE, this is now a PASS
(partial), not "stayed FLAT". Zero falls, gait_valid 6/6, clean video
on all four runs (stdanneal/seed17/longrun13/longrun17). Condensed
08-22 for the <=120-line budget — full lineage detail lives in
`RL_LOG.md` + ledger verdicts, not here. Keep this a short screenful:
Goal / Now / Next.

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

- **08-23 operator order (fb_20260823T220651_5c66e3): hist16
  full-circle joystick curriculum.** New `WALKCURR_BUCKETS_V6`
  ladder landed + tested (bridge -> front45 -> side90 -> rear135/180
  -> full-circle 60s -> DR 0.2/0.5; snapshot e180b161). Run
  `cw-arch-hist16-dep1-c1-joyfullcurr6` RUNNING on train-4, warm from
  `ppo_goal_cw_arch_hist16_dep1_c1` + loadslip/course reward; precert
  B0 PASS pre-PPO (prog 1.164, 0 falls, slip/m 0.89), promoted to b1
  @540k. Gate: eval_drive full-circle heading-max 180 DR0.5, 0 falls,
  rear/side directions followed, slip sane. No duplicate seeds unless
  primary fails pre-cert or triage demands it (operator).

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
  the reward was not aligned with the eval. Per the 08-21 ruling
  those are MISALIGNMENT results, not dead ends. `phasedir1` is
  additionally ENV-CONFOUNDED (convention-corrupted sim).
- **phasedir2-8 lineage (full detail: RL_LOG + ledger verdicts;
  every arm zero falls, gait 6/6, clean video)**: eight consecutive
  FAILs of the aligned-reward stack on the clone-relative rung-A
  gate, each refuting one lever class in turn — staged curriculum
  (obedient but slow, 0.836x progress), loadslip reprice (slip
  unpriced 1.41x), ent-coef anneal (std barely moved), warm-log-std
  override (progress PASS 0.984x but SLIP-FINANCED — drag family,
  dig-in-pinned), band retighten (VALUE lever refuted), k_drag_stance
  8000/4000 (STEP FUNCTION, identical slow optimum both doses), and
  finally phasedir8's stride-EMA kernel (allow 24, `walk_kernel_vel_ema`)
  which still missed (prog 0.770x, best of lineage to that point) —
  dig-in found the det/DR-0 pricing calibration DOES NOT TRANSFER to
  the noisy optimization regime (no separating allowance exists
  between the honest noisy tail and the det drag cheat under PPO
  exploration). REPAIRED via `train_ppo_mjx --log-std-final/
  --log-std-anneal-frac` (forced noise-anneal so std converges to
  the det regime where pricing IS measured-aligned) +
  `reward.walk_course_overspeed_ref_floor_m_s` (ramp-drift insurance).
  Bank 34/34.
- **phasedir9 (anneal from raw BC clone) vs phasedir9b (anneal
  continuing the pd8 cheat-committed checkpoint), same stack+seed**:
  `-9` UNDERTRAINED/near-pass at 2M (zero falls 24/24, gait 6/6,
  slip/dir_err/speed all inside gate, progress 0.873x clone — best
  of lineage, narrow miss of 0.9x cap, reward+drag-charge still
  moving). `-9b` VERDICTED FAIL: WORSE than pd8 itself on every axis
  (prog 0.704x, slip 1.506x, speed 0.054) despite drag charge
  falling 4x — dodged the bill by shrinking per-stance travel, not
  slipping less (walking in place). ROOT CAUSE (dig-in resolved):
  INIT-BASIN SELECTION, not checkpoint recency — `-9b`'s final
  TRAINING reward is equal-or-better than `-9`'s, so this reward
  stack's optimization-regime surface is ~FLAT across the honest and
  drag basins at annealed-low std; PPO is a local polisher and INIT
  decides which basin gets polished. **LINEAGE RULE (binding for this
  reward stack): pricing/regime repairs re-init from a pre-cheat
  checkpoint (teacher/clone), never continue-train an
  already-converged one.**
- **phasedir9-cont1 VERDICTED FAIL as a continuation** (+4M steps
  from the `-9` checkpoint): regressed hard on every clone-relative
  axis (progress 0.873x->0.66-0.71x, slip 1.08x->1.6-1.7x, speed
  dropped below the 0.06 floor), zero falls/gait-6-6 preserved. W&B
  ep_rew crashed 27->-670 by ~1.4M then only partly recovered to
  152-213 by 4M — collapse-then-partial-recovery, not steady
  climbing. Same mechanism as `-9b`, opposite direction: a GOOD init
  drifted into a WORSE basin because the reward surface can't tell
  them apart at annealed-low std. `-9` itself is unaffected, still
  the lineage's best 2M reading.
- **phasedir9-seed17 VERDICTED FAIL-as-reproduction** (08-22): the
  reproducibility replicate landed at/below pd8's own level (progress
  0.727x clone vs pd9's 0.873x and pd8's 0.766x; slip 1.27x vs pd9's
  1.08x and pd8's 1.254x), matching the pre-registered
  prediction-if-false almost exactly ("lands far below pd9... back
  near pd8 or worse"). Zero falls/gait-6-6 preserved both seeds.
  READS AS: pd9's near-pass was partly seed luck riding the
  log-std-anneal fix, not a reliably repeatable recipe — do not
  trust a single good seed's numbers as the lineage's ceiling.
  TELEMETRY LEAD for the dig-in this decides (see banner): W&B
  `train/bc_anchor_loss_walk` is already tiny (0.00005-0.0007) and
  `env/walk_anchor_frac` already high (0.80-0.93) on BOTH seed13 and
  seed17 — the phase-locked BC-anchor aux loss reads as converged —
  yet BOTH seeds' realized `swing_s_mean` runs ~30% slower than the
  teacher/clone (0.34-0.36s vs 0.25-0.27s). A near-zero action-space
  anchor loss coexisting with a large realized-cadence gap points at
  stochastic-action/servo-slew/plant realization as the boundary, not
  supervision strength — raising `train.bc_anchor_coef` blind is
  unlikely to help and was NOT attempted. LAUNCHED (fresh re-init
  from the raw BC clone, NOT a continuation — the lineage rule is
  intact) `phasedir9-longrun13`/`-longrun17`: same stacks/seeds as
  stdanneal/seed17, only `--steps` 2M->4M and
  `--log-std-anneal-frac` 0.6->0.3 (anneal still ends at the same
  absolute ~1.2M step; ~2.8M steps in the converged regime instead of
  ~800k) — tests the 08-21 "needs to go longer" branch on both seeds
  before committing to the phase-lock dig-in's on-pod tracing work.
- **phasedir9-longrun13 VERDICTED FAIL, -longrun17 CORRECTED TO
  PASS (partial)** (08-22, same cycle — this pipeline trains 2-4M
  steps in single-digit minutes, so both finished before triage did;
  `-longrun17`'s first verdict was a premature-verdict race, written
  before its own eval synced — see the top-of-file correction
  banner): the budget lever is NOT uniformly closed. `longrun13`
  (from the seed that near-passed at 2M) got WORSE with 2x the budget
  (progress 0.873x->0.792x clone, slip 1.08x->1.286x, confirmed).
  `longrun17` (from the seed that missed at 2M) actually CROSSED the
  rung-A gate at 4M — DR-0 det progress 1.02x clone, slip 0.74x,
  speed 0.069 m/s, zero falls, gait_valid 6/6, clean video — the
  first full det-mode PASS in the entire phasedir1-9/longrun lineage.
  Both runs' W&B ep_rew_mean rose strongly through the extra budget
  (quarters ending +187/+193 vs either 2M run's ~-300), but only one
  seed's gate metrics tracked that rise — a seed-dependent basin
  outcome, not a clean reward<->eval divergence story for both. Zero
  falls, gait_valid 6/6, clean 6-leg video, all four runs (stdanneal/
  seed17/longrun13/longrun17). Do not respec a 3rd continuation off
  any converged phasedir9 checkpoint, and do not retry another
  anneal-frac/--steps combination blind — DIG-IN queued first: (a)
  reproduce longrun17 independently, (b) root-cause the seed13-vs-
  seed17 divergence, alongside the on-pod per-tick
  trace (bc_target cadence vs realized policy cadence vs raw
  un-phase-locked teacher cadence) before any anchor-dose/phase_hz
  reward edit.

## Next

0. **100 Hz RATE CONVERSION (operator 08-24, fb_20260824T174619_c49b7e
   — launched this cycle as `cw-arch-hist16-dep1-c1-joyfullcurr13-v7-
   hz100-r2`, VERIFIED RUNNING train-0; attempt 1 without `-r2` died
   FAIL-CLOSED at the inherited walkcurr-precert bar — init prog=0.203
   < 0.50 at exact-b0/100 Hz with 0 falls, i.e. obs mapping INTACT,
   degradation = the expected rate mismatch — so -r2 drops
   cert-at-init/precert and starts the frontier honestly at b0)**:
   all NEW PPO models train at `control.hz=100` /
   `safety.max_delta_q_deg=0.375` (37.5 deg/s physical slew preserved;
   launcher-enforced, see CURRENT_TRUTHS 08-24 ruling). First arm: the
   V7 certfreeze recipe (stress-diversified WALKCURR_BUCKETS_V7,
   cert-only freeze, k_walk_stop_current=2.0) warm-started from the
   stable seed0 parent `ppo_goal_cw_arch_hist16_dep1_c1.zip` (V7 itself
   still mid-training, no checkpoint to prefer), 40M ticks = 1/4 the
   sim-seconds of a 25 Hz 40M run — a rate-conversion experiment, not
   a 160M-tick equivalence. Triage MUST compare reward vs frontier/
   joygate trends: rising reward + flat evals at 100 Hz = per-tick
   pricing misalignment (4x denser action-delta/current charges), not
   seed lottery. Every pre-08-24 checkpoint is 25 Hz — legacy evals
   pin control.hz=25 automatically (pod_eval), and the robot runner
   refuses rate-mismatched exports.

1. **CLOSED 08-22, all 3 remaining rise-bank items, root-cause (not
   re-measurement)** (7/7 tibia-150 residue now closed except
   fastprof, a separate already-tracked item): PLANT_SPEC's
   height-window "failure" was never the window — it was
   `goal.rise_height_mm=[108,114]`/`actions.max_height_mm=115`, the
   PRE-tibia-150 (128 mm) belly->plant height target, never updated
   when the tibia grew ~22 mm; the demonstrated reference
   deterministically settles at h_rel=131.94 mm (all seeds), ~24 mm
   past the stale target, tripping only `height_ok` while every other
   PLANT_SPEC check passed. Recalibrated the target to `[128,136]`/
   `137` (brackets the measured settled height, `RISE_OVERRIDES`/
   `SCORE_OVERRIDES` in `test_task_semantics.py` only — `LOWER_
   OVERRIDES` and other rise_height_mm call sites weren't broken,
   left untouched). `getup_honest_ordering`'s partial-crouch pricing:
   the one-shot progress ratchet (`reward.getup_k_progress`) didn't
   clear the honest rise motion's own extra regularizer cost over
   freezing (partial -12.16 < freeze -11.26 at k=60); recalibrated
   60->200 (partial +10.8 > freeze -11.5, full GETUP-bank ordering
   preserved, swept 60-500). Bank now 152 pass / 1 known-red
   (fastprof) / 4 skip / 1 xfail. The circular blocker is UNBLOCKED
   on the bank side: `extract_rise_ref.py --blend-mode ik`
   (built+tested 08-22) can remint a compliant reference once a
   tibia-150 stance source checkpoint exists, and a tibia-150 stance
   retrain arm can now be spec'd+launched (bank no longer red) —
   still gated only on joystick/amp GPU-budget priority, not on any
   remaining test. `cw-stand-footlow2-plant150-1` (train-1, warm-start
   from `ppo_goal_cw_stand_footlow2_hard1`, byte-identical recipe,
   only the plant now tibia-150) VERDICTED FAIL 08-22 — but HALF
   CONFIG BUG: rise/det 1/6, rise/sto 0/6 (parent 5/6 det, 6/6 sto),
   yet the launch's copy-pasted `--cfg-set` list carried the
   PRE-tibia-150 `actions.max_height_mm=115`/
   `goal.rise_height_mm=[108,114]` unchanged, never picking up the
   SAME 08-22 recalibration (`[128,136]`/137) already used by
   `test_task_semantics.py`'s bank — a gap in that fix's own scoping
   note (see CURRENT_TRUTHS). Re-evaluating the frozen checkpoint with
   the corrected cfg (zero retraining) turns bridge/crouch/flat rise
   CLEAN (1.4-3.9mm err, det 1/6->3/6, sto 0/6->4/6) — the policy had
   been overshooting the stale target by the tibia-150 height delta.
   Genuine residual, unmoved by the cfg fix: RSI-reset (mid-ramp
   DeepMimic spawn) starts fail every episode (5/5, ~22-29mm
   undershoot, more roll wobble) — passed fine at the 128mm parent
   (4/5), so this is real and tibia-150-specific. hold/lower unaffected
   (6/6 both). **LAUNCHED 08-22**: `cw-stand-footlow2-plant150-2c-
   heightfix` (train-5, same checkpoint, ONLY the corrected cfg, 10M
   more steps) — tests whether removing the reward-target fight also
   helps the harder RSI case; still targets the session gate's
   tibia-150 sit-fall (the stance half of the MEASURED-PLANT GATE
   BREAK, still open) so the download pair's stance half can be
   re-promoted alongside the walk half. **VERDICTED FAIL 08-22,
   ROOT-CAUSED**: RSI stayed pinned at 22-29mm height error through
   the full 10M extra steps (det rsi 0/3, sto rsi 0/2, zero movement
   from the pre-training baseline); bridge/crouch/flat held steady
   (det 3/6, sto 4/6, no regression). Training reward flattened after
   Q1 (120.6/201.1/198.1/200.6) — reward-flat + eval-flat, a genuine
   stuck mechanism per the 08-21 ruling, exactly the gate's own
   pre-registered FAIL branch. ROOT CAUSE found (not just
   re-confirmed): `sim_env.py`'s RSI height-schedule rewrite anchored
   the episode's ABSOLUTE height target to `rise_ref_belly2plant.npz`'s
   OWN recorded final height (`ref["h"][-1]` = 110.96mm) — that npz
   predates the tibia-150 change; the SAME `q_rad` trajectory settles
   at 131.94mm on the current sim (the exact number the
   `rise_valid_plant` fix already measured for a different bug). The
   ~21mm gap matches the observed RSI error almost exactly: every RSI
   episode was being TRAINED toward a target ~21mm below the real
   `goal.rise_height_mm=[128,136]` window the eval actually grades
   against. FIXED: the RSI schedule now anchors to the episode's own
   live-cfg target (already correct post-tibia-150) and uses the
   reference array only for FRACTIONAL progress at the spawn point —
   robust to the stale absolute scale. 2 new regression tests
   (`test_rise_rsi_height_target.py`, both pass) lock the fix; full
   `test_task_semantics.py` bank re-run clean (159 pass, only the
   pre-existing known-red `fastprof` fails, unrelated). **LAUNCHED
   08-22**: `cw-stand-footlow2-plant150-3-rsifix` (train-5, same
   checkpoint, ONLY the RSI mechanism fix, 10M more steps) — tests
   whether RSI now actually improves once its own reward target
   agrees with the eval target.
2. **Evaluator half DONE 08-22** (`rl_move/sim/eval_joystick_gate.py`
   + `test_eval_joystick_gate.py`, 8/8 pure-aggregation tests, no sim
   touched): a reusable, versioned 60 s randomized joystick-session
   gate — pure orchestration around `eval_checkpoint.py` with ONE
   pinned held-out command bundle (`goal.walk_cmd_mode=stress_mix`
   [full family: random_hold, flip_180=reverse, sweep_circle/
   square=turns, stop_go, jitter] + `walk_cmd_resample_s=4.0` +/-
   jitter 0.5, `--episode-seconds 60`, seed base 90000 — a range no
   other harness/training run draws from), run at DR-0 + the
   checkpoint's own DR, det+sto, aggregated into ONE PASS/FAIL against
   the DONE gate's own numbers (zero falls, slip/m <= 2.9, dir_err
   median within 5deg of the teacher's 35deg floor). FIRST-EVER
   READING (n=12x2 det/sto, DR-0, `logs/ckpt_eval/
   phase1_clone_joystick_gate_v1/`): the phase clone
   (`ppo_goal_cw_bcgait_init_fullprof_phase1`) — previously verified
   only against FIXED heading-per-episode panels — FAILS the real
   randomized multi-segment session hard: zero falls / gait_valid
   24/24 (the gait itself never breaks), but slip/m med 15.9 (cap
   2.9) and dir_err med 65.4deg (allow 40deg) once commands actually
   change direction/stop/reverse mid-episode instead of holding one
   heading. This is the first measured, reusable confirmation that
   "passes the direction-first curriculum" (fixed headings) and
   "joystick controllable" (this DONE gate) are different bars — the
   gap RL fine-tuning (Next item 4) must close. **DONE 08-22**: the
   two follow-ups flagged here (per-leg duty/swing/sacrificed-frac
   aggregation across the whole panel, and an opt-in `--video` flag
   replacing the hardcoded `--no-video`) are landed and tested
   (`test_eval_joystick_gate.py` 11/11, +3 new per-leg tests incl. a
   regression test for the exact frozen-tripod pattern seen this
   cycle on the AMP track). Semantics-bank half NOT started
   dedicated to this exact 60s session (the walk task's own reward is
   already alignment-tested per-mechanism via `test_phasedir_
   semantics.py`, 34/34); a session-specific bank is lower priority
   until a candidate gets close enough to the gate numbers to need
   cheat-proofing at this exact horizon.
3. **CLOSED 08-22**: the loadslip-band/drag-stance-dose/allowance
   lever family AND the "anneal noise, continue training" lever are
   both measured-refuted for this lineage (see Now: phasedir6/7/7b,
   and the `-9`/`-9b`/`-9-cont1` init-basin-flatness finding).
   **FORK CORRECTED 08-22 (was wrongly marked closed for ~15 min by
   a race)**: `phasedir9-longrun13`'s ledger verdict (seed13, WORSE:
   0.792x/1.286x) is correct and confirmed. But `-longrun17`'s FIRST
   committed verdict (~0.72x/1.32x, FAIL) was written at 14:03:08,
   BEFORE its own eval finished syncing (gate report synced 14:03:45,
   owncfg 14:04:04) — a genuine premature-verdict race, not a
   judgment call; a second concurrent-cycle sync at 14:07:02
   inherited the same stale numbers without re-checking the by-then-
   available real data. CORRECTED (triple-confirmed: on-disk
   report.json, `ops.sh report`, and this run's own W&B
   `eval/dr0/walk_det/*` summary all agree): `-longrun17` (seed17) is
   a **DET-mode PASS** of this lineage's rung-A clone-relative
   gate — the first in 34+ arms — at BOTH DR-0 (progress 1.02x clone,
   slip 0.74x, speed 0.069 m/s, all caps cleared) and its own DR-0.35
   (0.94x/1.01x/0.067, thinner margin). Zero falls, gait_valid 6/6,
   clean roll, no sacrificed legs, video shows a clean 6-leg
   alternating gait. STO still fails (as it always has lineage-wide —
   the clone's own sto baseline is itself degenerate, so sto was
   never part of the ratio criteria). Ledger status set to `PASS
   (partial)`, not a closing PASS: own-DR margin is thin, sto is
   still bad, and — the real puzzle — seed13 and seed17 ran the
   IDENTICAL recipe/steps/anneal and landed on opposite outcomes
   (worse vs. first-ever pass). That seed-dependent divergence, not a
   uniform "budget helps" or "budget doesn't help" story, is now the
   open question. DIG-IN state: (a) reproduce `-longrun17`'s reading
   — `-longrun29` (seed29) RUNNING, `-longrun23` (seed23) queued this
   cycle: with seed13 FAIL + seed17 PASS the question is the recipe's
   PASS RATE, n=4 seeds decides promotion strategy (select-best-of-N
   is legitimate if the surface is a seed lottery). (b) the family-
   boundary trace is **RESOLVED 08-22** (see banner): no cadence gap,
   supervision exonerated, `swing_s_mean` premise was clone contact-
   chatter; the divergence is the init/seed-basin lottery, and the
   failing seeds' deficit is stance slip at matched gait timing. Do
   NOT spend arms on anchor dose / walk_phase_hz / phase-lock edits.
   **UPDATE 08-22: DIG-IN item (a) is DONE — n=4 seed sample
   complete.** `longrun23` (0.818x progress/1.175x slip clone) and
   `longrun29` (0.740x/1.296x, worst of 4) both verdicted FAIL,
   landing in the same regression basin as seed13 — pass rate 1/4
   (`longrun17` only). "More seeds" is closed as a lever; per-seed
   promotion is legitimate (operator already live-accepted
   `longrun17` — see the file banner) but does not fix the recipe.
   Follow-up on the pricing side (item (b)'s "stance slip" lead):
   built+bank-tested `reward.drag_stance_allow_ramp_steps`/`_mm`
   (`test_drag_allow_ramp.py` 6/6) to directly re-attack the
   regime-gap (a fixed 24mm det-calibrated drag allowance overtaxes
   honest noisy exploration before the log-std anneal converges) by
   loosening the allowance early (48mm) and annealing it to the same
   24mm target in lockstep with the noise anneal. Single-change A/B
   on all 3 FAILing seeds (13/23/29, run in parallel by concurrent
   cycles): only **seed13** (`phasedir10-allowramp-a`) improved,
   moving BOTH clone-relative axes toward the gate at once for the
   first time in the whole lineage (progress 0.792x->0.830x, slip
   1.284x->1.162x) — still a FAIL (short of 0.9x/1.15x) but the first
   lever ever to help both axes together. **seed23** and **seed29**
   both got WORSE on both axes (seed23 0.818x/1.175x->0.792x/1.217x;
   seed29 0.740x/1.296x->0.725x/1.466x) — 1/3, FAIL, refuting it as a
   general fix. Zero falls/gait 6/6/clean video, no pathology, all
   three arms — a real basin effect, not a regression. Reads as: the
   regime-gap diagnosis is real, but a fixed ramp schedule is itself
   subject to the same per-seed basin lottery as every other lever
   here. Mechanism/code stays banked; do not arm it as a default off
   seed13 alone. **RAMP LEVER CLASS CLOSED 08-22 (~17:1x, judgment
   closure)**: the 4-arm grid (`allowramp2slow-seed23/-seed29`
   ramp_steps 1.2M->2.4M; `allowramp2wide-seed23/-seed29-b` ramp_mm
   48->64) came back 3 worse-on-both-axes (wide23 0.65x/1.65x,
   wide29 0.62x/1.74x, slow23 0.65x/1.70x — each worse than BOTH its
   no-ramp and 48mm baselines) and 1 nominal improver (slow29
   0.78x/1.25x vs 0.740x/1.296x and 0.725x/1.466x — prediction-if-
   true technically fired, so the strict 0-for-4 auto-closure did
   NOT literally trigger; recorded honestly). But the improver's
   prog delta (+0.044x) is at the edge of 6-ep noise, its slip delta
   inside noise, and across 6 ramp arms x 3 seeds every dose helps
   exactly one seed and hurts the rest (48mm->seed13 only,
   slow->seed29 only, wide->nobody) — a per-seed basin lottery, not
   a dose-response, with the best reading anywhere (seed13 0.830x)
   still far from the 0.9x gate. Zero falls / gait 6/6 / clean video
   all 4 arms; reward rose or peaked-then-tightened in all 4 while
   gate metrics mostly fell (misaligned-surface signature, not
   undertraining). CLASS CLOSED for failing seeds by judgment on
   this evidence; redirect stands: sto gap first (stotight dose
   grid — evals manually launched 17:0x after the 16:18 manual
   FINISHED-flip bypassed the watcher's finish-detection prestage;
   gate+joygate artifacts land in `logs/ckpt_eval/
   cw_dep_bcgait4_phasedir9_longrun17_stotight{,40,45}_*` — TRIAGE
   THEM NEXT CYCLE, no auto-spawn will fire for already-FINISHED
   runs), then item (b)'s matched-timing stance-slip mechanism
   (seed17 2.85/m vs clone 1.89/m).
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
