# RL_LOG — condensed campaign log

Full history: `archive/RL_LOG_FULL_2026-08-09.md` (through c34) and
`archive/RL_LOG_FULL_2026-08-09c.md` (c35–c59 detail). Per-run facts:
`rl_docs/runs/<run>.md` (generated) + W&B notes. Skills inventory:
`rl_docs/SKILLS.md`. Cycle transcripts: controller `/workspace/cycle_logs/`.

**APPEND RULE (operator, 08-09, tightened after the log tripled in half
a day): ONE line per cycle, written ONLY via `ops.sh logline "..."`.**
Never `cat >>` this file. The line is: cycle, run(s) → verdict word,
one-clause takeaway, what launched. ALL evidence, numbers, and
narrative go in the ledger verdict (auto-renders `rl_docs/runs/`) and
the W&B OUTCOME note — if it matters, it lives there, not here.

## State (2026-08-09 ~19:00Z)

- **WALK CHAMPION: ppo_goal_cw_walk_longdist_r2.zip md5 bcddc65c**
  (c44, seed-confirmed 3/3, operator-accepted; DR1.0 det slip 1.06;
  NOT hardware-ready — paddle-slide persists, sto draw-stalls).
  Slip root = sim contact/current pricing → operator calibration
  (P0); reward-side anti-slip levers ALL closed.
- **Driving:** joystick gate (`eval_drive`) is the binding eval;
  champion + joystick45 + joyjit-dr05-c1 all PASS it. Heading ladder
  FROZEN at ±90° (head135 FAIL); rear coverage waits on
  mirror-symmetry [CODE].
- **Validated axes (see SKILLS.md for the full table):** crouch to
  −60mm, terrain amp1.0 (saturated), payload +40%, latency 2.5x,
  deadband 3x, CoM shift 30mm, stop density 0.35, 60s endurance,
  DR0.5 on steering/±90. Full-DR (1.0) retrain lever CLOSED (2x).
- **Next implementation cycles [CODE]:** mirror-symmetry (3
  motivations), quad-hold goal mode (feasibility sweep = GO).
- Compute: 12 GPU pods (train-0..11; g12ba48 cordoned, 12–15
  deleted). `capacity.py` = live truth; backlog auto-drains.

## Walk line — what was tried and learned

- **dr04b lineage** (pre-08-08): scalar champion, 0/9 gait-valid on
  video — shuffle/flag-leg. RETIRED. Lesson: scalars hide exploits;
  video eval is the promotion standard.
- **w07/flag/flagw/lp/speedhi**: penalty-coefficient iteration on the
  shuffle is a dead approach (all REFUTED).
- **nv/nv2**: deployable obs are not the blocker. **aac**: asymmetric
  critic is a retention tool, not a gait fix.
- **phase / phase-stance(2)**: phase reward as basin escape REFUTED.
- **step0** (08-08, operator recipe: step-event + drag + park-duty
  pricing, fresh init, walk-only): **FIRST genuine six-leg gait.**
  Lineage standard since.
- **step0-c2 vs lowent**: entropy runaway = the plateau driver; warm
  starts use ent 0.001. Identical-config continuations bought nothing
  5x — CLOSED as a move.
- **h15b**: 15 s is the lineage eval standard. DR is NOT the
  bottleneck (untrained parent passes DR 0.3/0.6).
- **kgate**: park PRICING refuted. **Cycle 27:** there is no park
  attractor — the "park" was one fixed backward draw; real defects are
  paddling (91% of slip) + overspeed.
- **anchorgate** (c31): income GATING works where charging failed —
  det slip 1.543→1.240, champion. **anchorgate-c1** (c32):
  cadence-inflation exploit. **anchortol5** (c34): the policy PAYS a
  binding stake and keeps creeping — **no income lever can outbid
  in-sim-free sliding; slip root is contact/current pricing
  (operator).** **step0-anchor** 40M fresh (c33): re-derives the
  paddle from scratch — pricing problem, not basin/history.
- **loadslip** (c40): episode-level slip stake also paid-not-fixed —
  reward side of skating CLOSED (seed-confirmed).
- **fast** (c42): speed ceiling ~0.065 m/s is GAIT-limited; speed
  arms join skating behind the calibration.
- **stalls** (c47→c52): fixed-draw stalls are command-conditioning;
  resample training fixes resample-on eval only, incidence <2% —
  stall lever class CLOSED, panel draw kept as canary.
- Exploit-watch columns (permanent): cadence/stance count, per-leg
  swing asymmetry, allowance-riding, unload-sweep.

## Stance line

- Heights SOLVED at DR 1.0 (crown jewels, canary-protected). Lower
  posture (flag leg) never solved; refuted in order: mix, posture
  pricing, exploration, terminal pricing, reset diversity, dense
  charging. **Root cause (c28): the hover is INCOME-POSITIVE**
  (current model prices planted descent 4x hover) — **BLOCKED on the
  same operator pricing ruling.** No stance shaping until then.

## Infra lessons (all landed in tooling — see rl_docs/COMMANDS.md)

- Launcher is mandatory (capacity, code-SHA gate, dup refusal);
  ledger writes via `launch_run.py update` only.
- Canaries protect only skills the lineage HAS (`--no-canary` on
  step0 walk-only lineage).
- Pre-08-08 seed twins were bit-identical clones; conclusions void.
- Sync the pod at snapshot time; snapshot BEFORE drain, never
  mid-drain (HEAD moves under the launcher).
- Specs must ALWAYS pass `--out-name` (prestage pullckpt breaks on
  MJX default names otherwise).
- Dirty-marker refusals: state files, .md, logs, zips, tmp files are
  all EXCLUDED from the dirty check now; a genuinely dirty tree means
  COMMIT FIRST. wandb/ run logs are gitignored (were tracked, poisoned
  every sync 08-09).
- /dev/shm leaks after killrun: clean hexmjx-* segments before
  relaunching on that pod (c54: two 0-step deaths).
- Node loss is normal (g12ba48 cordoned 08-09): fleet truth is
  `capacity.py`, never a doc.
- Fixed-draw sto panels can pin a "failure" to one command draw —
  check WHICH episode fails before theorizing.

## Cycle digest (c38–c59, 08-09; detail in archive + rl_docs/runs/)

- c38 longdist-r2 det slip 0.96 campaign-best; drain dirty-fix. c39
  dr05-r1 FAIL; stability pricing arms launched. c40 loadslip FAIL →
  reward side of skating CLOSED. c41 fps SUSPECTs = false alarms.
- c42 fast FAIL (gait-limited ceiling). c43 steer-explore FAIL (no
  omni transport); diag45 operator-killed. c44 **champion promoted:
  longdist_r2** (accepted by operator ruling 8); speedband killed as
  stale. c45 lowgait/wander PASS, fwdband-r1 FAIL (command mix closed),
  fall300 NO-EFFECT.
- c46 lowgait30 PASS. c47 terrain05/wander30/endur60 PASS,
  longdist-dr05 FAIL (DR not the stall lever). c48 strafe-dr05 +
  wander-dr05 PASS. c49 joystick45 PASS (joystick gate born);
  joyjit-dr05 starved→rebalanced; dr.<field> cfg hooks landed.
- c50 lowgait40 PASS, tilt05 NO-EFFECT (tilt lever closed), endur60-s1
  endurance seed-robust. c51 head90 PASS (±90 envelope; L/R asym →
  mirror-sym motivation); drain-dead root cause = one uncommitted
  artifact. c52 stallfix FAIL → stall class CLOSED.
- c53 joyjit-dr05-c1 PASS (best driving candidate), longdist-s2 PASS
  (3/3 seeds), longdist-dr10 FAIL (full-DR lever closed);
  stopgo35 starved→c1. c54 lowgait50/terrain10 PASS (terrain
  SATURATED), strafe-dr10 FAIL (DR0.5 = strafe ceiling); endur60-r2
  FAIL on slip clause; shm-leak bug found+fixed.
- c55 wander-dr10 FAIL (DR0.5 = steering ceiling); READY well dry →
  quadruped sweep flagged. c56 head135 FAIL (**ladder frozen ±90**),
  head90-dr05 PASS, latjit25 PASS. c57 payload50 PASS; **quadruped
  feasibility sweep = GO** (39mm margin w/ shift+splay). c58
  lowgait60/wander-dr05-s1/wander60 PASS; comshift30/deadband30 PASS,
  friclow letter-pass only (slick draws skate). c59 stopgo35-c1 PASS,
  torquedroop NO-EFFECT (champion already covers 0.75–1.05x free —
  run the parent baseline FIRST on exposure axes); fricvar PASS
  (0.4–1.6x, slick-tail churn), speedband2-r1 FAIL (stale reissue of
  the CLOSED c42 speed class — check run docs for the CLASS before
  requeueing).

## Cycle log (one line per cycle via `ops.sh logline` — no exceptions)
- 08-09 18:46 OPERATOR process pass: triage = ops.sh review; RL_LOG writes = ops.sh logline ONLY (598->143 trim, full text archived); snapshot dirty-check hardened; backlog add warns on same-axis dupes. 

- Cycle 60 (08-09 ~18:3x-19:3x): 1 triage. `cw-walk-groundtilt5` PASS — floor-
  slope axis (13b) lands by exposure: own-cfg tilt u(0,5deg) gv 12/12, 0 term,
  det med fwd 1.40m; DR0 retention slip 1.03/prog 0.96 = champion band; honest
  tail 2/6 steepest det draws shuffle at ~1/3 speed (slip 3.4-4.5, no falls) —
  solid to ~3-4deg, 5deg marginal; SKILLS row added; paddle lineage, not
  hardware-ready. Infra: watcher's dr0ret eval deadlocked on a corrupt ffmpeg
  pipe mid-sto-video (utime frozen, no children) — killed, det-only --no-video
  rerun recovered the retention numbers (gotcha added to COMMANDS.md). Refills
  (4 = cycle cap, 80M GPU): imumount10 (13c-class sensor axis, IMU mount
  miscalib 10deg) + badstart (13b boot-pose axis, prob 0.25 @8-35deg), both
  isolated off no-DR champion, drained+ALREADY FINISHED same hour (watcher
  will cycle their triage); groundtilt-dr05 (compose rung off today's PASS,
  RUNNING t8 after 2 drain-race REFUSEDs + pod code sync) + payload50-s1
  (ruling-7-style seed twin of the c57 payload PASS, queued).
- 08-09 19:31 c61: payload-dr05 FAIL (own-DR0.5+payload panel clean 12/12, det med 1.36m, but DR0 no-payload retention eroded: slip 1.38>1.24, prog 0.54 vs parent 0.95 — first dr05 compose to charge nominal; watch in-flight comshift/deadband/fricvar/latjit-dr05 retentions); refilled train-11 with latjit-dr05 compose (drain VERIFIED, checkup HEALTHY); note: 2 controller evals OOM/load-killed silently at load~212, relaunched via setsid. 
- 08-09 19:42 c60: 3 triages. joylat25 PASS - latency 0.5-2.5x composes onto the abrupt-flip DR0.5 driving package, NEW BEST DRIVING CANDIDATE (joystick gate 0 falls, own-DR gv 12/12; SKILLS updated; s1 seed run training). cmddrop10 NO-EFFECT + velsag30 FAIL(letter)/NO-EFFECT - parent baseline matches both per-episode: champion already covers 10% cmd dropout + servo-speed sag to ~0.8x FREE, deep sag ~0.7x = untrainable transport boundary (battery-calibration class); servo-imperfection single-axis exposure now 0-for-3, TEST CHAMPION FIRST before queueing this class (cmddrop20 verdict pending elsewhere, treat as one ladder). Infra: batch-eval shell footgun found+documented (CFG assignment swallowed by first bg job -> 4 evals silently ran default cfg, all rerun valid; COMMANDS.md gotcha 14). Refills: joylat25-s1 (running t0) + joylat60 (60s driving endurance) + joycom30 (off-center-payload driving) queued; ckpts pre-pushed to t5/t11. 

- 08-09 20:3x c60b: 3 triages. `cw-walk-lowgait70` PASS — crouch envelope
  extends to -70mm (gv 12/12, 0 term, mean end-height err 2.0/1.9mm, det agg
  slip 1.07; SKILLS row -20..-70mm); lowgait80 rung launched. `cw-walk-wander-dr05-s2`
  PASS — own-DR0.5 prog 0.97/0.93, slip 1.46/1.88 = seeds 0/1 noise band; ruling-7
  3-seed panel COMPLETE for the steering-DR recipe. `cw-walk-cmddrop20` NO-EFFECT
  (letter passed; parent longdist-r2 under identical 0.20 drop spread matches
  episode-by-episode incl. the same 2 churn draws) — cmd-drop exposure lever CLOSED
  0.10+0.20 as ONE ladder study with c60's cmddrop10; servo-imperfection exposure
  0-for-4, champion envelope row updated. Refills (4 = cycle cap, 80M GPU, all
  VERIFIED RUNNING): zerobias3 + gainvar + imubias3 (13b/13c servo/IMU calibration
  axes, parent-baseline-at-triage pre-registered in each gate) + lowgait80; remaining
  free slot left to concurrent cycles (HARD reason: max_new_launches_per_cycle=4).
  Infra: hit the c60 batch-eval $CFG footgun + silent load-kills myself — 3 evals
  relaunched with /proc-verified cfg (COMMANDS.md gotcha 14 extended); watcher PAUSE
  present since ~19:03 (operator/restart window — not mine to clear).
- 08-09 20:46 c62: badstart FAIL-letter (boot-pose recovery works — 12/12 gv, 0 falls, no lurch — but bad-start draws transport 0.83-0.85m and nominal fwd shaded 1.32 vs champ 1.57, det med 1.17<1.2 gate; axis trainable-not-free, no requeue, dr05 compose cancelled); comshift-dr05 PASS (retention CLEAN 1.49/0.98 — dr05-compose class now 2-1 vs payload-dr05) + deadband-dr05 PASS-caveat (retention slip 1.22 at 1.24 cap, fwd 1.42); SKILLS rows updated. Refills (cap 4/80M reached): contactstiff+linklen (13b axes, champ baselines measured FIRST per c59 — tails 0.61-0.77m justify; both ALREADY FINISHED awaiting triage) + comshift-dr05-s1 + payload-dr05-s1 ruling-7 twins to settle the compose-class retention split. NOTE: watcher PAUSE file present — no auto-triage/drain; next cycle must pick up contactstiff/linklen + idle t0/t9/t10/t11. 
- 08-09 20:50 c62: 3 triages. fric50 FAIL (if-false: 0.5-1.0x grip still skates the slick draws, det slip med 1.35 vs gate 1.3, worst draws 3.1-3.5; retention CLEAN 1.57m/slip 0.86 = friclow's 9% tax came from its 0.3x extreme, not friction exposure per se — friction-exposure axis CLOSED pending operator contact-pricing calibration, evidence fric50+friclow; other friction runs get no verdict from this). fricvar-dr05 PASS + latjit-dr05 PASS (cleanest dr05 compose: det fwd 1.50m, slip 1.04 champion band; BOTH DR0 retentions clean — no payload-dr05-style erosion; SKILLS rows added). Watcher was paused: pulled ckpts + ran gates by hand; fric50 ckpt was default-named on t9 (spec lacked --out-name) — canonical copy made, md5 bacaa00f both sides. Champion baselines (c59 rule) show contact-stiff + leg-mass axes NOT free (2/6 det draws crater each). Refills 4/4 VERIFIED: legmass25 t0 (last unexposed 13b axis), joyfric t6 + joytilt3 t7 (driving-package composes off joylat25), latjit-dr05-s1 t8 (ruling-7 seed twin); joylat25+latjit25 parents pre-pushed to all idle pods. 
- 08-09 21:16 OPERATOR (08-09 ~17:1x): UNIFIED JOYSTICK POLICY = top deliverable - ONE checkpoint for stand/walk/steer/sit (no per-skill zoo). Line opened: cw-uni-blend1 queued (goal-mix blend off joyjit-dr05-c1). RL_PLAN Queue item 0 + WISHLIST item -1 are binding. 
- 08-09 21:31 c63: lowgait80 FAIL (crouch envelope BOTTOM at -70mm: height err 12mm>8, det slip 1.50>1.15, one sto flag leg, reward declined — ladder closed, no -90mm); multiaxis1 PASS (4-axis compose gv 12/12, det med fwd 1.29m, DR0 retention clean — axes stack without interference, robustness-champion base candidate); payload50-s1 PASS (seed twin mirrors seed0: det med 1.32 vs 1.31, identical 2/6 heavy tail — payload recipe confirmed, not luck). Watcher had skipped all 3 gate evals (7-eval cap) — ran by hand, incl. det-only retentions. Refills queued (3, 56M GPU): multiaxis1-s1 seed twin + multiaxis-dr05 compose + multiaxis2 (+tilt 5th axis), multiaxis1 ckpt pushed to t0-t3.
- 08-09 22:3x OPERATOR SESSION — SECOND floor-penetration cause fixed (0823ac0): tibia_link.stl is drawn 29.5mm LONGER than the kinematic leg (CAD builder measures the tube from the yoke socket, not the knee axis; tip lands at 157.5mm vs TIBIA_LENGTH=128). Contact/physics were always at 128 — but every render/eval VIDEO showed boots poking through the floor. Visual now squashed to match physics (verified <1mm at stance hold, all six legs). VIDEO REVIEWERS: pre-0823ac0 videos show phantom leg-through-floor; do not verdict on it. OPERATOR RULING (08-09 22:4x, no bench measurement): kinematics (TIBIA=128) = truth, drawing must match kinematics (done), leg-length DR absorbs as-built variance (link_len_scale_pct 0.02 global x link_len_leg_pct 0.012 per-segment ~ +/-4mm tibia at DR1.0 — covers the known 4mm short tubes on legs 0/4; would NOT cover a 30mm systematic error, accepted). CAD builder discrepancy stays open as a CAD-side issue only, not a training blocker.
- 08-09 22:2x OPERATOR SESSION — SIM DEFECT FIXED (273ebde): leg segments (femur, tibia, knee servo) had NO floor collision — only foot spheres/chassis/yaw-servo boxes collided — so rise/lower policies could sweep shins THROUGH the ground (operator caught it watching stand/sit in MuJoCo; chain-standwalksit's flailing was worst-case: its rise never trained, verdict stands). Fix: floor-only collision capsules + knee-servo box, bitmasked (contype 4 vs floor conaffinity 5) so no leg-leg/leg-chassis false positives. Walk champions VERIFIED unaffected (zero shin-floor contacts in gait, wander30 0.60m/12s retained). Consequence: any rise/lower behavior trained BEFORE 273ebde is suspect near the ground; walk-only lineages fine. cw-uni-blend1 killed 25min in and requeued as cw-uni-blend1-r2 (VERIFIED RUNNING t4) on the fixed sim; its gate now includes "VIDEO: no leg-through-floor in rise/lower". 
- 08-09 22:35 c66: 1 triage. cw-walk-zerobias3 NO-EFFECT (letter-PASS: own-cfg per-joint zero-bias u(-3,3)deg gv 6/6 det, det med fwd 1.28m>=1.2 gate, DR0 retention clean slip 0.99; but parent longdist-r2 under IDENTICAL bias spread matches episode-for-episode incl. the 2 steep-bias craters, frames pixel-identical churn-in-place) - 13b/13c calibration-exposure ladder now 0-for-6 (joins gainvar+imubias3, both also NO-EFFECT this window by concurrent cycles); SKILLS tally updated, no requeue. Refills (4, all VERIFIED healthy after one /dev/shm-leak retry on groundtilt5-s1->s1r1 per gotcha 13): groundtilt5-s1 + fricvar-s1 + deadband30-s1 (promotion-panel seed twins for 3 already-PASSED single-seed 13b axes, ruling-7 completeness) + quietcurrent (NEW axis, wishlist item 13: enables the existing default-OFF k_current_hot=0.2/current_hot_a=1.5 hot-current-concentration penalty on the WALK task for the first time, champion-baseline-at-triage pre-registered). 
- 08-09 22:35 c65: 3 triages. cw-walk-lowgait-dr035 PASS -- -50mm crouch survives DR up to 0.35 (own-cfg 12/12 gv, 0 term, height err 6.8/4.6mm det/sto, slip 1.08/1.26; DR0 retention clean 0.98) -- ceiling banked between 0.35 (holds) and 0.5 (FAILED prior); SKILLS row added. legmass25 NO-EFFECT + stiffvar NO-EFFECT: both exposure runs are episode-identical to their unexposed champion baselines on the hard draws (same 2/6 det craters, matching severity) with clean DR0 nominal retention -- leg-mass-asymmetry and contact-compliance join torque-droop/servo-gain/cmd-dropout as axes NOT fixable by naive single-axis DR-0 exposure (last 2 unexposed 13b axes closed this way). Infra: hit the default-checkpoint-name gotcha again (lowgait-dr035 ckpt saved without --out-name) -- manual copy+md5, and the pod-code dirty-marker deadlock (WRAPUP file deletion left uncommitted -> drain REFUSED on all 3 free pods) -- snapshot+sync unblocked it, cw-uni-blend1 unparked from backlog_failed and relaunched. OPERATOR SESSION landed 2 sim fixes mid-cycle (273ebde leg-floor collision, 0823ac0 tibia visual-length): walk-only lineages confirmed unaffected, cw-uni-blend1 killed+requeued as -r2 on the fixed sim. Refills: groundtilt-dr05-s1 + fricvar-dr05-s1 (unparked backlog) + cw-uni-blend1 + lowgait-dr035-s1 seed twin (4 = cycle cap); heavy concurrent drain churn from other cycles filled remaining slots. 
- 08-09 22:39 c66: 3 triages. comshift-dr05-s1 PASS (seed twin confirms compose, own-cfg gv 12/12, DR0 retention clean slip 1.00; honest note: this seed's det tail is worse, 2/6 severe craters vs seed0's 1 mild dip). joyhead90-lat25 PASS - latency 0.5-2.5x composes onto +-90deg abrupt-flip DR0.5, NEW WIDEST driving candidate (JOYSTICK GATE @90 0 falls, own-cfg gv 12/12, prog 0.93/0.96 >= parent); joyjit-dr05-s1 PASS (seed-confirms the old c53 promotion panel, now superseded by joylat25/joyhead90-lat25). SKILLS updated x3. Refills: gyronoise15 (new 13b axis, champion-baseline-confirmed NOT free at 3x default) landed RUNNING; joyhead90-lat25-s1 seed twin + placementnoise6 (baseline-confirmed axis) each hit a launch-COLLISION storm twice (many concurrent cycles' drains racing the same free pods simultaneously -> worker EOFError at init, 0 steps, no science result) - requeued to backlog (-s1r2/-6r) for the self-repairing drain once contention clears, not fought further. Checkup SUSPECTs (joytilt3, joyhead90-r1-s1) both resolved on their own by finishing training - no action needed. 
- 08-09 22:45 c66: 3 triages. joyjit-dr05-s2 PASS (2nd seed confirmation of joyjit-dr05-c1: joystick gate 0 falls, own-DR0.5 gv 12/12, prog 0.94/0.97, slip 1.50/1.50, DR0 retention clean — driving recipe seed-robust across 3 seeds now). joylat60 PASS (60s abrupt-flip+DR0.5+latency endurance rung off joylat25: gv 12/12 own-cfg+DR0, joystick gate 0 falls, no first/second-half decay — minute-scale hardened driving banked; SKILLS updated). latjit-dr05-s1 FAIL-letter (own-DR0.5 replicates parent cleanly, but DR0 nominal retention misses the slip<=1.24 cap: 2/6 det draws that were clean for the parent shuffle at half/third speed here — real seed-specific retention erosion, not noise; use seed-0 latjit-dr05 as the compose input, not this seed). New gotcha 13b documented: launch-collision EOFError under concurrent-cycle drain storms (clean shm, not gotcha-13's leak) cost 3 of my 4 new-axis launches (imupos15, gyrobias3, tiltnoise) one retry each; gyrobias3-r1 landed, imupos15-r1/tiltnoise-r1 lost the race again and are left for the next drain (cap of 4 new specs reached: encodernoise landed clean). Free slots after: concurrent cycles + watcher drain own the rest. 
