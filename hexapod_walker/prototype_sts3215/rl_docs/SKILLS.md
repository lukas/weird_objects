# SKILLS — what the robot can do today, and which checkpoint does it

The answer to "are the successes getting lost?" (operator, 08-09).
Every PASSED capability lives here with its checkpoint. Verdicts stay
in the ledger / `rl_docs/runs/`; this is the accumulating INVENTORY.

**UPDATE RULE (binding, 08-09): a cycle that verdicts a PASS adds or
updates one row here in the same cycle.** Checkpoints are durable in
W&B artifacts (`ckpt-<name>`, type `policy-checkpoint`) — trainer
publishes automatically since 08-09; earlier ones backfilled. The
controller's `rl_move/sim/policies/` is a cache, NOT the archive
(it is gitignored and the controller is an ephemeral pod).

Demo any row locally:
`.venv/bin/python -m rl_move.sim.drive_policy rl_move/sim/policies/<ckpt>.zip`

## Walk (main line)

| Skill | Checkpoint (artifact `ckpt-<name>`) | Evidence | Envelope / limits |
|---|---|---|---|
| **CHAMPION: forward walk, 30 s, correct speed** | `ppo_goal_cw_walk_longdist_r2` (md5 bcddc65c) | c44 promotion, seed-confirmed (s1): DR0 det 6/6, slip/m 0.94–0.96, 1.63 m @ 30 s, prog 0.98; JOYSTICK GATE PASS @DR0.2 AND @DR0.5 (0 falls incl. flip stress; baselines: `logs/ckpt_eval/champion_longdist_r2_drive*.json`) | fwd ±45° cmds only; slip ~1/m (not hardware-ready); sto stalls on some fixed draws — NOT fixed by DR (longdist-dr05, longdist-dr10 FAIL) or resample training (stallfix FAIL); stall class CLOSED c52 (<2% tail, canary only); recipe 3/3 seed-robust (s2 PASS c53); backward cmd barely moves (0.06 m/7 s); L strafe 0.21 m vs R 0.30 m; tolerates motor-torque sag to ~0.75× FOR FREE (c58 torquedroop baseline: champion = exposure-trained policy on identical draws); ≤~0.7× torque = transport stall, no falls — exposure lever CLOSED, waits for estimator rung |
| Steer through direction changes + stops (up to 60 s drives) | `ppo_goal_cw_walk_wander` / `_wander30` / `_wander60` (md5 bcabaea0) | c45/c53 PASS + wander60 c56 PASS: 60 s eps (~12 changes + stops), gv 12/12, 0 term, prog 0.94–0.99, worst slip/m 1.67 (< 30 s parent's 1.93) — no endurance decay | fwd hemisphere ONLY — backward command = fall (operator repro'd, 08-09); DR0 only at 60 s (DR0.5×60 s rung queued); paddle gait, not hardware-ready |
| Steering robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_wander_dr05` (md5 18af118f) | c50 PASS: own-cfg DR0.5 gv 12/12, 0 term, prog med 0.95–0.96, slip/m med 1.39–1.70; DR0 retention gv 6/6 | ±45° cmds, gentle 5 s resamples only (abrupt-flip hardening in flight); paddle gait, not hardware-ready; **DR 0.5 is the line's ceiling** — DR1.0 rung FAILED (wander-dr10 c53: gv 11/12, flag-leg draw at full DR); seed-robust (wander-dr05-s1 c56 PASS: prog 0.94–0.99, slip 1.59/1.75, gv 12/12) |
| Strafe ±90°, robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_strafe_dr05` (md5 cb178b91) | this cycle PASS: own-cfg DR0.5 gv 12/12, 0 term, prog med 0.92–0.97, slip/m med 1.89–2.00; DR0 retention prog 1.09, slip 1.80 (< parent 2.20) | lateral paddle; fixed commands (no resampling trained); not hardware-ready |
| Drive anywhere in the front half-circle (±90°, resampled cmds + stops) | `ppo_goal_cw_walk_head90` (md5 bcf474ff) | c49 PASS: own-cfg DR0 gv 12/12, 0 term, lateral err ≤1.6× fwd; JOYSTICK GATE PASS @DR0.2 (0 falls incl. instant-flip stress) | prog ~0.84 on mixed headings (lateral costs progress); left strafe ~½ the displacement of right (L/R asymmetry); paddle gait, not hardware-ready; **heading envelope FROZEN at ±90** — ±135 rung FAILED (head135: det tilt_pitch term, prog med 0.53, slip ~2×; rear coverage → mirror-symmetry line) |
| Front half-circle driving robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_head90_dr05` (md5 9409e7e6) | this cycle PASS: own-DR0.5 gv 12/12, 0 term, prog med 0.83 det / 0.90 sto, slip/m med 1.83/1.66; DR0 retention det gv 6/6, prog 0.84 = parent | ±90° gentle 5 s resamples (abrupt-flip ±90 rung = joyhead90, in flight); L/R asym persists; paddle gait, not hardware-ready; DR0.5 = steering ceiling (full-DR refuted 2×) |
| Stop-and-go driving (35% stop density) | `ppo_goal_cw_walk_stopgo35_c1` | c57 PASS: own-cfg DR0 gv 12/12, 0 term, prog med 0.97 (min ep 0.92); frames: quiet level parks, prompt restarts (re-tracks ~3 s after go) | ±45° cmds @0.05–0.06 m/s, 5 s resample, DR0 only; det slip/m 1.43 (lineage paddle-slide, not hardware-ready) |
| Crouch walking (−20…−60 mm height) | `ppo_goal_cw_walk_lowgait` / `_lowgait30` / `_lowgait40` / `_lowgait50` / `_lowgait60` (md5 4dcc8a34) | c45/c47/c48/c56 PASS at each rung: gv 12/12, 0 term, end-height err ≤7 mm (−60 mm: det 3.9/sto 4.5 mm), det agg slip/m 0.92–1.04 (≤ champion band) | envelope verified to −60 mm (−70 mm rung queued); one sto in-place-paddle ep per panel (lineage brittleness) |
| Rough ground (hfield bumps to 36 mm) | `ppo_goal_cw_walk_terrain10` (md5 57cea2dc) | this cycle PASS: own-cfg amp1.0 det 6/6 gv, 0 term, prog 1.06, slip/m 0.94; flat retention identical (no regression) | sim hfield only; SATURATED — bumps ≤36 mm never perturb the paddle gait; real clutter/obstacles need [CODE] scene work (wishlist 13d/24) |
| Joystick-style abrupt command flips, no falls (DR0) | `ppo_goal_cw_walk_joystick45` (md5 999bd5d6) | c49 PASS: eval_drive JOYSTICK GATE 0 in-envelope falls (fwd/diag/stop-go panel + 3 flip-stress eps); own-cfg DR0 harness gv 12/12, 0 term, prog ~1.04 | envelope heading ≤±45°, speed ≤0.06 m/s; paddle foot-slide (slip/m ~1.4-1.6, not hardware-ready); superseded as driving candidate by joyjit-dr05-c1 (row below) |
| **Joystick flips + physics variation (DR 0.5) — best driving candidate** | `ppo_goal_cw_walk_joyjit_dr05_c1` (md5 7feaf4b9) | c53 PASS: eval_drive JOYSTICK GATE @DR0.2 0 in-envelope falls (panel + 3 flip-stress eps, trk_err 0.025–0.056); own-cfg DR0.5 harness gv 12/12, 0 term, prog med 0.94/0.98, slip/m med 1.38/1.44; DR0 retention gv 12/12 | envelope heading ≤±45°, speed ≤0.06 m/s; backward cmd parks (0.026 m), doesn't fall; paddle foot-slide, not hardware-ready; ±90° widening (joyhead90) queued |
| 60 s endurance walking | `ppo_goal_cw_walk_endur60` | c47 PASS + c48 seed twin: both seeds ~3 m @ 60 s, gv 12/12, 0 term, NO gait decay — endurance is seed-robust (endur60's low slip 0.887 was seed luck; s1: 1.13) | anchorgate lineage (not champion); slip ~0.9–1.1/m; 1/6 sto draw-stall; champion-60s fold queued (endur60-r2) |
| Walking under command-latency jitter (0.5–2.5× fitted servo delay) | `ppo_goal_cw_walk_latjit25` (md5 abd19461) | this cycle PASS: own-cfg jitter panel gv 12/12, 0 term, det med fwd 1.4 m; DR0 no-jitter retention det slip/m 0.96, prog 0.96 (= champion band, nothing forgotten) | isolated 13b axis off champion; extreme-delay draws degrade to a shuffle (2/6 det: ~40% distance, no fall) — median hardened, not the 2.5× tail; paddle slip, not hardware-ready |
| Walking with payload (+0…+50% chassis mass) | `ppo_goal_cw_walk_payload50` (md5 f4619dc5) | c56 PASS: own-cfg mass 1.0–1.5× panel gv 12/12, 0 term, det med fwd 1.31 m @30 s; DR0 no-payload retention gv 6/6, slip/m 1.15, prog 0.95 | isolated axis off champion (dr.mass_scale only, DR0); heaviest draws (~1.4–1.5×) squat-shuffle at ~half speed, slip/m 3.4–3.8 (2/6 det) — solid to ~+40%, top of range marginal; DR0.5 compose queued (payload-dr05); paddle slip, not hardware-ready |
| Walking with off-center payload (CoM shifted ±30 mm x/y) | `ppo_goal_cw_walk_comshift30` (md5 35b892b8) | this cycle PASS: own-cfg com-offset panel gv 12/12, 0 term, det med fwd 1.44 m @30 s; DR0 retention = champion (det fwd 1.58, slip/m 0.95 vs 1.57/0.96) | isolated wishlist-11 axis off champion (dr.com_offset_m=0.03, 2.5× standard envelope); worst offset draw 0.69 m @ slip 2.87 half-speed shuffle (no fall/flag leg); DR0.5 compose queued (comshift-dr05); paddle slip, not hardware-ready |
| Walking with servo deadband up to 3× nominal | `ppo_goal_cw_walk_deadband30` (md5 1ea53e9e) | this cycle PASS: own-cfg deadband 1–3× panel gv 12/12, 0 term, det med fwd 1.41 m @30 s; DR0 retention clean (det fwd 1.58, slip/m 1.00) | isolated 13b/13c axis off champion; no jerky overdrive compensation in frames (gait stays smooth); worst draw 0.68 m @ slip 2.86; DR0.5 compose queued (deadband-dr05); paddle slip, not hardware-ready |
| Walking across grip levels (0.4–1.6× floor friction) | `ppo_goal_cw_walk_fricvar` (md5 7e371de3) | this cycle PASS: own-cfg grip panel gv 12/12, 0 term, det prog med 0.87; DR0 retention det gv 6/6, slip/m 1.09, prog 0.97 (champion band, nothing forgotten) | isolated 13b axis off champion (dr.friction_scale only, DR0); 2/6 slickest det draws churn near-in-place (prog 0.36–0.56, slip/m 2.4–4.2, stride halves) — solid across moderate grip spread, ice-like floors unsolved (paddle gait needs grip); complements friclow row below (two-sided 0.4–1.6 spread kept DR0 retention clean where friclow's slick-only 0.3–1.0 charged it); 1/6 DR0 sto fixed-draw stall = known canary class; DR0.5 compose queued (fricvar-dr05); paddle slip, not hardware-ready |
| Walking on slippery floors (grip 0.3–1.0×) — MARGINAL | `ppo_goal_cw_walk_friclow` (md5 f985fced) | this cycle PASS on the letter: own-cfg grip panel gv 12/12, 0 term, det med fwd 1.23 m (gate 1.2, scrapes) | slick draws transport by SKATING (own-cfg slip/m med 1.73, worst 0.69 m @ 3.79); only axis of the three that charged nominal: DR0 det fwd 1.57→1.43 (ranges disjoint); fric50 refinement queued to pin the clean-grip floor; truly slick ground waits on contact-pricing fix; not hardware-ready |

## Quadruped mode (party-trick line, readiness review P1)

- **Feasibility sweep PASSED (c56, `rl_move/sim/quadruped_feasibility.py`,
  `logs/experiments/quadruped-feasibility/sweep.json`):** four-leg static
  stance is geometrically comfortable. Neutral six-leg stance with fronts
  (L0/L5) raised puts the CoM 68–82 mm OUTSIDE the 4-foot polygon (the
  review's warning was real), but either −40 mm body shift or ~17–31°
  middle-leg (L1/L4) forward splay fixes it: best config (−20 mm shift +
  17° splay, fronts tucked) holds 39 mm margin at 0.6 A max servo current
  (trip 2.5 A) and survives a 6 N forward push; 11 of 18 static passes
  are push-robust. Next rung per review §4: static four-leg-stance RL
  task ([CODE]: needs a quad-hold goal mode — front-feet-clear +
  four-planted + level + low-current reward).

## Stance / posture (older line — see archive for full state)

- Rise/lower heights at DR 1.0: solved pre-walk-campaign (see
  `archive/RL_PLAN_FULL_2026-08-09.md`); lower-line rework per rulings.

## Pending verdicts that would add rows

wander30 (envelope extension), backforth (reverse), standwalksit
(skill chaining), pose-track.
(strafe ±90° landed — see DR 0.5 row above; lowgait30–50,
terrain10 and endur60+s1 landed — rows above.)

## Consolidation status (single deployable policy)

Skills above are SEPARATE checkpoints. The deployable robot needs
either one multi-skill policy (goal-mix training — `standwalksit` is
the first chaining probe) or a deploy-time skill switcher. Champion
strategy: the champion is the BASE the walk line breeds from; skill
passes are preserved here and folded in via goal-mix arms — a
promotion never deletes a skill checkpoint (append-only).
