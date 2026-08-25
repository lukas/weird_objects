# cw-standwalk-stance-mesh1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T04:07:55+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**hypothesis**: Plain English: can the proven rise/hold/lower (stance) skill be re-learned FROM SCRATCH on the new mesh-accurate 3.50 kg robot model at 100 Hz, using the stance champion lineage's modern reward stack with mesh-recalibrated heights? Stage-1 arm of the standwalk track (operator kick 08-25). Recipe = footlow2-lineage joint_goal (goal-mix hold=.1/rise=.45/lower=.45, 15s episodes, rise-ref tracking k=2.0 + posture/income/finish gates + hold_still_gate/hold_flag_fade + rise RSI 0.5) minus its impossible-on-mesh parts (primitive warm start, bc_anchor), fresh-init field standard (log-std 0, ent .005, DR 0.2). Mesh calibrations measured this cycle: plant settles h_rel=82.96mm (vs 131.94 primitive) -> goal.rise_height_mm=[79,87], actions.max_height_mm=88; the 25Hz rise_ref_belly2plant.npz EXECUTES on mesh (time-aligned open-loop replay ends valid plant 3/3 seeds; trainer consumer _rise_ref_clock is time-based). Bank-checked ON MESH under this exact stack (rise replay 2703 > mesh-honest partial 536 > flagleg 395 > stilt -47 > freeze/thrash negative; lower honest 2131 > partial 629 > refuse -64<0, posture-strict rejects aloft cheats; hold quiet 1472 > stepping 870 > flag 50) after fixing the bank's 25-vs-100Hz replay-rate defect. Prediction-if-true: rise ends valid plant (height within +-15mm of target) and lower posture-strict on the panel majority by 20M, zero falls/tips. Prediction-if-false: (i) reward rises but rise stays invalid -> mesh-specific misalignment, dig-in per 08-21 ruling before any continuation; (ii) reward AND task metrics flat -> mesh joint_goal MJX wiring defect, infra dig-in. Strongest alternative: the +66% mass needs a DR/budget ladder like the original dr08->dr10 - 20M@DR0.2 is the first rung, not the last. Known caveat to watch on video: mesh lower can be satisfied by collapse (bank thrash s2 banked 1852 by lucky fall-to-belly) - crash-lowering instead of controlled descent = misalignment to price in a follow-up rung.

**gate**: Stage-1 pre-gate read at 20M: pod_eval stance panel (rise/hold/lower) n>=12 det+sto at DR-0 + own-DR(0.2): zero falls/tips; rise ends valid plant (PLANT_SPEC, height within +-15mm of the [79,87] mesh band) >=5/6 det AND sto; lower posture-strict (|h_err|<=15mm, all pads <=60mm) >=5/6; hold quiet 6/6 no creep, no crash-lowering on video. First passing run's numbers become the recorded mesh reference band (track STATUS). Read jointly with -seed1/-seed2: 2-3/3 healthy = recipe robust; 0-1/3 = seed-dependent or recipe gap.

