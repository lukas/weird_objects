# cw-walk-fricvar-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T18:45:11+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_fricvar.zip

**wandb_id**: 40xnv2in

**hardware_ready**: no

**hypothesis**: Composition rung off today's fricvar PASS (same move class as payload-dr05/comshift-dr05/deadband-dr05, pattern 'widen-then-harden composes' proven by head90-dr05): friction 0.4-1.6x competence was proven at dr-scale 0.0 only. One variable off fricvar: dr-scale 0.0 -> 0.5 (model DR at half strength) while keeping the absolute dr.friction_scale=0.4,1.6 override. Plain: grip-level robustness must survive general physics spread, not just nominal sim. If-true: the axes stack - own-cfg DR0.5+friction harness gv 12/12, 0 term, det median prog >=0.75, and DR0 nominal retention stays in the champion band (slip/m <=1.15) - friction joins the deployable robustness recipe. If-false: DR draws plus slick floors compound (terminations, or the slick-draw churn spreads from the 2/6 tail to median draws) - friction stays a nominal-sim skill and we note the compose ceiling. Strongest alternative: passes with slip creeping past the champion band under DR like other DR composes - then the caveat is slip, not gait.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.friction_scale=0.4,1.6, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median prog_ratio >=0.75; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.15; frames watched det

**verdict**: PASS: friction 0.4-1.6x competence survives DR0.5 compose — own-cfg gv 12/12, 0 term, det prog med 0.89 (gate >=0.75), det fwd med 1.32m; DR0 nominal retention CLEAN (gv 6/6, slip 1.10 <=1.15, prog 0.96 — no payload-dr05-style erosion). Frames watched det: upright, level, six legs cycling, real travel. Pre-registered caveat landed: sto slip creeps under DR (med 1.48, worst 2.48/prog 0.61 slick-draw churn) — caveat is slip, not gait. Friction joins the DR0.5 compose recipe. Paddle lineage, NOT hardware-ready.

