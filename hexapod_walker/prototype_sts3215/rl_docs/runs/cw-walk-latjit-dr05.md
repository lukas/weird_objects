# cw-walk-latjit-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T18:55:22+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_latjit25.zip

**wandb_id**: dujb0tue

**hypothesis**: Compose rung off the latjit25 PASS (same widen-then-harden pattern as payload-dr05/comshift-dr05/deadband-dr05/fricvar-dr05): latency-jitter competence was proven at dr-scale 0.0 only. One variable off latjit25: dr-scale 0.0 -> 0.5 (model DR at half strength) while keeping dr.latency_scale=0.5,2.5. Plain: tolerating slow/erratic servo timing must survive general physics spread, not just nominal sim. If-true: own-cfg DR0.5+latjit harness gv 12/12, 0 term, det median fwd >=1.1m @30s and DR0 no-jitter retention stays in the champion band - latency joins the deployable robustness stack. If-false: DR draws plus 2.5x latency overload the gait (terminations or the known 2.5x-tail progress halving spreads to median draws) - latency stays a nominal-sim skill, estimator rung inherits it. Strongest alternative: it holds but slip creeps past the champion band under DR like other composes - then the caveat is slip, not gait.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.latency_scale=0.5,2.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1 m; plus DR0 no-jitter retention det 6/6 gv, det slip/m <=1.24; frames watched det

