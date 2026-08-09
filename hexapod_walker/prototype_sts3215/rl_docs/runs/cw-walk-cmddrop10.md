# cw-walk-cmddrop10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T17:45:56+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip

**wandb_id**: ntme45bm

**hypothesis**: WISHLIST 13c axis (servo command dropout): the real Feetech bus loses SyncWrite commands; the sim champion has never trained against lost commands. ISOLATED axis off the no-DR champion: dr-scale 0.0 + ONLY dr.cmd_drop_prob_max=0.10 (2x the full-DR ceiling - each episode drops up to 10% of control ticks). Plain: the walk must not fall apart when some commands never arrive. If-true: gait absorbs dropouts (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 clean retention holds - dropout robustness is free, keep the axis. If-false: dropped ticks break the gait rhythm (terminations, prog craters, or slip blowup) - the deployment stack needs command-resend at the bus level instead of policy robustness. Strongest alternative: policy survives by stiffening into a slower shuffle - frames + stride/cadence will show it.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.cmd_drop_prob_max=0.10, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 no-dropout retention det 6/6 gv, det slip/m <=1.24; frames watched det

**note**: OVERLAP FLAG (c59): same dr.cmd_drop_prob_max axis as cw-walk-cmddrop20 (max 0.20, RUNNING t10) — concurrent refill collision from the dry READY well. Triage the pair as ONE intensity-ladder study, and run the PARENT (longdist-r2) under the same own-cfg drop spread first (torquedroop c59 lesson: champion may already cover the axis for free; the gate letter cannot tell exposure effect from pre-existing tolerance).

