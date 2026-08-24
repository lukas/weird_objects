# cw-arch-hist16-dep1-c1-joyfullcurr6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T22:28:35+00:00

**pod**: hexapod-mjx-train-4

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1

**wandb_id**: 1yjkjaa1

**hypothesis**: Plain English: teach the 16-frame deployment-contract walker (hist16-dep1-c1) to follow joystick commands in EVERY direction - sideways, backwards, full circle - via a heading-band curriculum (operator order fb_20260823T220651_5c66e3). The lineage only ever trained front-cone (+-45 deg) commands. New WALKCURR_BUCKETS_V6 ladder: bridge at the source operating point (0.05-0.06 m/s straight), then front45 20/60s -> side90 20/60s -> rear135 40s -> rear180 60s -> full-circle 60s -> retain under DR 0.2/0.5. Reward adds the bank-proven direct loaded-slip charge (k_loadslip_excess=0.8, ok 1.2, max 3.0) and smoothed course-direction charge (k_walk_course=1.0, tau 0.75s) so training prices exactly what the full-circle drive eval measures; optional k_walk_cmd_track omitted to keep the coupled-change count down. If-true: precert B0 passes and the frontier promotes past side90 with real rear-heading command following (cmd_prog>=0.65, slip<=2.0, 0 falls at cert). If-false: precert fails (broken transplant) or frontier stalls at side90/rear while reward rises - that is a reward/eval misalignment audit per the 08-21 ruling, not a same-recipe seed relaunch. Strongest alternative: the gait is already heading-symmetric and rear rungs pass trivially (would show as fast full-ladder promotion, still a win).

**gate**: walkcurr V6 precert B0 prog>=0.50; frontier past side90_60s via det certs (cmd_prog>=0.65, slip/m<=2.0, 0 falls); FINAL: eval_drive full-circle --heading-max-deg 180 DR0.5 random 60s flips: 0 falls, rear/side directions actually followed (low dir err/wrong-way), slip not exploding vs parent; DR0 + own-DR walk gates 6/6 gait_valid 0 term; video all six feet cycling, no flag leg

**verdict**: FAIL - the operator's full-circle V6 ladder run finished its whole 40M budget frozen at bucket b1 (front45_20s) and every landed eval fails. Plain words: the robot never learned to actually STOP when told to, so the curriculum never let it try sideways or backwards walking, and 40M steps of hammering one 20s front-cone bucket made the base gait WORSE than its parent. Evidence: (1) promotion b0->b1 at 540k, then b1 cert FAILED its stop check on all ~79 remaining rounds - walkcurr/b1_front45_20s/stop_speed_m_s flat in [0.036,0.047] vs the 0.015 cert bar, reward converged early (quarters 913.9/1062.0/1054.1/1042.6) so per the 08-21 ruling this is misalignment, not undertraining; (2) joygate FAIL: 8 falls/48 (5 at DR-0!), dir_err 40.69>40, gait_valid 0.917 - parent lineage passed this gate 0 falls; (3) own-DR walk FAIL: det 1/6 over_current termination, gait_valid 3/6 det, dir_err med 42.1. ROOT CAUSE (corrects the earlier triage note): k_walk_freeprog was NEVER active in this run (default 0.0, not in the cfg-set list; no freeprog keys in W&B) - the real defect is that NOTHING in this cfg prices stop-command speed except the Gaussian velocity kernel, whose optimum-vs-creep margin is shallow (still 2.0/tick vs 0.04-creep 1.45/tick, +0.55/tick) while prog-gate/course/park-duty/step/drag are all guarded by s_ref>1e-3 and pay/charge NOTHING on stop ticks. The cert bar (mean stop-tick speed<=0.015 incl. decel transient) has no matching reward term, so PPO's converged optimum is the observed ~0.04 creep. WHAT'S NEXT: add a dedicated walk-mode stop charge (new cfg key, default OFF, bit-exact off; linear speed charge scaled to the 0.015 bar on s_ref~0 ticks), prove it in the semantics bank (still > creep > walk-during-stop under this exact cfg), and relaunch the V6 ladder from the SAME parent ppo_goal_cw_arch_hist16_dep1_c1.zip (NOT this checkpoint - it regressed: falls at DR0, over_current; phasedir9-vs-9b precedent on converged wrong basins). Checkpoint kept append-only, not a champion candidate.

