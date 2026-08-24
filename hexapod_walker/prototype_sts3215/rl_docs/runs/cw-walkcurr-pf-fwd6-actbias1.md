# cw-walkcurr-pf-fwd6-actbias1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T00:37:59+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: dl84nfsm

**hypothesis**: Plain English: every one of the 20+ rung-1 arms so far (RND, height-gate, park-duty, GRU, gSDE, reward-scale) converges to the identical belly-sit-like collapse regardless of reward mechanism -- because a zero-training, zero-reward physical probe (env.step(np.zeros(18)) in a loop, no policy) reproduces that EXACT signature by itself: this recipe's raw-joint action space maps a=0 to the hardware axis MID-RANGE (hip=-25deg/knee=65deg), not the settled standing pose (q_nom ~ hip=16deg/knee=85deg, matching the bank's WALK_PLANT=20,80). A near-zero-mean PPO policy (true at init, and per the rscale wave's evidence, true for a long time after) physically sinks the chassis -110mm over 2s while staying level -- the belly-sit signature -- independent of what the reward function prices. This is a sim/action-space defect upstream of every reward-mechanism fix tried, per the 08-21 ruling's own root-cause chain. Fix: goal.joint_action_bias_hip_deg/_knee_deg (new, cfg-gated, bit-exact-when-0 lever in joint_task.py) re-centers a=0 on the honest stance (hip 20/knee 80) via a pure translation of the action's zero point, full hardware range still reachable. Single lever vs the strongest existing rung-1 baseline (rscale50, the only prior arm whose walk_freeprog_score ever trended toward 0 instead of flat). Bank/unit evidence: test_joint_action_bias.py pins the defect (constant zero action collapses -110mm/level within 50 ticks) and proves the fix (same probe stays <30mm) and proves bias=0 is bit-exact; full walkcurr semantics bank re-run 56/56 green (reward semantics untouched).

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- prog_ratio>0 and gait_valid on >=4/6 det episodes with visible forward travel on video, env/walk_freeprog_score leaves [-0.10,-0.05] and trends toward/past 0 by 2M, clip_fraction stays healthy (no crush). Additionally watch env/height_err_mm at t=0 vs rscale50 (should start near 0 rather than immediately climbing) and direction_err_mean_deg (should move off the current ~80deg toward 0, since the previous 'jitter' framing means legs already cycle, just not coherently). PASS = rung-1 lands, move to rung 2. FAIL with the same collapse signature despite a verified-correct neutral pose = the zero-point defect was real but not the only blocker; next fork pairs this bias with a tighter --log-std-init (PPO's own exploration noise may random-walk back into the collapse basin regardless of where the mean sits) before trying the foot-contact-charge mechanism or BC-kickstart.

