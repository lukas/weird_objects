# cw-arch-joystick-canary1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-17T01:28:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: hkliaidm

**hypothesis**: Teach the robot to walk where a joystick points by first proving the TRAINING MACHINERY is healthy: this 2M-step canary tests whether the operator-approved update-path redesign (fb_20260817T005114_775298) lets a from-scratch policy learn with a WORKING value critic and controlled updates, where four prior attempts collapsed. Changes vs the failed cw-arch-joystick-long-scratch3 recipe, all landed+tested this cycle (snapshot fd051918, JOYCANARY bank + test_value_learning.py green): bounded terminal cost (term_cost_max=240 caps the -730 critic cliff; bank-recalibrated after cap 60 reopened the c2 drag-then-fall exploit), walk-height income gate calibrated from the honest scripted gait (sigma 11mm; honest band +0.7..+7.3mm) with low-height termination tightened 90->25mm, separate actor/critic optimizer groups (actor 1e-4 decaying to 1e-5, critic constant 3e-4), 3 PPO epochs, target_kl 0.01, transactional updates (policy rollback + actor-LR halving on realized KL>0.03), best-checkpoint retention + joint reward/survival/direction regression auto-stop, critic-EV hard-failure auto-stop (EMA<0.05 @1.5M), and a staged command curriculum (fwd/back -> headings/circles/squares -> full stress_mix via sched on goal.walk_cmd_stage; switches stay instantaneous). Frozen-rollout audit proved the transformer critic CAN fit returns, so if EV now rises the cliff was the blocker; if EV stays ~0 the difficulty is elsewhere and the line goes back to the operator. Mechanism health only, per phase contract; mature walking is NOT judged here.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY PROMOTION GATES (all required for any long-run follow-up; refusal on any hard failure, and NEVER promote on reward alone): (1) train/explained_variance EMA clearly >0 and rising by 1.5M (auto-stop enforces; EV~0 = hard failure); (2) update health: median approx_kl <=0.01, no ACCEPTED update >0.03 (kl_rollback events are visible and bounded, not runaway), clip_fraction not pinned near 0.4; (3) no sustained episode-length collapse after the run's own peak; (4) joystick/v_along_m_s and eval/walk/dir_err_deg_mean improving TOGETHER over the run; (5) walk_height_factor near 1 in training (no crouch), no walk_low_height termination epidemic after grace; (6) final video shows upright stepping responses to command switches, all six legs cycling. FAIL branch: no 40M clone; report to operator with per-gate readings.

