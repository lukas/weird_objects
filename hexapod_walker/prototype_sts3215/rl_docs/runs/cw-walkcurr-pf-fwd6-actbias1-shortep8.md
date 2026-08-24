# cw-walkcurr-pf-fwd6-actbias1-shortep8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T03:50:16+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1

**wandb_id**: hol9okg5

**hypothesis**: Plain English: dose sibling of -shortep3, same reasoning (episode-length/reset-frequency is the one axis the 13-mechanism-refuted walkcurr rung-1 campaign never tried; see walkcurr STATUS.md 08-24 ~03:3x). This arm sets --training-episode-seconds=8.0 (vs -shortep3's 3.0 and the parent's implicit 25.0) -- roughly 3x more resets per step budget than baseline, but with >10 EMA tau-windows (walk_kernel_vel_tau_s=0.75) per episode so the freeprog reward signal has time to stabilize, ruling out the 3s sibling's 'metric too short to read' alternative. Off the same actbias1 base, single lever, from scratch, 2M steps. Prediction-if-true: walk_freeprog_score trends toward/past 0 and/or the 25s gate eval shows real stepping, WITH a cleaner (less noisy) freeprog trace than -shortep3 since the training signal itself has more time per episode to resolve. Prediction-if-false: same static-stand-or-thrash signature -- if BOTH doses fail with the aligned-FAIL signature (reward flat/falling, healthy optimizer, eval flat), the episode-length-curriculum axis is closed and BC-kickstart becomes the sole remaining lever (q_20260824T0233Z).

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel at the standard 25s EVAL horizon (unchanged) -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: env/walk_freeprog_score trend vs the dead band; env/height_err_mm stays in actbias1's healthy ~15-30mm band; clip_fraction stays healthy. Read jointly with -shortep3 for the dose-response shape before closing or promoting the episode-length axis.

**verdict**: Result: FAIL -- the 8s training-episode dose (the sibling designed to rule out '3s is too short for the freeprog EMA to read') does NOT unfreeze rung-1 either; identical static-stand-or-thrash signature to -shortep3 and all 14 prior refuted mechanisms. Evidence (own-cfg C-env det panel, run's own pod, eval-only): det gate 0/6 gait_valid, legs [0,2,3,5] sacrificed IDENTICALLY on all 6/6 episodes, forward_dist_m pinned at 0.03 every episode, speed_mean_m_s 0.002-0.003 (cmd 0.05-0.06), contact sheet bit-identical static tripod-lock crouch across all 10 frames, zero net translation -- indistinguishable from -shortep3's video. Sto: gait_valid 6/6 but in-place thrash (slip/m 30-48, direction_err 84-93deg = chance-level, forward_dist_m 0.02-0.08/25s). W&B: env/walk_freeprog_score stayed pinned in [-0.085,-0.056] the entire 2M run, no trend toward 0 (matches -shortep3's [-0.085,-0.04] bounce, both far worse than the rscale50 parent's monotonic -0.10->-0.015 climb); env/height_err_mm 5->20-24mm, inside actbias1's healthy non-collapse band (rules out belly-sit); train/clip_fraction healthy 0.002->0.09-0.12 rising all run (no optimizer crush); rollout/ep_rew_mean DECLINES every quarter (47.8/43.5/41.3/41.1). Why: aligned FAIL per the 08-21 ruling -- reward declining, eval flat/zero, optimizer healthy, adequate 2M budget with >10 EMA tau-windows/episode (rules out the 'metric too short to stabilize' alternative this arm was built to test) -- a genuine refutation, not misalignment or undertraining. Combined with -shortep3 (3.0s dose, also FAIL, also verdicted this cycle-family): the episode-length/reset-frequency axis is dose-insensitive across an 8x span (3s/8s/implicit-25s) and is now CLOSED, the 15th independently-designed rung-1 mechanism/architecture/schedule class refuted, all aligned FAILs. What's next: per OPERATOR_QUESTIONS.md q_20260824T0233Z's own pre-registration, this was explicitly the last rule-(a)-compliant (no gait clock/BC teacher/motion prior) lever this track had identified -- no further non-BC mechanism is pre-registered or credible. The BC-kickstart operator ruling is now the track's SOLE remaining lever with nothing else in flight; STATUS.md and the question note updated this cycle to reflect the axis is fully closed, not just 1/2 read. No new walkcurr rung-1 arms should be launched pending that ruling.

