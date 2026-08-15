# cw-joystick-translate-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T12:29:31+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**wandb_id**: tvzk2nn8

**hypothesis**: Does a randomly-initialized policy learn joystick-directed walking better than the champion-lineage continuation? This from-scratch twin of cw-joystick-translate1 - identical trainer config, seed, reward and command distribution; the ONLY difference is no --init-from (random init) - separates two explanations for translate1's current low-motion phase: C2's inherited narrow-direction/unsafe local optimum clashing with the new gait gate, vs the all-direction reward setup itself favoring long low-motion survival (explicit operator directive fb_20260815T122345_2c039a). Fresh init IS the hypothesis (warm-start default waived by the operator). Prediction-if-scratch-outpaces: the inherited optimum is the drag - translate1's lineage is a liability. Prediction-if-scratch-also-stalls-at-low-motion: the reward setup itself favors survival over movement. Compare on joystick/v_along_m_s (+_cumulative), env/reward_task, rollout/ep_len_mean, reward per active tick.

**gate**: Comparator, not a promotion arm: report acquisition (joystick/v_along_m_s + cumulative trend vs cw-joystick-translate1 at matched steps), survival (ep_len_mean, falls), and gait quality (per-leg duty_cycle >= 0.10 on all six, no sacrificed leg) SEPARATELY - no global PASS/FAIL from total episode return (fb_20260815T122345_2c039a, fb_20260815T121512_00533c). Stop early only on numerical/mechanical failure.

