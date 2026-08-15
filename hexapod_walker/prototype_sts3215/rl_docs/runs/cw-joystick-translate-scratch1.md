# cw-joystick-translate-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-15T12:29:31+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**wandb_id**: tvzk2nn8

**hardware_ready**: False

**hypothesis**: Does a randomly-initialized policy learn joystick-directed walking better than the champion-lineage continuation? This from-scratch twin of cw-joystick-translate1 - identical trainer config, seed, reward and command distribution; the ONLY difference is no --init-from (random init) - separates two explanations for translate1's current low-motion phase: C2's inherited narrow-direction/unsafe local optimum clashing with the new gait gate, vs the all-direction reward setup itself favoring long low-motion survival (explicit operator directive fb_20260815T122345_2c039a). Fresh init IS the hypothesis (warm-start default waived by the operator). Prediction-if-scratch-outpaces: the inherited optimum is the drag - translate1's lineage is a liability. Prediction-if-scratch-also-stalls-at-low-motion: the reward setup itself favors survival over movement. Compare on joystick/v_along_m_s (+_cumulative), env/reward_task, rollout/ep_len_mean, reward per active tick.

**gate**: Comparator, not a promotion arm: report acquisition (joystick/v_along_m_s + cumulative trend vs cw-joystick-translate1 at matched steps), survival (ep_len_mean, falls), and gait quality (per-leg duty_cycle >= 0.10 on all six, no sacrificed leg) SEPARATELY - no global PASS/FAIL from total episode return (fb_20260815T122345_2c039a, fb_20260815T121512_00533c). Stop early only on numerical/mechanical failure.

**verdict**: FAIL (comparator, not promotion) - from-scratch init does not fix the joystick-translate stall, it just cheats a DIFFERENT way. Acquisition: joystick/v_along_m_s_cumulative flat ~0.003-0.006 m/s for all 40M steps (never trended), train/wrong_way_frac pinned ~0.43-0.44 from step 0 -- identical shape to the warm twin cw-joystick-translate1. Harness confirms: prog_ratio med 0.11-0.17 (promotion band 0.75-1.25), fwd med 0.05-0.09m over a 60s episode commanding 0.03-0.06 m/s (~2-3m expected) across all 4 passes (gate det/sto, own-DR det/sto), despite env/reward_task climbing to 0.85-0.89 -- reward/task-metric disconnect, RUN_INTERPRETATION rule 2. Survival: fully solved, ep_len saturates 1500/1500, ~1-2 terminations total across ~26k episodes. Gait quality DIFFERS from the twin: gait_valid 6/6 on all 4 passes, no sacrificed leg (twin had a persistent single-leg park/stilt, sac[2], gait_valid 1-2/6) -- but video shows why that does not matter: all six legs cycle vigorously while the body sits on the same floor tile the entire episode (walk_det_0/4, walk_sto_1 frame strips, t=0 to t=48-60s, no visible displacement) -- a march-in-place exploit, RUN_INTERPRETATION_RULES #4 known-exploit video pattern, one-line STOP verdict. CONFIRMS the operator prediction-if-scratch-also-stalls branch: the joystick-translate reward/command setup itself favors low-motion survival over real command-following, independent of lineage -- two independent inits (warm c2-derived stilt-leg, fresh random march-in-place) both found zero-net-motion cheats. Recipe closed, no forensics/continuation/re-run. multitask one-time pause exception now spent for both arms; 08-13 pause otherwise stands.

