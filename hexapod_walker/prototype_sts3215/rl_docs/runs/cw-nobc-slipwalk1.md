# cw-nobc-slipwalk1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DEFECTIVE

**created**: 2026-08-21T13:51:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**hardware_ready**: False

**hypothesis**: Teach a robot that knows nothing about walking to walk forward while keeping its feet planted: it earns reward for travelling in the commanded direction at ANY speed (faster is never punished), pays hard for sliding a loaded foot along the ground, and pays for standing still or marching in place. This is the operator's 2026-08-21 from-scratch anti-slip experiment (order 20260821T133626Z, asked by Lukas): no imitation anchor, no scripted-gait teacher, no pretrained locomotion brain, and exactly ONE fixed forward direction in this first rung — direction changes come later only if this rung shows real, low-slip movement. New default-off reward code carries it: reward.k_walk_freeprog (direction-first income with no speed target) and reward.k_walk_idle_charge (anti-park travel floor), paired with the structural episode-accumulated loaded-slip charge reward.k_loadslip_excess and the all-six-legs walk_gait_gate. Prediction if TRUE: by 2M the deterministic video shows six legs cycling and the body actually covering ground, median travel >= 0.15 m per 15 s episode, direction error <= 30 deg, slip/m <= 3. Prediction if FALSE: the same freeze / march-in-place / skate fingerprint every previous from-scratch gait arm produced (travel ~0 m), which stops this sub-line rather than buying more steps. Strongest alternative explanation to guard against: the anti-slip charge makes ANY early foot motion so expensive that freezing wins locally — the SLIPWALK preflight bank was calibrated exactly against that (stepping-in-place still out-earns parking, and every real-travel rung out-earns both).

**gate**: Deterministic DR-0 harness, walk mode, 6 episodes x 15 s, run's own cfg. PASS requires ALL: (a) zero falls/terminations, 6/6 episodes survive; (b) real distance: median along_dist_m >= 0.15 m/episode (scripted tripod at this command measures 0.22 m) - the near-zero fingerprint of dragstance1/rsi1/slowfirst1/sched1/ease1 is a FAIL; (c) direction: median walk direction error <= 30 deg, no wrong-way episode; (d) legitimate stepping: gait_valid >= 4/6 AND video shows all six legs cycling contact/swing (flag-leg, stilt, tripod park = automatic STOP); (e) low slip absolute AND normalized: median slip_per_m <= 3.0 AND median slip_m_total <= 1.0 m (the distance floor in (b) keeps slip_per_m meaningful). Harness walk 'success' (vel_err <= 0.03 in-band speed) is deliberately NOT part of this gate - this arm has no speed target. KILL EARLY at the 1M periodic eval if travel is still ~0 (freeze/march) or slip/m > 6 (skating). PASS stages rung 2 (second direction, then commanded direction changes); FAIL verdicts honestly and stops the sub-line - no re-run with more steps.

**verdict**: DEFECTIVE LAUNCH, not a science result: 0 PPO steps trained. All 24 sharded-env workers SIGBUS'd (exitcode -7) at the very first env.reset(), before any training. Root cause: obs.history_frames=16 at --n-envs=4096 on hexapod-mjx-train-0, which still has the k8s-default 64M /dev/shm (documented COMMANDS.md #13c gotcha — history_frames=16 doubled the obs shm block past the 64M cap; pods 1/5/7/9/10/11 already carry the 4Gi dshm fix, 0/2/3/4/6/8 do not). No verdict possible on the anti-slip freeprog/idle-charge reward mechanism from this attempt. Retried as cw-nobc-slipwalk1-r1, identical spec, placed on 4Gi-dshm pod hexapod-mjx-train-1 — already RUNNING and reached its full 2M steps within minutes (MJX/warp throughput), finishing eval/checkpoint now; triage next cycle.

**failed_reason**: run never appeared as 'running' in W&B within 240s

