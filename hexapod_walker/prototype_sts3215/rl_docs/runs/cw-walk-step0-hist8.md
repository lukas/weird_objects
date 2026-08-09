# cw-walk-step0-hist8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T02:24:32+00:00

**pod**: hexapod-sweep-s6

**steps**: 4000000

**parent**: none (from scratch - obs change forces new baseline; plan-sanctioned)

**wandb_id**: wgba1l9o

**hypothesis**: Temporal-actor rung (plan Walk rung 1, lit-review priority): 8-frame (~300ms) obs/action history provides the substrate for rhythmic gait (internal phase/clock) the reactive MLP lacks. ONE variable vs archived cw-walk-step0 4M from-scratch baseline: obs.history_frames=8, all else identical (walk-only, DR0, step-event package, std 1.0 / ent 0.01 / target_kl 0.02, seed 0). If-true: step0 gate clauses met at 4M AND cadence regularity beats step0-4M baseline outside noise (per-leg swing-count spread tightens vs det rows like [7,6,5,7,13,3]; duty split narrows). If-false: (a) no gait by 4M (step_event fails to rise; wider first layer slows from-scratch PPO) -> history hurts DR0 sample efficiency, retry only as warm start after pricing work; (b) gait forms, cadence/tracking indistinguishable -> capability not binding, all-in on pricing (ii)/(iii). Strongest alternative: skating/overspeed/park are pure pricing defects history cannot touch - distinguished because if-true is claimed on CADENCE (swing CV, duty spread), not tracking. Probe probe-walk-step0-hist8 PASSED (150k, obs 576=8x72, rew 60->142, no traceback). Snapshot c50aadd.

**gate**: step0 gate at 4M vs archived logs/ckpt_eval/cw_walk_step0_4M_gate baseline: DR0 det AND sto, >=10cm fwd, six legs cycling, duty ~[0.2,0.9], >=2 swings/leg, no drag/park, video pathology-first; plus cadence-regularity comparison (swing-count spread, duty split)

**verdict**: PASS on step0 gate clauses (12/12 gait_valid, fwd 0.187-0.484m, 0 terms, all duties [0.28,0.71], min 3 swings/leg, det+sto, all 12 eps on video) BUT registered cadence question NEGATIVE: det swing-CV 0.299 vs baseline 0.332 (overlap), sto 0.271 vs 0.232 (overlap, worse); only det duty-spread tightened (0.135 vs 0.345) - single-metric det-only hint, not evidence; new leg-1/3 double-time pattern. HYPOTHESIS REFUTED (branch b): temporal capability not binding at DR0; pricing (kgate) stays the line. NOT HARDWARE-READY: slip/m 1.5-2.0, sprawly stance, irregular cadence, std 2.2, DR0 only. Champion unchanged (h15b d0a12a94). Lineage closed at 4M; kill any mechanical -c1. RL_LOG cycle 22.

