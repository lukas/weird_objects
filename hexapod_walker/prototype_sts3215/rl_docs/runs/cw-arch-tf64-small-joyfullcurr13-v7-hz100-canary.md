# cw-arch-tf64-small-joyfullcurr13-v7-hz100-canary

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-24T18:19:28+00:00

**pod**: hexapod-mjx-train-8

**steps**: 2000000

**hypothesis**: Plain English: can a small transformer brain that reads a long (0.64s) history learn the joystick walking curriculum at the new 100 Hz control rate, where the old MLP brain's short memory window shrank 4x? Operator-ordered architecture arm (fb_20260824T181220_a3931e): 1-layer causal transformer (d64, 4 heads, ff128) attending over obs.history_frames=64 at control.hz=100 (same 0.64s temporal window the hist16 MLP had at 25 Hz), on the exact V7 certfreeze joystick recipe (WALKCURR_BUCKETS_V7 turning+reversal diet, cert-only stop-freeze, k_walk_stop_current=2.0, slew contract preserved at 37.5 deg/s via max_delta_q_deg=0.375). FROM-SCRATCH by necessity: the trainer refuses MLP->transformer transplants, so this is a new-architecture arm, NOT a continuation of the MLP champion; it separates the attention-architecture question from the rate-conversion question that the concurrent MLP control (cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-r2) answers. This 2M canary answers ONLY mechanism health: does transformer+PPO+walkcurr-V7 boot at 100 Hz/3072 envs on CUDA torch, train stably (no NaN/collapse, usable fps), with reward and walkcurr frontier/eval moving together? Prediction-if-true: reward climbs and b0 frontier metrics (prog, survive) move with it. Prediction-if-false: crash/NaN, dead-flat reward, or reward rising while b0 eval is flat (100 Hz reward/eval mismatch -> audit per-tick pricing before any budget). Strongest alternative: the tf64-small trunk (much smaller than the proven tf 2/128/4/256) is under-capacity -- reads as slow-but-healthy learning, answered at full budget, not here.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. PASS = boots and trains to 2M with no crash/NaN, healthy CUDA fps, reward/frontier/eval AGREEMENT (ep_rew and b0 walkcurr eval move together; per Lukas's rule, reward up + eval flat/worse = STOP and audit reward/eval alignment before ANY more budget or seeds). Directional gait NOT required at 2M (tf-r1 precedent: transformer family needed 40M for gait). If PASS -> respec full 40M as cw-arch-tf64-small-joyfullcurr13-v7-hz100 (acquisition, evidence = this canary + cw-arch-tf-r1-hard1 40M transformer-walking precedent). If FAIL by misalignment -> audit; if under-capacity suspected -> the escalation axis is tf width/layers, not seeds.

**refused_reason**: hexapod-mjx-train-8 code marker 11bb18ed03026cc279ed71f0f9fd07c5cb79b974 != local HEAD de7bdbcc002b207ac6737f6e09bc391730897aa4 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-8 (and snapshot/commit before that if the tree is dirty).

