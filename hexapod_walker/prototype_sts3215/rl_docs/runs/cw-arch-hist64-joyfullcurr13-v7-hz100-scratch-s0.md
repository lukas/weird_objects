# cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-24T18:34:15+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100

**hypothesis**: Plain English: two transplant attempts proved a 25 Hz brain cannot be weight-converted to 100 Hz (naive frame-rate mismatch: precert prog 0.203; verified rate-matched inputs: prog 0.052 — WORSE, because with the slew contract preserved a 25 Hz policy's per-decision motion authority is quartered at 100 Hz), so per fb_20260824T180427_4c2e26's own fallback this is the LABELED from-scratch arm: same V7 certfreeze joystick ladder, control.hz=100, max_delta_q_deg=0.375 (37.5 deg/s physical slew), obs.history_frames=64 (100 Hz-native 640 ms context), fresh init BY DESIGN — the fresh init IS the hypothesis (rate-native action rhythm must be learned, not transplanted), which is the recorded reason to override warm-start-default. Complementary to the concurrent -r2 adaptation arm (guard-dropped warm start): r2 asks if a degraded 25 Hz gait can adapt, this asks if the recipe can learn 100 Hz walking at all. RATE CAVEAT: 40M ticks at 100 Hz = 1/4 the simulated seconds of a 25 Hz 40M run. Prediction-if-true: the V7 ladder climbs from b0 with reward and frontier promotions rising together at 100 Hz. Prediction-if-false: b0 never clears with flat reward at adequate budget — the recipe/reward is not learnable at this rate under the current step cap; next is a rate-normalized action/obs contract audit, not seeds. Strongest alternative: learns b0 but slower than 25 Hz equivalents purely from the 4x sim-seconds deficit — reads as continuation candidate per 08-21, not failure.

**gate**: PASS: frontier promotions past b0 with reward/eval AGREEMENT at 100 Hz AND 60s randomized joygate at 100 Hz: falls<=2/48, no over_current cluster, directions followed on side/rear/turn/reversal stress, slip <=~2.9/m, video shows all six feet cycling. PARTIAL: genuinely learning (b0+ promotions, improving rung evals) but short of the 25 Hz lineage at budget end — continuation candidate (expected: 1/4 sim-seconds). FAIL: b0 never promotes with reward AND rung metrics flat at adequate budget, or reward rises while rungs stay flat (100 Hz reward/eval mismatch — audit before any seed sweep).

**verdict**: Pure infra death, zero training: the sharded vec-env workers SIGBUS'd (Fatal Python error: Bus error in mjx_sharded_vec_env._worker_main) at _setup_learn because the hist64 obs shm buffer (3072 envs x 4608 dims x f32 = 56.6MB) nearly fills the pods' 64MB /dev/shm; the env's own _check_shm_budget guard passed by a sliver and the first full write went over. Not a recipe/science result — the from-scratch 100 Hz question is untouched. Retry (checkup DEAD->retry-once): same arm at --n-envs 2048 (obs 37.7MB, ~46MB total, real margin) as cw-arch-hist64-joyfullcurr13-v7-hz100-scratch2k-s0. Data point for the fleet: any obs.history_frames=64 run needs n_envs<=2048 on the current 64MB-shm pods.

**failed_reason**: run never appeared as 'running' in W&B within 240s

