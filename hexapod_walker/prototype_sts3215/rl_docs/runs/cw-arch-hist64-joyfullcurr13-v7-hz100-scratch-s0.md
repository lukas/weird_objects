# cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T18:34:15+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100

**hypothesis**: Plain English: two transplant attempts proved a 25 Hz brain cannot be weight-converted to 100 Hz (naive frame-rate mismatch: precert prog 0.203; verified rate-matched inputs: prog 0.052 — WORSE, because with the slew contract preserved a 25 Hz policy's per-decision motion authority is quartered at 100 Hz), so per fb_20260824T180427_4c2e26's own fallback this is the LABELED from-scratch arm: same V7 certfreeze joystick ladder, control.hz=100, max_delta_q_deg=0.375 (37.5 deg/s physical slew), obs.history_frames=64 (100 Hz-native 640 ms context), fresh init BY DESIGN — the fresh init IS the hypothesis (rate-native action rhythm must be learned, not transplanted), which is the recorded reason to override warm-start-default. Complementary to the concurrent -r2 adaptation arm (guard-dropped warm start): r2 asks if a degraded 25 Hz gait can adapt, this asks if the recipe can learn 100 Hz walking at all. RATE CAVEAT: 40M ticks at 100 Hz = 1/4 the simulated seconds of a 25 Hz 40M run. Prediction-if-true: the V7 ladder climbs from b0 with reward and frontier promotions rising together at 100 Hz. Prediction-if-false: b0 never clears with flat reward at adequate budget — the recipe/reward is not learnable at this rate under the current step cap; next is a rate-normalized action/obs contract audit, not seeds. Strongest alternative: learns b0 but slower than 25 Hz equivalents purely from the 4x sim-seconds deficit — reads as continuation candidate per 08-21, not failure.

**gate**: PASS: frontier promotions past b0 with reward/eval AGREEMENT at 100 Hz AND 60s randomized joygate at 100 Hz: falls<=2/48, no over_current cluster, directions followed on side/rear/turn/reversal stress, slip <=~2.9/m, video shows all six feet cycling. PARTIAL: genuinely learning (b0+ promotions, improving rung evals) but short of the 25 Hz lineage at budget end — continuation candidate (expected: 1/4 sim-seconds). FAIL: b0 never promotes with reward AND rung metrics flat at adequate budget, or reward rises while rungs stay flat (100 Hz reward/eval mismatch — audit before any seed sweep).

**verdict**: Never trained (0 PPO steps) -- died at env-construction with 24x silent SIGBUS (exitcode=-7), auto-marked FAILED by the launch-verify window. NOT the rate-learnability question this arm was meant to answer. Root cause traced this cycle: obs.history_frames=64 (the 100Hz-native 640ms context this arm needs) quadruples n_obs (1152->4608), needing ~101MB of /dev/shm at --n-envs 3072 -- but the pod it landed on (train-2) is one of the fleet's legacy-64M-shm pods that never got the 08-10 dshm-4Gi fix (a concurrent cycle's parallel fleet audit this same cycle found train-0/2/3/4 stuck on the old 64M tmpfs while train-1/5/6/7/8/9/10/11 have the fixed 4.0G). Real fix is routing to a fixed pod, not shrinking n-envs -- relaunched UNCHANGED (n-envs 3072) pinned to train-1 (verified 4.0G shm, CUDA-capable, free). Also landed a permanent guard (mjx_sharded_vec_env._check_shm_budget) so any future obs/DR-field growth or accidental legacy-pod routing fails fast with a clear message and the safe --n-envs spelled out instead of 24 silent worker crashes -- 5 new unit tests (rl_move/tests/test_mjx_shm_budget.py). The from-scratch-100Hz-learnability question itself is UNANSWERED, now actually running on train-1.

**failed_reason**: ROOT CAUSE (this cycle): infra, not a training/rate-learnability failure. Landed on train-2, a legacy-64M-/dev/shm pod (never got the 08-10 dshm-4Gi fix -- confirmed via a concurrent cycle's parallel fleet audit finding train-0/2/3/4 still legacy). hist64's obs.history_frames=64 needs ~101MB of shm at n-envs=3072 -- 'mount -o remount' is permission-denied, so the launch died as 24 silent SIGBUS (exitcode=-7) workers with no python traceback before any PPO step ran. Fix: relaunched unchanged (n-envs 3072) pinned to train-1 (verified 4.0G shm + CUDA capable). Also landed mjx_sharded_vec_env._check_shm_budget (fails fast with a clear message pre-allocation instead of silent worker crashes; 5 new unit tests, rl_move/tests/test_mjx_shm_budget.py).

