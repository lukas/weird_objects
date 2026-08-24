# cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-24T19:06:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0

**hypothesis**: Same as scratch-s0 (from-scratch V7 certfreeze ladder at control.hz=100, obs.history_frames=64 for a 100Hz-native 640ms context, fresh init BY DESIGN): the prior attempt never trained a single PPO step -- it died to 24x silent SIGBUS from landing on train-2, a legacy-64M-/dev/shm pod that never got the 08-10 dshm-4Gi fix (confirmed via a concurrent cycle's fleet audit: train-0/2/3/4 legacy, train-1/5/6/7/8/9/10/11 fixed at 4.0G). Same recipe, same n-envs 3072, pinned to train-1 (verified 4.0G shm + CUDA-capable + free) instead. Prediction-if-true: the V7 ladder climbs from b0 with reward and frontier promotions rising together at 100 Hz. Prediction-if-false: b0 never clears with flat reward at adequate budget -- the recipe/reward is not learnable at this rate under the current step cap. Strongest alternative: learns b0 but slower than 25 Hz equivalents purely from the 4x sim-seconds deficit at 100Hz -- reads as continuation candidate per 08-21, not failure.

**gate**: PASS: frontier promotions past b0 with reward/eval AGREEMENT at 100 Hz AND 60s randomized joygate at 100 Hz: falls<=2/48, no over_current cluster, directions followed on side/rear/turn/reversal stress, slip <=~2.9/m, video shows all six feet cycling. PARTIAL: genuinely learning (b0+ promotions, improving rung evals) but short of the 25 Hz lineage at budget end -- continuation candidate (expected: 1/4 sim-seconds). FAIL: b0 never promotes with reward AND rung metrics flat at adequate budget, or reward rises while rungs stay flat (100 Hz reward/eval mismatch -- audit before any seed sweep).

