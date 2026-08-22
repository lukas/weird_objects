# cw-amp-m2-freeprog-term400-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T14:51:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-noamp

**hypothesis**: Plain English: does making the robot's random leg-jitter calm down over training (not just pricing death out) let it stop thrashing in place and start actually walking? term400-noamp (0 terminations, suicide fixed) still failed the walk gate by shuffling in place: env/reward_walk_freeprog_pen (the cross-track/backward charge) sat flat ~-1.4 to -1.8/tick with NO improvement over the full 2M steps at CONSTANT policy std=0.368 (--log-std-init=-1.0 default, never annealed) -- the exact same 'noisy exploration never organizes' fingerprint the joystick track root-caused and fixed for phasedir9 via a log-std anneal. Single change vs cw-amp-m2-freeprog-term400-noamp: add --log-std-final=-2.0 --log-std-anneal-frac=0.5 (std 0.368->0.135 linearly over the first 1M of 2M steps, matching the joystick track's mechanism, ported to a from-scratch run instead of a warm start). Prediction-if-true: freeprog_pen improves over training and median fwd travel rises toward the 0.10m bar. Prediction-if-false: shuffle persists at low std too (the flailing is priced-in by the reward shape, not exploration noise) -- points at a genuine reward-shape defect (freeprog's cross-track charge with no net-displacement credit) needing a direct patch, not a training-lever fix.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg), judged RELATIVE to cw-amp-m2-freeprog-term400-noamp at matched budget/config: PASS = median fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND zero/rare terminations held AND video shows six legs cycling with net displacement (not in-place shuffling). Secondary read regardless of PASS/FAIL: does env/reward_walk_freeprog_pen actually improve over the 4 quarters vs noamp's flat trace -- decides whether std-anneal is a viable lever for this reward family before any reward-shape patch.

**refused_reason**: hexapod-mjx-train-0 code marker f1f235001b336737d97dd915d9f2ead0e55697b1-dirty != local HEAD f1f235001b336737d97dd915d9f2ead0e55697b1. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

