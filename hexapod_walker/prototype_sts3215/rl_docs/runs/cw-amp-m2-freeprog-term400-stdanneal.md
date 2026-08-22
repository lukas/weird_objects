# cw-amp-m2-freeprog-term400-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T14:52:48+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-noamp

**wandb_id**: ebazqv3o

**hypothesis**: Plain English: does making the robot's random leg-jitter calm down over training (not just pricing death out) let it stop thrashing in place and start actually walking? term400-noamp (0 terminations, suicide fixed) still failed the walk gate by shuffling in place: env/reward_walk_freeprog_pen (the cross-track/backward charge) sat flat ~-1.4 to -1.8/tick with NO improvement over the full 2M steps at CONSTANT policy std=0.368 (--log-std-init=-1.0 default, never annealed) -- the exact same 'noisy exploration never organizes' fingerprint the joystick track root-caused and fixed for phasedir9 via a log-std anneal. Single change vs cw-amp-m2-freeprog-term400-noamp: add --log-std-final=-2.0 --log-std-anneal-frac=0.5 (std 0.368->0.135 linearly over the first 1M of 2M steps, matching the joystick track's mechanism, ported to a from-scratch run instead of a warm start). Prediction-if-true: freeprog_pen improves over training and median fwd travel rises toward the 0.10m bar. Prediction-if-false: shuffle persists at low std too (the flailing is priced-in by the reward shape, not exploration noise) -- points at a genuine reward-shape defect (freeprog's cross-track charge with no net-displacement credit) needing a direct patch, not a training-lever fix.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg), judged RELATIVE to cw-amp-m2-freeprog-term400-noamp at matched budget/config: PASS = median fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND zero/rare terminations held AND video shows six legs cycling with net displacement (not in-place shuffling). Secondary read regardless of PASS/FAIL: does env/reward_walk_freeprog_pen actually improve over the 4 quarters vs noamp's flat trace -- decides whether std-anneal is a viable lever for this reward family before any reward-shape patch.

**verdict**: Prediction-if-false CONFIRMED: annealing std 0.368->0.135 (train/std tracks exactly) did NOT fix the shuffle-in-place problem, it made the gait MORE regular while staying just as stationary. Median fwd travel collapsed to 0.005m det / 0.021m sto (worse than noamp's 0.026/0.032), but gait_valid is now a clean 6/6 det AND 6/6 sto (vs noamp's 3-5/6) with tight low-variance slip (10.3-11.5/m det) -- a textbook-regular six-leg marching-in-place gait, zero terminations. env/reward_walk_freeprog_pen still flat (-1.77->-1.18->-1.09->-1.10 across quarters, same plateau as noamp) and optimization/reward_per_tick_ema is WORSE than noamp's own plateau (-3.29 vs -2.84) -- the tighter policy just committed harder to the stationary local optimum. Root cause narrowed: this is a REWARD-SHAPE defect in walk_freeprog_score (rewards along-command velocity/no cross-track without a net-displacement floor), not an exploration-noise problem -- std-anneal lever CLOSED for this failure mode (mirrors the joystick track's own init-basin-flatness finding: a flat reward surface doesn't move by annealing noise alone).

