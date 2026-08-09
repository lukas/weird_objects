# cw-walk-backforth

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:51:53+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: xi5bkjlt

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 4: back and forth. Full-circle heading commands (+-180deg) with resampling and 15% stops — walk out, stop, walk BACK. Backward walking is the new behavior; the champion never saw heading beyond +-45deg. If-true: reverse tracking with gait_valid; if-false: backward gait fails (leg geometry asymmetry) and backward needs its own reward shaping.

**gate**: DR0 det+sto 6/6 incl reverse commands: gait_valid, zero terminations, reverse tracking err <= 2x forward err

**verdict**: FAIL / refuted on if-false. Rear-hemisphere (±180°) commands do not transport: 0/12 success, det prog med 0.59, slip/m med 3.9-4.5, sacrificed leg [3] both passes; frames show BOTH front legs held airborne while the body creeps on four legs. Reward declined over training. Matches steer-explore/fdiag: backward walking blocked on the same contact-pricing root + own shaping; operator ruling 4 (rear hemisphere deferred) stands. No requeue.

