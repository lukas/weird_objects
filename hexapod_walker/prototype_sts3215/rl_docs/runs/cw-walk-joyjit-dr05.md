# cw-walk-joyjit-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-09T15:42:23+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-wander-dr05

**wandb_id**: m7kthv1j

**hypothesis**: OPERATOR BINDING TARGET (joystick operability) x robustness: wander-dr05 holds steering at DR 0.5 but only with GENTLE fixed 5s/1s-blend command changes; a joystick sends irregular instant flips. One move off the wander-dr05 checkpoint: adopt the randomized abrupt-resample package (interval 1.5s +-60% jitter, blend 0.1-1.0s, 20% stops — same package as joystick45, which tests it at DR0 off wander30) while KEEPING DR 0.5. Controlled pair with joystick45: this arm answers whether flip-hardening works under physics uncertainty. If-true: JOYSTICK GATE (eval_drive, DR 0.2) passes with zero in-envelope falls and own-cfg DR0.5 harness holds gv 12/12, 0 term — transition hardening and DR compose. If-false: abrupt resampling under DR degrades the base gait (falls or prog craters) — hardening needs a curriculum, not joint exposure.

**gate**: JOYSTICK GATE: python3 -m rl_move.sim.eval_drive <ckpt> --dr-scale 0.2 with own cfg — ZERO in-envelope falls across panel + flip stress; plus own-cfg DR0.5 harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.85; frames watched det

**verdict**: No verdict on hypothesis - REBALANCED at ~2M/20M: node g142d86 host-wide starved (loadavg 216/128, fps 2.1-2.9k vs 5k floor, GPU 26% util; g131eec carries 3 trainers at load 51, so likely a foreign tenant). Checkpoint pulled (md5 bbde2450), relaunched as cw-walk-joyjit-dr05-c1 on idle node g129004.

