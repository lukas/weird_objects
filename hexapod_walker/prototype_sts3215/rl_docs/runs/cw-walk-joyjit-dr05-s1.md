# cw-walk-joyjit-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T20:32:26+00:00

**pod**: hexapod-mjx-train-2

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05

**wandb_id**: 1602jw5x

**hardware_ready**: no

**hypothesis**: Seed-1 replication of the joystick-gate PASS (joyjit-dr05-c1). If-true: gate pass is seed-robust - promote the config to driving champion. If-false: c1 was a lucky seed - promotion blocked, panel verdict goes to the plan.

**gate**: eval_drive own-cfg DR0.2: 0 in-envelope falls (panel + 3 flip-stress eps); generic own-cfg gate gv 12/12, 0 term

**verdict**: PASS. Seed-1 replication of the joyjit-dr05-c1 promotion-panel PASS (ruling-7): eval_drive JOYSTICK GATE @DR0.2 (heading45) 0 in-envelope falls (panel + 3 flip-stress eps, trk_err 0.025-0.056); own-cfg DR0.5 harness gv 12/12, 0 term, prog med 1.00 det/0.98 sto, slip/m med 1.46/1.40 (matches c1's 1.38/1.44 band). Gate pass is seed-robust, not seed luck. NOTE: joyjit-dr05 has since been superseded as best driving candidate by joylat25/joyhead90-lat25 (wider envelope + latency), so this seed-confirm settles the promotion-panel question for the record but does not itself change the current best-candidate pointer. Frames det: six legs cycling, level, no flag leg, paddle foot-slide persists. Not hardware-ready.

