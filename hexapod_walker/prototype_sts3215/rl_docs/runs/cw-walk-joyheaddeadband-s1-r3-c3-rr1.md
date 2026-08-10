# cw-walk-joyheaddeadband-s1-r3-c3-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T02:53:45+00:00

**pod**: hexapod-mjx-train-9

**steps**: 16000000

**parent**: cw-walk-joyheaddeadband-s1-r3

**wandb_id**: bajx2kcs

**hardware_ready**: False

**hypothesis**: REBALANCE continuation, 3rd attempt (c1 + c2 both lost launch-collision EOFError races during a heavy concurrent-launch window, gotcha 13b, 0 steps each -- no science result either time). Root cause unchanged: cw-walk-joyheaddeadband-s1-r3 was killed at ~3.8M/20M because node g142d86 was host-starved (loadavg ~245/128, fps declined 8961->5000). Same hypothesis as parent: ruling-7 seed-1 deadband twin, unchanged spec. If-true: same gate as parent. If-false: flag for dig-in per c67 precedent (isolated-spec failure, not fleet storm).

**gate**: own-cfg DR0 6+6 harness gv>=10/12, no sacrificed legs, det med fwd within champion band; JOYSTICK-style deadband/friction DR retention clean vs parent lineage

**verdict**: PASS -- seed-1 twin reproduces cw-walk-joyheaddeadband cleanly (2nd seed, closes the ruling-7 panel): JOYSTICK GATE @90deg 0 in-envelope falls (fwd/diag/stop-go panel + 3 flip-stress eps, trk_err 0.024-0.051); own-cfg (DR0.5+lat+friction0.4-1.6x+deadband1-3x) det+sto gv 6/6, 0 term, prog med 0.92/0.90 (>=0.80 gate); DR0 flat retention gv 6/6, 0 term, prog med 0.92/0.93, slip 1.50/1.43 -- matches parent band (0.90/0.93). Frames: six legs cycling every episode, no flag leg, known paddle foot-slide persists. Recipe seed-robust 2/2.

