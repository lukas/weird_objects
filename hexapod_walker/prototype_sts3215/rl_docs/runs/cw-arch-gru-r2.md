# cw-arch-gru-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T13:15:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-gru-r1

**wandb_id**: 7jgxvq9o

**hardware_ready**: no

**hypothesis**: GRU rung continuation: r1 (2M from scratch) showed every skill emerging in ONE recurrent policy (hold 2.5deg, crouch-rise 2/2, tipped recovery 2/2, reward still climbing) but walk barely tracking (vel err 0.074). r2 continues from the r1 checkpoint with a walk-heavier diet (0.55 -> 0.70) to feed the data-hungry skill while rise/lower/hold ride along. If walk err drops materially with stance skills retained -> the recipe scales and a hardening-length run is licensed; if walk stays flat -> recipe rethink (BPTT window / lr) before spending more.

**gate**: PASS if final det eval walk err < 0.065 m/s AND trending down vs r1 (0.074), with hold <= 3.5deg and at least one rise completion retained. FAIL otherwise.

**verdict**: FAIL (known exploit, no forensics): identical leg-sacrifice/paddle-jitter fingerprint as parent cw-arch-gru-r1 despite the walk-heavier diet (0.55->0.70). Gate(DR0) det walk 0/6 gait_valid, legs sacrificed, frozen 0.004 m/s; sto walk 6/6 gait_valid but prog_ratio ~0, slip/m 17.8 (no-progress paddle). Own-cfg(DR0.5) same shape. Return climbed, task didn't -- reward-shortcut pattern. GRU rung stays FROZEN off the blocker list; no step-count/diet relaunch without a recipe change.

