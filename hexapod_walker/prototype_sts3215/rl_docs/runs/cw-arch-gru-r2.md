# cw-arch-gru-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T13:15:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-gru-r1

**wandb_id**: 7jgxvq9o

**hypothesis**: GRU rung continuation: r1 (2M from scratch) showed every skill emerging in ONE recurrent policy (hold 2.5deg, crouch-rise 2/2, tipped recovery 2/2, reward still climbing) but walk barely tracking (vel err 0.074). r2 continues from the r1 checkpoint with a walk-heavier diet (0.55 -> 0.70) to feed the data-hungry skill while rise/lower/hold ride along. If walk err drops materially with stance skills retained -> the recipe scales and a hardening-length run is licensed; if walk stays flat -> recipe rethink (BPTT window / lr) before spending more.

**gate**: PASS if final det eval walk err < 0.065 m/s AND trending down vs r1 (0.074), with hold <= 3.5deg and at least one rise completion retained. FAIL otherwise.

