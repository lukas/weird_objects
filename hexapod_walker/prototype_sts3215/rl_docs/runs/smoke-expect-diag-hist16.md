# smoke-expect-diag-hist16

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T03:52:50+00:00

**pod**: hexapod-mjx-train-7

**steps**: 256

**hypothesis**: smoke: worker-death diagnostics patch (faulthandler in workers + named-worker-exitcode EOF report in parent) boots and trains 256 steps

**verdict**: Smoke did its job twice over: patched worker-death diagnostics worked on first use AND caught the fleet bug. All 24 workers exitcode -7 (SIGBUS) at first reset on train-7 — /dev/shm was 90% full (58M/64M) of segments LEAKED by earlier crashed runs; k8s default /dev/shm is 64M and a normal 4096-env sharded layout maps ~58M, so one crashed run poisons the pod and every later launch dies at 0 steps with the bare EOFError previously misread as gotcha-13b launch collisions. Fixes shipped: (1) faulthandler + per-worker exitcode report in mjx_sharded_vec_env (c4f3625), (2) startup GC of orphaned hexmjx-* shm segments (bcf46be, unit-tested: removes orphans, keeps live-mapped), (3) dshm 4Gi Memory emptyDir added to both pod manifests — apply by delete+recreate+bootstrap per pod WHEN IDLE (needed before any history_frames>=16 run at 4096 envs; hist16 layout >64M even clean). arch-hist16-r7 requeued at 3072 envs (~50M, fits).

