# cw-arch-hist16-r7

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T04:25:36+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-r6-rr1

**hypothesis**: TEMPORAL-ARCH rung 1 (history_frames=16, from-scratch), 9th attempt — first with the ROOT CAUSE of the 8x 0-step deaths fixed. Deep dig-in (08-10) proved the 'launch-collision EOFError' on this spec was actually /dev/shm exhaustion: train pods have the 64M k8s default, a 4096-env sharded layout maps ~58M, and hist16 doubles the obs block to >64M, so every worker died of SIGBUS on first page touch (all 24 workers exitcode -7, caught by the new faulthandler+exitcode diagnostics, snapshot c4f3625/bcf46be). This respec drops n-envs 4096->3072 (~50M layout, fits 64M with margin; rollout still 49k/update) — the one deliberate deviation from the r6 spec, noted per one-variable rules; pods also now GC orphaned shm segments at startup. If-true: run BOOTS and trains, giving the first real hist16 science (gate below). If-false-boot: SIGBUS persists at 3072 -> size model wrong, wait for dshm-volume pod recreation (manifests already patched). Science gate unchanged from r6.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges, verdict is 'bootstrap failure, history16 untested' NOT a history16 FAIL; frames watched det

**refused_reason**: hexapod-mjx-train-2 already runs cw-quad-hold2 — GPU pods host exactly one run; pick a free GPU pod.

