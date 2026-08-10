# cw-arch-hist16-r7

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T04:48:51+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-r6-rr1

**hardware_ready**: False

**hypothesis**: TEMPORAL-ARCH rung 1 (history_frames=16, from-scratch), 9th attempt — first with the ROOT CAUSE of the 8x 0-step deaths fixed. Deep dig-in (08-10) proved the 'launch-collision EOFError' on this spec was actually /dev/shm exhaustion: train pods have the 64M k8s default, a 4096-env sharded layout maps ~58M, and hist16 doubles the obs block to >64M, so every worker died of SIGBUS on first page touch (all 24 workers exitcode -7, caught by the new faulthandler+exitcode diagnostics, snapshot c4f3625/bcf46be). This respec drops n-envs 4096->3072 (~50M layout, fits 64M with margin; rollout still 49k/update) — the one deliberate deviation from the r6 spec, noted per one-variable rules; pods also now GC orphaned shm segments at startup. If-true: run BOOTS and trains, giving the first real hist16 science (gate below). If-false-boot: SIGBUS persists at 3072 -> size model wrong, wait for dshm-volume pod recreation (manifests already patched). Science gate unchanged from r6.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges, verdict is 'bootstrap failure, history16 untested' NOT a history16 FAIL; frames watched det

**verdict**: PASS -- first TEMPORAL-ARCH hist16 (history_frames=16, 3072-env, from-scratch) run to actually boot and train after the /dev/shm root-cause fix; ends the 8x 0-step death streak with real science. Own-cfg flat(DR0) det/sto gv 6/6, terms 0, prog med 1.17/1.02 (>=0.85 gate). Own-cfg DR0.5 (its trained DR) det/sto also gv 6/6, terms 0, prog med 1.08/1.09 -- clears the gate cleanly, no sacrificed legs, no falls, slip 1.3-1.6 (elevated vs contract-line champions but not gated here). JOYSTICK GATE (eval_drive DR0.2): 0 in-envelope falls across the full direction panel + flip-stress. Video: all six legs visibly cycling swing/stance every episode sampled (det 0/2/4), no flag leg, no dragging -- a real, if unremarkable, walking gait. Architecture question answered: history_frames=16 CAN learn to walk from scratch at 3072 envs; whether it beats the 8-frame baseline needs a head-to-head, not this run. Not hardware-ready as-is (does not use the dep-line deployment-exact obs contract; this is an architecture-line checkpoint).

