# cw-arch-hist16

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:38:20+00:00

**pod**: hexapod-mjx-train-10

**steps**: 40000000

**parent**: none

**hypothesis**: TEMPORAL-ARCH line rung 1 (operator directive 08-10: keep 1-2 pods on architectures capturing more past states; line empty 2+ cycles). obs.history_frames 8->16 (~640ms at 25Hz) on the driving-champion config (DR0.5 + abrupt-flip resample). Warm start impossible across the obs-width change -> from-scratch rules (std 1.0, ent 0.01). External review section 8: temporal history = online system identification, ranked above bigger MLPs; operator expects it to help complicated movements (rise/sit, flips). If-true: joystick gate passes and det prog/slip land within champion band despite from-scratch in 40M — 16-frame history at worst matches 8 with headroom for the unified line. If-false (no gait at all): from-scratch at DR0.5+abrupt-resample cannot bootstrap — next rung runs an 8-frame from-scratch CONTROL or lowers DR before blaming history16. If-false (gait but worse than champion beyond noise): longer history hurts at this capacity — rung 3 (256x256) tests capacity vs memory. ASSUMPTION (operator to review): rung 1 launched at full driving config per WISHLIST -0.5 as written; bootstrap risk accepted and pre-registered rather than idling the line.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges, verdict is 'bootstrap failure, history16 untested' NOT a history16 FAIL; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

