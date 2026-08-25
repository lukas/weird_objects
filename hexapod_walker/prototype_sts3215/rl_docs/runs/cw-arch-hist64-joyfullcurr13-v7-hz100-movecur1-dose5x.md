# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-dose5x

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T08:48:24+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1

**wandb_id**: ff5xgy92

**hypothesis**: Plain English: all 3 movecur1 arms at k_walk_move_current=2.0 (MLP-alone, MLP+gait_gate, transformer) FAILed identically, every one pinned at cur_max_a=2.64A (the safety-trip edge) regardless of charge -- before closing the whole current-dwell-charge MECHANISM CLASS (not just this dose) as a fix for the mesh-family over_current/leg-sacrifice exploit, this is a cheap 2M bound-the-mechanism canary at 5x the dose (k=10.0, everything else byte-identical to the original movecur1 2M base) to see whether the Imax=2.64A signature can be moved AT ALL by pricing pressure, or whether it is structurally unreachable by this reward term (e.g. actuator saturation dominating the pricing gradient) regardless of magnitude.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. PASS: cur_max_a measurably below 2.64A (not pinned at the safety-trip edge) at 2M, even if walking itself is still generic-early-valley -- proves the mechanism CAN bite at higher dose, worth a full acquisition-budget arm at this dose. FAIL: cur_max_a still pinned ~2.64A / over_current still fires every episode -- the mechanism is dose-insensitive, not just underdosed at k=2.0, and the current-dwell-charge class is fully closed regardless of magnitude (structural, not a pricing-magnitude problem) -- do not iterate this dose axis further, the standwalk-style teacher-distillation route becomes the only named next lever for this exploit.

