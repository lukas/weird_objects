# cw-walk-joyfric-payload-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASSED

**created**: 2026-08-10T01:34:45+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-joyfric-payload

**wandb_id**: 1bi25msp

**hardware_ready**: False

**hypothesis**: Ruling-7 seed twin of cw-walk-joyfric-payload (PASS this cycle: mass-DR payload compose does NOT erode the driving line's retention beyond its own parent joyfric's baseline -- resolves the c61 payload-dr05 lineage-specificity question). One variable: seed 0->1. If-true: seed1 reproduces the same pattern (own-cfg gv 6/6, JOYSTICK GATE 0 falls, DR0 retention within noise of joyfric's own 1.41/1.37 baseline) -- confirms the no-erosion finding is recipe-robust, not seed luck. If-false: seed1 shows real retention erosion (slip clearly above the joyfric baseline band, not just above the mis-calibrated 1.24 letter-cap) -- the seed0 result was luck and payload-DR-compose erosion is real but noisy.

**gate**: Own-cfg (DR0.5+lat+fric+mass1.0-1.4x) det+sto 6/6 @15s: gait_valid 6/6, 0 term, prog med>=0.75; JOYSTICK GATE @45deg 0 in-envelope falls; DR0 nominal retention gv 6/6, 0 term, slip/m within ~15% of joyfric's own retention band (1.41 det/1.37 sto), not the flat 1.24 cap; frames watched det.

**verdict**: PASS: seed-1 twin reproduces cw-walk-joyfric-payload cleanly. Own-cfg (DR0.5+lat0.5-2.5x+fric0.4-1.6x+mass1.0-1.4x) det gv 6/6 prog med 0.94, sto gv 6/6 prog med 0.99, 0 term either pass. JOYSTICK GATE @45deg: 0 in-envelope falls incl. flip-stress. DR0 nominal retention (walk-mode-only harness gate): det slip med 1.30 / sto 1.55, gv 6/6, 0 term -- within the same noise band as parent joyfric-paylods own retention (1.41/1.37) and this cycles seed0 payload PASS (1.46/1.51), confirming the no-erosion finding is seed-robust, not seed-0 luck. Frames (det, own-cfg + owndr + retention): six legs cycling, level body, no flag leg, no leg-through-floor; known foot-slide/paddling persists unchanged from parent. Not hardware-ready (paddling).

