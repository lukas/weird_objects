# cw-walk-lowgait-fricvar-s1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T04:05:01+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-lowgait-fricvar

**wandb_id**: hbj84lj2

**hardware_ready**: False

**hypothesis**: Seed twin of cw-walk-lowgait-fricvar (PASS this cycle, dig-in verified): friction 0.4-1.6x composes onto the -20mm crouch, and the pinned-0.4 dead-skate draw was proven INHERITED from the parent (not a fricvar defect). Ruling-7 seed panel: if the compose is a recipe not seed-luck, seed 1 reproduces it. If-true: own-cfg det+sto gv 12/12, 0 term, mean end-height err <=8mm; DR0 retention det gv 6/6 slip/m <=1.24 — same band as seed 0. If-false: seed 1 erodes crouch height tracking or slick-end medians fall below the upright fricvar band (prog med <0.8).

**gate**: Own-cfg (walk_height_off_mm=-20 + dr.friction_scale=0.4,1.6) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err <=8mm; DR0 retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: PASS: seed-1 twin of cw-walk-lowgait-fricvar (parent PASS) confirms crouch(-20mm) x friction-var(0.4-1.6x) is a recipe not seed luck -- own-cfg det gv 6/6 (0 term, height err mean 3.8mm det/3.4mm sto, both <=8mm gate), sto gv 6/6, det/4 craters to the lineage's known fixed-draw march-in-place stall (prog -0.04, slip 14.9, frame-checked: level body, all 6 legs still cycling, no flag-leg) matching the seed-0 fricvar's own documented slick-draw caveat, not a new defect; DR0 nominal-crouch retention det gv 6/6, slip/m 1.08 (<=1.24 gate).

