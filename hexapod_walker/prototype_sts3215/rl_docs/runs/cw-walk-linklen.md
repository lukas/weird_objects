# cw-walk-linklen

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T20:09:16+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 4p6affkv

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 13b geometry axis: LINK-LENGTH ASSEMBLY ERROR (print/CAD + per-leg assembly spread; nearest READY proxy for foot-geometry perturbation). ISOLATED: dr-scale 0.0 with ONLY dr.link_len_leg_pct=0.03 (2.5x the 0.012 full-DR default - beyond-envelope test, same pattern as comshift30). Champion baseline MEASURED FIRST per c59 lesson (c62, logs/ckpt_eval/champ_baseline_linklen): gv 12/12, 0 term, but hardest det draws collapse to fwd 0.61/0.66m at slip 3.3-3.9 (nominal 1.57m/0.96) - not covered free. Plain: walk the same even if every leg is built a few percent off. Prediction-if-true: exposure fixes the tail - det worst-2 draws >=1.0m, det med >=1.2m, gv 12/12, 0 term, DR0 retention in champion band. Prediction-if-false: >2% per-leg geometry error breaks the paddle's fine timing (tail stays <=0.8m) - record a buildable-tolerance envelope note instead. Strongest alternative: median holds while tail persists (partially trainable) - judge per-episode vs named baseline. Parent: cw-walk-longdist-r2.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.link_len_leg_pct=0.03, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd >=1.2m AND det worst-2 draws beat champ baseline 0.61/0.66m outside noise (>=1.0m); plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: FAIL (if-false confirmed): link-length exposure does NOT fix the geometry tail. Own-cfg (dr.link_len_leg_pct=0.03) gv 12/12, 0 term, det med fwd 1.35m (>=1.2 gate met) but worst-2 det draws 0.70/0.71m vs gate >=1.0m - vs champ baseline 0.61/0.66m the delta is inside noise. Medians match the untrained champion (1.35 vs 1.43m det) = 20M steps bought nothing. DR0 nominal retention CLEAN (6/6 gv, slip 0.96, fwd 1.57m = champion band, no forgetting). Frames on worst draw: level, six legs cycling, no flag leg, no falls - transport just halves via slip. ENVELOPE NOTE: build the real legs to <~2% per-leg link-length error; at 3% the hardest draws halve transport (graceful shuffle, no falls). >2% geometry error breaks the paddle's fine timing and exposure can't retime it - same trainability boundary as torque/servo-sag classes. No requeue; no dr05 compose.

