# cw-dep-bcgait4-phasedir9-stotight50-seed29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T18:48:56+00:00

**pod**: hexapod-mjx-train-3

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50

**wandb_id**: 8n50emls

**hypothesis**: Plain English: the -5.0 noise floor widened seed17's gate margins but HURT seed13 (slip 2.407->2.63), so this arm tests transfer on seed29, the passer with the thinnest own-DR sto slip margin (2.736 vs cap 2.9 at -4.5) -- the other named hardening gap. Single change vs the stotight50 PASS: seed 17 -> 29 (identical -5.0 recipe, fresh reinit per the lineage rule). Prediction-if-true: joygate pass=true with combined slip below seed29@-4.5's 2.704 and own-DR slip pulled off the 2.736 edge (ladder transfers; own-DR hardening gap closes). Prediction-if-false: margins at or worse than seed29@-4.5 (together with seed13's regression this establishes the ladder as seed17-only; dose hardening is closed and champion selection rests on the -4.5 readings).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined slip < seed29@-4.5's 2.704 AND own-DR slip < 2.736 (gap closed). INFORMATIVE = pass=true but slip within noise of or worse than seed29@-4.5 (no transfer; dose hardening closed for seed29). FAIL = pass=false (dose flips the basin).

**verdict**: The -5.0 dose does NOT transfer to seed29 — its slip-shaped hardening gap got worse, not better. 60s joystick DONE-gate evaluator still passes (0 falls/48, gait 48/48) but combined slip 2.748 vs the pre-registered PASS bar <2.704 (seed29@-4.5), and own-DR slip 2.986 vs bar <2.736 — the own-DR pass alone is over the 2.9 teacher-band cap. Dir 39.3 is also within 0.7 deg of the 40 allowance. 15s det gate weakest of the -5.0 batch (prog 0.63/slip 2.31 DR-0; own-DR det slip med 2.96 with 6/6 over-cap episodes). Contact sheet clean (six legs cycling, no sacrifice) — this is margin erosion, not pathology. Reward rose and converged (quarters -132/76/473/462) so this is an honest basin answer, not undertraining: per the pre-registered INFORMATIVE branch, dose hardening is CLOSED for seed29. Combined with seed13 (worse at -5.0) and seed23 (better at -5.0): the ladder dose-response is seed-basin-specific and does not track gap type (seed23's dir gap closed, seed29's slip gap widened). Best-per-seed stands at seed13@-4.5 (champion), seed17@-5.5, seed23@-5.0, seed29@-4.5. Next: per-seed dose search is done — no further blanket-dose arms; remaining joystick work is the champion consolidation and the held-out panel per track STATUS.

