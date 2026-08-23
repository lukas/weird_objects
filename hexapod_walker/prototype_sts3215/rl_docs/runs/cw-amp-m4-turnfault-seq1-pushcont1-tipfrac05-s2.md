# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T06:43:35+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: t40gwlco

**hypothesis**: Plain English: is the tipfrac05 M5-suite pass a RECIPE or a seed-lottery win? Exact same config (50% dedicated turn-episode exposure on the composed turn+push+fault stack, overshoot pricing keys ON, same pre-cheat turnfault-seq1 init, 2M) -- only the RNG seed changes (7->23). Joystick-track precedent (stotight45 4/4 seeds): a first gate pass counts as a recipe only after it reproduces across seeds. Prediction-if-true: tips land <=0.20-0.25 with safety floors held. Prediction-if-false: tips park near 0.25+ = tipfrac05 was a lucky basin, recipe claim downgraded. Strongest alternative: passes the 0.25 band but misses the 0.20 m5 bar (partial reproduction).

**gate**: PASS = eval_amp_m5 yaw tips <=0.20-0.25 both signs (m5-bar-clean at <=0.20) AND safety floors held (own-cfg DR-0 gait_valid >=9/12, push/fault sections PASS). Grid read across s2/s3 + original: 3/3 in-band = recipe confirmed; 1/3 = lottery, question moves to budget/pricing not more seeds. Judge tips from the m5 yaw section (hazards zeroed), never DR-0 panel medians (tip-episode contamination gotcha).

**verdict**: Result: recipe reproduces on a second seed (band-clean, not bar-clean). Evidence: eval_amp_m5 tip-left/right err 0.207/0.228 -- inside the run's own PASS band (<=0.20-0.25 both signs) though ~0.01-0.03 over the stricter 'm5-bar-clean' 0.20 line the seed=7 original hit (0.162/0.184). own-cfg DR-0 gait_valid 12/12 (6/6 det + 6/6 sto, well over the >=9/12 floor) with 2 isolated terminations (1 det tilt_roll, 1 sto tilt_pitch) that did not break gait_valid or cost sacrificed legs. m5 push section PASS clean (0/12 terms, gait_valid 12/12); m5 fault section PASS at the bar (gait_valid 10/12, one extra carried leg vs the original's pattern). m5 walk section itself misses its own slip bar (det_slip_med 4.167 vs 3.5) but that axis is not part of THIS run's own pre-registered PASS criteria (tips + safety floor + push/fault sections only). Video: clean six-leg cycling, no dragging, no visible falls. Why: second of three planned seed twins (seed 7->23) lands in-band, same direction and rough magnitude as the seed=7 original -- the turn_in_place_frac=0.5 curriculum lever is looking like a reproducible recipe, not a one-seed lottery win, though not yet bar-clean on this seed. Next: hold for the sibling -s3/-seed13 triage (owned by a concurrent cycle) to close the recipe-vs-lottery read at n=3; do not re-launch more seeds on this exact recipe from this cycle.

