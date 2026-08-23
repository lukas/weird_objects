# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T06:43:35+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: t40gwlco

**hypothesis**: Plain English: is the tipfrac05 M5-suite pass a RECIPE or a seed-lottery win? Exact same config (50% dedicated turn-episode exposure on the composed turn+push+fault stack, overshoot pricing keys ON, same pre-cheat turnfault-seq1 init, 2M) -- only the RNG seed changes (7->23). Joystick-track precedent (stotight45 4/4 seeds): a first gate pass counts as a recipe only after it reproduces across seeds. Prediction-if-true: tips land <=0.20-0.25 with safety floors held. Prediction-if-false: tips park near 0.25+ = tipfrac05 was a lucky basin, recipe claim downgraded. Strongest alternative: passes the 0.25 band but misses the 0.20 m5 bar (partial reproduction).

**gate**: PASS = eval_amp_m5 yaw tips <=0.20-0.25 both signs (m5-bar-clean at <=0.20) AND safety floors held (own-cfg DR-0 gait_valid >=9/12, push/fault sections PASS). Grid read across s2/s3 + original: 3/3 in-band = recipe confirmed; 1/3 = lottery, question moves to budget/pricing not more seeds. Judge tips from the m5 yaw section (hazards zeroed), never DR-0 panel medians (tip-episode contamination gotcha).

