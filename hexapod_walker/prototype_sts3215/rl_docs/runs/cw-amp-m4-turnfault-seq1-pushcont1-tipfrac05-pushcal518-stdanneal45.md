# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-stdanneal45

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T14:12:18+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: 4wd90gkv

**hypothesis**: Plain English: this family's walk drags every loaded foot ~2x farther per stance than the joystick champion (11.49mm median vs 5.5mm, same command/geometry), and every reward-side lever is now measured useless (additive 6x/12x, full income gate, swing income — swing1 FAIL: median WORSENED to 13.54mm, only the tail compressed). The one big mechanical difference left: this family trains at a FIXED action-noise std 0.135 (--warm-log-std-override -2, no anneal) while the 5.5mm-stance champion stotight45s13 annealed to log-std -4.5, and the joystick track MEASURED (08-22 pd8 dig-in) that the 0.135 noise floor is exactly what makes honest stances drag — the learned mean gait is shaped by the noise it trains under. This arm changes ONE lever: --log-std-final=-4.5 (std 0.135 -> 0.011 over the run) on the unchanged pushcal518 recipe. Prediction-if-true: slipdist probe stance median drops from 11.49 toward <=9mm and m5 walk det slip med reaches <=3.5 with falls/gait/tips unchanged. Prediction-if-false: median unmoved — noise floor is not the amp slip mechanism. Strongest alternative: the slip level is DEMO-ANCHORED — the AMP style term holds slip near the teacher demos' own slip (noamp1 showed style is the slip floor-holder), so no policy-side noise change moves it; if so the next mechanism is a cleaner motion library (CPG contextual winner gait, slip 0.56-0.63/m, as teacher v4).

**gate**: eval_amp_m5 walk+yaw own cfg + slipdist probe rerun (hazard-free own-cfg, seed 0, 6 eps). PASS = 0/12 raw falls AND walk det slip med <=3.5 AND walk gait_valid 12/12 AND tips within 0.25 band AND probe stance median <=9mm. PARTIAL = probe median drops >=2.5mm vs parent's 11.49 (mechanism real at stance level) but m5 slip misses 3.5 — dose next (-5.0 rung per the joystick ladder). FAIL = probe median unmoved (>=10.5) AND slip unmoved (+-0.15 of 3.67) — noise-floor mechanism refuted for amp; remaining fork = demo-anchor mechanism (teacher v4 from CPG winner) vs the 3.5-bar ruling amendment (q_20260823T0130Z), with pricing/income/demos/gait-income/noise ALL then measured refuted.

