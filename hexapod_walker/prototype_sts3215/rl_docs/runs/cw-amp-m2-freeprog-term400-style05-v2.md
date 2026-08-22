# cw-amp-m2-freeprog-term400-style05-v2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T16:07:15+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-style05

**wandb_id**: s8lnr8oo

**hypothesis**: Plain English: re-run the Wave-1 style-vs-control fork with a motion library that actually contains the verified teacher's gait -- every AMP style arm so far trained against the frame-corrupted teacher_v1 (fb_20260822T145428 audit: raw absolute-tibia TripodGait fed unconverted, knee off up to 15.7deg vs the sim_gait_compat truth), so the discriminator has been pulling the policy toward a subtly WRONG gait and the 'AMP gave cleaner gait_valid, not real travel' fork resolution is contaminated. Single change vs cw-amp-m2-freeprog-term400-style05 (FAIL, statue): --amp-motion-lib=teacher_v2.npz (45/45 clips, slip in teacher band, built via the sim_gait_compat boundary). Everything else byte-identical: SLIPWALK pricing, term_penalty=400, style/task 0.5/0.5, from scratch, 2M discovery.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg), judged RELATIVE to cw-amp-m2-freeprog-term400-noamp AND -style05 at matched budget: PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six-leg cycling with net displacement. Secondary reads either way: amp/d_real-d_fake stays unsaturated, style_rew trend, and whether freeprog_pen leaves the -1.5/tick statue plateau for the first time in this family. Prediction-if-true: the clean prior gives the statue an escape gradient (teacher-like leg cycling earns style income immediately) and travel appears. Prediction-if-false: statue persists identically -> the corrupted-lib caveat is retired as an explanation and the 0.5 style weight being priced out (max 0.5/tick vs -1.5/tick charges) becomes the live suspect, read jointly with the -stylew2-v2 dose twin launched this cycle.

