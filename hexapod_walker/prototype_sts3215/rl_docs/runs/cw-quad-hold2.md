# cw-quad-hold2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T04:24:19+00:00

**pod**: hexapod-mjx-train-2

**steps**: 10000000

**parent**: cw-walk-longdist-r2

**wandb_id**: itnbozx1

**hypothesis**: cw-quad-hold1-r2 (50/40/10 quad/walk/hold mix) learned a solid four-leg hold (survived_frac 1.0, video-clean, no falls) but eroded walk retention below the standard slip gate (det med slip/m 1.42 vs cap 1.25, plus a sto flag-leg skate). Laddering the mix down to quad=0.3/walk=0.6/hold=0.1 (same init, same reward coefficients, ONLY the mix ratio changes) tests whether less quad-training pressure recovers walk retention while still learning the trick. If true: walk det med slip/m <=1.25 (matching parent band) AND quad survived_frac stays ~1.0 with fronts_off/planted_frac/height_err still solid on video+eval telemetry. If false: slip stays >1.25 even at 30% quad pressure -> the erosion isn't dose-dependent on mix ratio, points to a reward/pricing conflict (quad reward terms fighting walk's anti-slip pricing) rather than raw training-time competition -- next lever would be an explicit anti-slip cost active during quad episodes, not further mix laddering.

**gate**: Own-cfg (mix quad=0.3/walk=0.6/hold=0.1) det+sto 6/6: walk-mode det med slip/m <=1.25, gait_valid 6/6, 0 term, fwd per parent band; quad-mode training eval survived_frac 1.0 across final 3 eval points, height_err_end_mm <=20mm, video (det, both early+late in training) shows level 4-leg stance with both fronts clearly lifted and no tipping

