# cw-walk-encodernoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T22:09:58+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: lo3wwjr5

**hardware_ready**: False

**hypothesis**: OPERATOR WISHLIST 13c untested axis (encoder quantization): the STS3215's 12-bit encoder gives ~0.09deg LSB noise on every joint reading, always present on hardware but trained at 0 in the champion. ISOLATED axis: dr-scale 0.0 with ONLY dr.encoder_noise_deg=0.5 (~5.5x the real LSB, a stress margin since real noise is tiny but we want headroom). Plain: does per-step joint-angle jitter this small perturb the gait at all? Prediction-if-true: gait holds across the noise (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - noise-free by construction, likely closes as NO-EFFECT like torquedroop. Prediction-if-false: jitter degrades foot placement precision (stumbles, slip increase). Strongest alternative: champion already tolerates this free at this magnitude - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting (c59 rule).

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.encoder_noise_deg=0.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: NO-EFFECT (letter-pass only): own-cfg DR0 + dr.encoder_noise_deg=0.5 (~5.5x real LSB) det med fwd 1.36m>=1.2 gate, gv 12/12, 0 term; DR0 nominal retention clean (det gv 6/6, slip/m 0.97<=1.24). But per the c59 rule, parent champion longdist-r2 evaluated under the IDENTICAL encoder-noise spread matches it draw-for-draw: same 3/6 det craters (episodes 3-5, both shuffle to ~0.6-1.4m at slip 1.2-3.9), similar/better median (parent det fwd 1.49m vs this checkpoints own 1.36m). The champion already tolerates this magnitude of per-step joint-angle jitter for free -- exposure training on it added nothing measurable. Frames watched (det ep0 clean gait, det ep4 crater): six legs present and cycling throughout, no flag leg/collapse/drag, just a slow paddle-shuffle on the hard draws (known champion limitation, not an encoder-noise-specific failure). Joins zerobias3/gainvar/imubias3/torquedroop/cmddrop/legmass25/stiffvar/imumount10 in the calibration/sensor-noise NO-EFFECT class (now 9-for-9 unfixable-by-naive-DR-exposure on this ladder). No requeue; no DR0.5 compose (nothing to compose).

