# cw-walk-encodernoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T22:09:58+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: lo3wwjr5

**hypothesis**: OPERATOR WISHLIST 13c untested axis (encoder quantization): the STS3215's 12-bit encoder gives ~0.09deg LSB noise on every joint reading, always present on hardware but trained at 0 in the champion. ISOLATED axis: dr-scale 0.0 with ONLY dr.encoder_noise_deg=0.5 (~5.5x the real LSB, a stress margin since real noise is tiny but we want headroom). Plain: does per-step joint-angle jitter this small perturb the gait at all? Prediction-if-true: gait holds across the noise (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - noise-free by construction, likely closes as NO-EFFECT like torquedroop. Prediction-if-false: jitter degrades foot placement precision (stumbles, slip increase). Strongest alternative: champion already tolerates this free at this magnitude - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting (c59 rule).

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.encoder_noise_deg=0.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

