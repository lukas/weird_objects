# cw-quad-hold1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T03:02:05+00:00

**pod**: hexapod-mjx-train-3

**steps**: 10000000

**parent**: cw-walk-longdist-r2

**hypothesis**: Quad-hold goal mode (code c086a22, feasibility GO c57, probe clean: scratch policy already lifts fronts 95% off / 55mm): warm start from walk champion (obs UNCHANGED, fronts 0+5 commanded via two-hot goal one-hot), 50% quad mix learns lift-fronts-hold-level-on-four while 40% walk mix guards retention. Probe showed planted_frac 0.65 from scratch -> k_quad_plant raised to 1.0. If false: planted_frac stays < 0.9 (tips into tripod) -> route a stance_contact-style term for the support four; or walk erodes -> ladder the mix.

**gate**: eval quad episodes: fronts_off >= 0.9 AND mean front clear >= 20 mm AND planted_frac >= 0.95 over final 10 s in >= 10/12 eps AND |roll|,|pitch| <= 4 deg AND 0 terminations; walk retention det med forward per parent band, slip <= 1.25

**failed_reason**: run never appeared as 'running' in W&B within 240s

