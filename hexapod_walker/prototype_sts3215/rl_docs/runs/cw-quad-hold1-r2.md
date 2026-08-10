# cw-quad-hold1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T03:18:46+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**parent**: cw-walk-longdist-r2

**wandb_id**: zf9qx38y

**hardware_ready**: False

**hypothesis**: Quad-hold goal mode (code c086a22, feasibility GO c57, probe clean: scratch policy already lifts fronts 95% off / 55mm): warm start from walk champion (obs UNCHANGED, fronts 0+5 commanded via two-hot goal one-hot), 50% quad mix learns lift-fronts-hold-level-on-four while 40% walk mix guards retention. Probe showed planted_frac 0.65 from scratch -> k_quad_plant raised to 1.0. If false: planted_frac stays < 0.9 (tips into tripod) -> route a stance_contact-style term for the support four; or walk erodes -> ladder the mix. (r2: rr1 died to /dev/shm leak, gotcha 13; shm cleaned)

**gate**: eval quad episodes: fronts_off >= 0.9 AND mean front clear >= 20 mm AND planted_frac >= 0.95 over final 10 s in >= 10/12 eps AND |roll|,|pitch| <= 4 deg AND 0 terminations; walk retention det med forward per parent band, slip <= 1.25

**verdict**: FAIL on the walk-retention leg of the compound gate (if-false branch of the hypothesis: 'walk erodes -> ladder the mix'). Quad-hold itself looks solid: training's own quad-mode eval survived_frac 1.0/1.0 across all 6 checkpoints, height_err_end_mm mostly <10mm, track_err_deg ~1deg, and W&B rollout videos (early 03:21 + late 03:35 in training) both show a clean, level four-leg stance with both front legs lifted clear and no tipping -- the party-trick mechanism composes fine. But own-cfg det walk-mode harness eval: med slip/m 1.42 (cap 1.25, matches the standard champion-walk gate threshold used campaign-wide), det gait_valid 6/6 (clean), sto only 5/6 gait_valid with one catastrophic episode (slip 18.1, leg 3 flag/drag, prog -0.01) -- real, not noise (all 6 det episodes sit 1.28-1.48, none under the cap). No dedicated quad-mode harness eval exists yet (eval_checkpoint.py ALL_MODES has no 'quad' entry) -- quad numbers above are read from training's periodic env-eval telemetry + video, not the exact-path harness; flagged as a harness gap, not a blocker for this call since the compound gate already fails on the walk leg.

