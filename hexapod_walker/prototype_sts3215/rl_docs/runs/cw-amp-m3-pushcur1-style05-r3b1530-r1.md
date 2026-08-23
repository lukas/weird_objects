# cw-amp-m3-pushcur1-style05-r3b1530-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T00:47:19+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m3-pushhard1-style05-repeat3

**wandb_id**: 9yf07d2m

**hypothesis**: Plain English: the walker that survives three 10-25N shoves per episode now takes the same up-to-three shoves at a moderately harder 15-30N — does multi-push survival extend up the force axis when the dose is bridged instead of jumped? This is the style05-line curriculum rung with COUNT HELD (repeat_max=3 kept, from repeat3's own PASSing ckpt); it complements, not duplicates, the noamp batch (b1530 single-shove bridge, n2040-c1 budget control, repeat2 count staging — none combine count and force). Both n2040 twins showed a flat jump to 20-40N exceeds the 6M-recoverable envelope (terms flat, 4/12 topples), so bridge first. Prediction-if-true: tilt-terms rise modestly at start then FALL (unlike repeat3's own flat profile at base dose — a falling trend here would also say the count plateau was dose-headroom, not capacity), gate holds topples <=2/6 det + <=3/6 sto. Prediction-if-false: terms flat AND reward flat again at the bridged dose -> the count-hardened substrate has no force headroom either; the M3 recoverable envelope is ~25N even under curriculum - informative ceiling for the M5 transfer spec. Strongest alternatives: (a) crouch-statue survival - watch height band + det prog med >=0.9; (b) discriminator veto as style thins (0.109 at repeat3 end) - watch style_reward_mean vs 0.1.

**gate**: Hardening (6M, DR-0, dr.ext_push_repeat_max=3 AND dr.ext_push_n=15-30N). PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=2/6 det AND <=3/6 sto, style_reward_mean >0.1 at end, majority of surviving pushed episodes roll_class=recovered. INFORMATIVE-ceiling = topples above bar but training tilt-terms still falling at cutoff => continue per 08-21 ruling. FAIL-statue = det prog med <0.6 or crouch fingerprint (height band exit). FAIL = collapse/NaN.

