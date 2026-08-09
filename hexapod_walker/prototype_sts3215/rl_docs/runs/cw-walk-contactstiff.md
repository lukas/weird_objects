# cw-walk-contactstiff

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T20:10:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 8p7lt5t3

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 13b next isolated axis: CONTACT COMPLIANCE. Real foam/rubber feet and floor spread contact stiffness; the champion only ever felt nominal contact. ISOLATED: dr-scale 0.0 with ONLY dr.contact_stiff_scale=0.7,2.0 (the full-DR envelope). Champion baseline MEASURED FIRST per c59 lesson (c62, logs/ckpt_eval/champ_baseline_contactstiff): gv 12/12, 0 term, but the 2 hardest det draws collapse to fwd 0.72/0.77m at slip 3.4-3.7 (nominal band 1.57m/0.96) - the axis is NOT covered free. Plain: keep walking the same on squishy vs hard ground. Prediction-if-true: exposure fixes the tail - det worst-2 draws fwd >=1.0m, det med >=1.2m, gv 12/12, 0 term, DR0 nominal retention in champion band. Prediction-if-false: extreme stiffness draws are gait-breaking like ice (worst draws stay <=0.8m) - compliance joins the contact-pricing operator-calibration class. Strongest alternative: median improves but tail persists (partially trainable) - judge per-episode vs the named baseline. Parent: cw-walk-longdist-r2.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.contact_stiff_scale=0.7,2.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd >=1.2m AND det worst-2 draws beat champ baseline 0.72/0.77m outside noise (>=1.0m); plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: FAIL (pre-registered if-false): contact-compliance extremes are NOT trainable by exposure — 20M steps at dr.contact_stiff_scale=0.7,2.0 left the tail exactly where the replicated champ baseline sits (det worst-2 fwd 0.74/0.67m at slip 3.2-4.4 vs baseline 0.72/0.77 and 0.60/0.67; gate required >=1.0m). Median unchanged too (det 1.42m = baseline 1.42/1.46) — exposure added nothing anywhere on the axis. gv 12/12, 0 term, DR0 nominal retention clean (slip med 0.93). Frames: extreme-stiffness det draws churn in place, splayed and skating, no falls. Compliance extremes join the contact-pricing operator-calibration class (with friction, fric50/friclow evidence). Do not requeue exposure arms on this axis.

