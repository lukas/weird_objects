# cw-stand-margin1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T08:03:15+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: jp1k83oh

**hardware_ready**: no

**hypothesis**: Teach the stand specialist to stop standing up through a balance knife-edge: right now it ends every rise balanced on just three feet with its weight teetering on the edge (the exact mechanism behind the real robot's 10/10 stand-up falls, found by replaying the hardware tapes), and this arm pays it for keeping its weight deep inside its planted feet instead. One variable vs holdbc1-hard1: reward.k_support_margin 0 -> 1.0 (an already-built, never-trained term; bank-verified 08-12 that the wide plant earns the 40mm cap while the replay's own degenerate 0/1/4 support set earns 0.19x, so the gradient points off the knife edge). Prediction-if-true: det rise ends with >=4-5 loaded feet and plant_margin_mm up vs the frozen parent, retention clean — a deployable stand candidate addressing the bench trip. Prediction-if-false: margin income is eaten by the BC anchor pinning the same trajectory (margin stat unchanged), or the policy buys margin with a new cheat (outrigger/park — disqualifying). Strongest alternative: the knife-edge is forced by the rise reference itself, in which case the fix is reference-side, not pricing.

**gate**: PASS if det rise plant_margin_mm median rises vs frozen holdbc1-hard1 on the same eval (matched seeds) AND det rise ends >=5/6 valid_plant with no new falls AND nominal hold/lower retention matches the parent probe (hold >=11/12, lower det >=5/6) with NO outrigger/flag-leg cheat on lower video (riserock3/4 lesson). FAIL if margin stat unchanged (anchor-pinned) or any known-exploit cheat appears in retention.

**verdict**: FAIL both pre-registered branches: det-rise plant_margin_mm median 157 vs matched frozen parent 154 (+3mm, inside noise — static hold margins differ 155 vs 154 on identical stances, so the stat did NOT move) AND known-exploit in retention: det hold parks foot idx1 (duty 0.05 vs parent 0.90, HUD feet #.####, outrigger visible in frames) — the support-margin income is park-compatible (a 5-foot hull is still wide). Secondary wins don't rescue it (rise det 6/6 vs parent 5/6, lower det 5/6 vs 4/6, roll tail 4.8 vs 6.4deg). Pricing cannot fix the 3-foot knife-edge without waking the park; anchor-side lever stands.

