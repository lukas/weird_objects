# cw-amp-m3-pushacq1-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T23:43:55+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m3-pushsmoke1-style05

**wandb_id**: s2js9v0h

**hypothesis**: Plain English: the push smoke just showed the AMP style reward tolerates shove-recovery transients (topple rate fell 2-3x in 2M at constant dose, matching the task-only twin), so give the style-kept recipe the same honest 3x acquisition budget (6M) its noamp control cw-amp-m3-pushacq1-noamp is already running -- does style stay tolerant at acquisition scale, and does the style-kept walker match the noamp one on push survival? This produces the style-kept M3 hardening candidate and a matched style-vs-nostyle read at 6M. Single change vs pushsmoke1-style05: warm-start from its own checkpoint + 6M steps (push dose unchanged: dr.ext_push_prob=1.0, 10-25N / 0.15-0.4s / random direction, once per episode). Prediction-if-true: training tilt terminations keep falling to <=half the smoke's final window (<=4 pitch / <=4 roll), gate topples <=1/6 det and <=2/6 sto, gait_valid >=5/6, det prog med >=0.9, style_reward_mean stays >0.1 (discriminator not collapsed). Prediction-if-false: terminations plateau at the smoke's floor while the noamp acquisition keeps improving -- style becomes the binding constraint at scale; M3 hardening proceeds noamp or with style annealed. Strongest alternative: survival improves via crouch-statue (height_err leaves the 18-31mm walking band, prog collapses) -- pricing problem, fix before any M3 claim.

**gate**: Acquisition (6M, DR-0, push dose unchanged). PASS = DR-0 own-cfg gate det terminations <=1/6 AND sto <=2/6, gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, height_err in the 18-31mm walking band, style_reward_mean >0.1 at end, video shows stumble-and-keep-walking on pushed episodes. INFORMATIVE-plateau = gate topples no better than the 2M smoke (1 det / 1 sto) with training tilt-terms flat over the last 2M => flat-dose ceiling with style named; joint read vs cw-amp-m3-pushacq1-noamp decides whether style is the binding constraint. FAIL-styleveto = clearly worse than noamp acquisition on topples (>2x) or crouch-statue fingerprint. FAIL = collapse/numerical blowup.

**verdict**: The style-kept push walker holds up at acquisition scale and passes its pre-registered gate: after 6M steps with a guaranteed mid-episode shove (10-25N, 0.15-0.4s, random direction), the DR-0 own-cfg panel shows 1/6 det topples and 0/6 sto (gate <=1/6 det, <=2/6 sto; the 2M smoke was 1+1), gait_valid 6/6 det+sto, zero sacrificed legs, det prog med 1.23 / slip med 3.19 / fwd 0.64m, sto prog 1.01 / slip 3.49. Video: det_0 and sto_3 strips show upright six-leg walking straight through the pushed window (sto_3 covers 1.11m); det_3 is the one loss — clean walking ~13s then a late shove flips it on its back. No crouch-statue: stance height on video matches the parent despite the mild env/height_mm training drift (14.7->8.2). style_reward_mean ended 0.116 — above the 0.1 collapse bar but thinning steadily (0.30->0.12), worth watching in follow-ups. Honest caveat: training tilt terminations plateaued over the final ~3M at constant dose (pitch 7.4->6.1->6.9, roll 6.0->5.1->6.6 per window) — the prediction-if-true halving (<=4/<=4) did NOT happen; the single-push 10-25N regime is mined out and more same-dose budget would buy nothing. Next: this cycle launches the M3 escalation grid from this checkpoint — cw-amp-m3-pushhard1-style05-repeat3 (up to 3 shoves/episode, M3's explicit repeated-push requirement) and cw-amp-m3-pushhard1-style05-n2040 (single 20-40N shove) — decomposing push COUNT vs push FORCE as the next frontier. The style-vs-noamp joint read at 6M pends the concurrent cycle's cw-amp-m3-pushacq1-noamp verdict; if noamp reads clearly better, escalation respecs onto the noamp line are cheap.

