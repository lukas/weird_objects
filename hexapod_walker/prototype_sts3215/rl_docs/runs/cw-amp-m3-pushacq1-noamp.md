# cw-amp-m3-pushacq1-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T23:27:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m3-pushsmoke1-noamp-r4

**wandb_id**: us2ovifu

**hypothesis**: Plain English: the push smoke showed the walker actively LEARNING to survive shoves -- topple rate fell ~3x in 2M and was still falling at the cutoff, with reward still rising (+88 in the last quarter) -- so give the identical recipe an honest 3x acquisition budget (6M) and measure how far the topple rate falls; this is the direct 'survive repeated randomized pushes' capability run for M3. Single change vs pushsmoke1-noamp-r4: warm-start from r4's own checkpoint + 6M steps (push dose unchanged, 10-25N / 0.15-0.4s / random direction, once per episode). Prediction-if-true: training tilt terminations keep falling to <=half r4's final window (<=8 pitch / <=5 roll), and the DR-0 own-cfg gate topples drop to <=1/6 det and <=2/6 sto with gait_valid >=5/6 and det prog med >=0.9. Prediction-if-false: terminations plateau at r4's floor -- a flat-dose ceiling; next lever is the push-magnitude curriculum (brief section 7.4's stated design) and/or a repeated-push mechanism extension (M3 wants REPEATED pushes; dr.ext_push_* currently draws exactly one per episode), not more budget. Strongest alternative: survival improves but via parking/crouching (height_err drops out of the walking band, prog collapses) -- a pricing problem to fix before any M3 claim.

**gate**: Acquisition (6M, DR-0, push dose unchanged from r4). PASS = DR-0 own-cfg gate det terminations <=1/6 AND sto <=2/6, gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, height_err stays in the 18-31mm walking band (no crouch-to-survive), video shows genuine stumble-and-keep-walking on pushed episodes. INFORMATIVE-plateau = gate topples no better than r4's (1 det / 3 sto) with training tilt-terms flat over the last 2M => flat-dose ceiling named; next lever is the push curriculum / repeated-push extension, not budget. FAIL = collapse/statue/numerical blowup.

