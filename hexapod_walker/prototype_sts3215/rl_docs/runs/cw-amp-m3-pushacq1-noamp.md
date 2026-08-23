# cw-amp-m3-pushacq1-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T23:27:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m3-pushsmoke1-noamp-r4

**wandb_id**: us2ovifu

**hypothesis**: Plain English: the push smoke showed the walker actively LEARNING to survive shoves -- topple rate fell ~3x in 2M and was still falling at the cutoff, with reward still rising (+88 in the last quarter) -- so give the identical recipe an honest 3x acquisition budget (6M) and measure how far the topple rate falls; this is the direct 'survive repeated randomized pushes' capability run for M3. Single change vs pushsmoke1-noamp-r4: warm-start from r4's own checkpoint + 6M steps (push dose unchanged, 10-25N / 0.15-0.4s / random direction, once per episode). Prediction-if-true: training tilt terminations keep falling to <=half r4's final window (<=8 pitch / <=5 roll), and the DR-0 own-cfg gate topples drop to <=1/6 det and <=2/6 sto with gait_valid >=5/6 and det prog med >=0.9. Prediction-if-false: terminations plateau at r4's floor -- a flat-dose ceiling; next lever is the push-magnitude curriculum (brief section 7.4's stated design) and/or a repeated-push mechanism extension (M3 wants REPEATED pushes; dr.ext_push_* currently draws exactly one per episode), not more budget. Strongest alternative: survival improves but via parking/crouching (height_err drops out of the walking band, prog collapses) -- a pricing problem to fix before any M3 claim.

**gate**: Acquisition (6M, DR-0, push dose unchanged from r4). PASS = DR-0 own-cfg gate det terminations <=1/6 AND sto <=2/6, gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, height_err stays in the 18-31mm walking band (no crouch-to-survive), video shows genuine stumble-and-keep-walking on pushed episodes. INFORMATIVE-plateau = gate topples no better than r4's (1 det / 3 sto) with training tilt-terms flat over the last 2M => flat-dose ceiling named; next lever is the push curriculum / repeated-push extension, not budget. FAIL = collapse/statue/numerical blowup.

**verdict**: 6M push-survival acquisition (dose unchanged from pushsmoke1-noamp-r4) converts 'survive-most-pushes' into 'survive-every-sampled-push' at the SAME dose -- clean gate PASS, beats the pre-registered bar outright. DR-0 own-cfg gate: 0/12 terminations across ALL det+sto episodes (vs r4's 1/6 det + 3/6 sto), gait_valid 6/6 both modes, zero sacrificed legs, det prog med 1.06 (bar >=0.9) / sto 1.02, slip med 3.93 det / 3.67 sto, height_err single-digit-to-12mm all 12 episodes (tighter than the 18-31mm walking band -- real tracking, not a crouch: several episodes prog_ratio >1.0). Training tilt terminations kept falling the whole 6M (tilt_pitch window-mean 18.6->7.5, tilt_roll 11.4->5.4), reward rose 143->381 (Q3->Q4 plateauing but never falling) -- reward and gate agree, textbook continuation per the 08-21 ruling. Video (det_0 roll peak 10.1deg, sto_4 roll peak 15.7deg -- the two hardest-hit episodes): clean six-leg cycling continues through and after the shove, no flip, no drag, no statue. Blunt caveat: still ONE push per episode (M3's own bar wants REPEATED pushes) -- this measures survive-a-single-random-shove-then-keep-walking, not recovery-after-a-full-knockdown (no sampled episode came close to tipping over). Next (this cycle): built + bank-tested dr.ext_push_repeat_max (default 1 = bit-exact; >1 draws that many non-overlapping pulses spaced by dr.ext_push_gap_s, capped by dr.ext_push_horizon_s) -- test_ext_push_injection.py 17/17 (12 orig + 5 new), full test_sim_env.py 44/44, semantics bank 163/163 (+4 skip/1 xfail/1 pre-existing unrelated fastprof fail, untouched); launching the repeated-push smoke from this checkpoint.

