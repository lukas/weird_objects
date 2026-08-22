# cw-amp-m2-taskdown01-style1-v2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T18:03:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-style05-v2

**hypothesis**: Plain English: twin to cw-amp-m2-styleonly-v2 asking the dose question from the other side — does keeping a SMALL task signal (10% of the freeprog stack, ~-0.15/tick charges vs style's potential 1.0/tick) preserve command-following pressure without re-burying the style gradient? Every prior style arm ran task charges 10-30x the realized style income; this arm inverts the ratio for the first time (style capacity ~6x the scaled task charge floor). Changes vs cw-amp-m2-freeprog-term400-style05-v2: --amp-task-weight 0.5->0.1, --amp-style-weight 0.5->1.0; everything else byte-identical (teacher_v2 lib, SLIPWALK pricing incl. term_penalty=400 now scaled to 40 effective, stress_mix commands, from scratch, 2M discovery). Read JOINTLY with styleonly-v2: if styleonly climbs but this arm stays pinned, even 0.1x task charges re-bury the gradient (fix = income normalization or style-first curriculum); if both climb, this arm is the better base (style learnable AND command signal alive); if both stay pinned, AMP mechanism defect confirmed with n=2. Prediction-if-true: style_reward_mean climbs off the 0.05-0.07 band AND some commanded-direction obedience appears (dir_err below the ~77deg family plateau). Prediction-if-false: identical statue/shuffle to the closed lever ladder, style pinned. Cheat watch: in-place teacher mimicry farming style income while ignoring commands — INFORMATIVE for the mechanism but named as the next repricing problem.

**gate**: Discovery (2M, DR-0 harness walk mode 6 det + 6 sto, own cfg, + W&B amp/*): INFORMATIVE-PASS = amp/style_reward_mean >=0.3 by run end with discriminator unsaturated (mechanism learnable under light task pressure); FULL PASS additionally needs median det fwd travel >= 0.10 m/15s with gait_valid >=4/6 det and six legs cycling on video. FAIL = style pinned <=0.1 (task charges re-bury the gradient even at 0.1x — read against styleonly-v2 to locate the threshold). Judge within scope: this is a mechanism-isolation arm, not a gait-quality claim.

**refused_reason**: hexapod-mjx-train-0 code marker 799d8f88405b9278ef28c2edef9b9d375df0feb7 != local HEAD bc394b08290fa20f9755569196a018ca1725e342. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

