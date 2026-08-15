# cw-stand-postlower4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-15T11:39:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-postlower3

**wandb_id**: rnq37awx

**hardware_ready**: False

**hypothesis**: Stop teaching the robot to flop onto its belly before standing back up: postlower3's in-context sequence training FAILED because its rise schedule starts at belly-frame 0, paying the policy to re-descend/splay and redo the flat stand-up after every sit (Cohort c3 dig-in: det post-lower rise 0.419 vs parent 0.967, over_current stalls mid-detour, detour visible in success re-renders too). This arm is the identical recipe with ONE change: goal.mode_seq_rise_from_h=1 makes every mid-sequence rise schedule start at the robot's CURRENT height ('stand up from where you are', no commanded descent; default-off key, tests test_rise_from_h_starts_at_current_height). Prediction-if-true: post-lower rises become direct push-ups and Cohort c4 moves sto post-lower rise above the parent's 0.801 with retention intact. Prediction-if-false: post-lower rise still <= parent despite no descent command — in-context sequence training itself doesn't transfer to the runner's reanchor semantics, closing the class (second miss) and sending the train==deploy schedule-alignment fork to the operator.

**gate**: Discovery, 2M, judged on PRE-REGISTERED Cohort c4 (SESSION_BULK_GATE.md 'Cohort c4', fresh banks det 960000../sto 970000.., candidate spec-pl4, registered before training). PASS = all six clauses incl. the new eye clause 6 (watched post-lower re-renders show a DIRECT push-up; any belly detour = FAIL regardless of counts). PARTIAL = sto post-lower rise CI-separated above parent 0.801 but <0.90 with retention+eye clean -> one 6M hardening rerun on fresh c5. FAIL = post-lower rise <= parent or any retention/visual/eye clause broken -> in-context class closed (second miss), schedule-alignment fork goes to the operator.

**verdict**: FAIL (Cohort c4, n=600 held-out, banks 960000../970000..): the mode_seq_rise_from_h schedule fix WORKED as designed -- 10 watched re-renders (6 fail+4 clean) show every post-lower rise is now a direct push-up, zero belly-detours (the c3 defect is gone) -- and det post-lower rise recovered 0.419->0.872, sto 0.631->0.690, but both stayed short of the parent (det 0.967, sto 0.801) and the pre-registered parity bar (det session zero-fall 0.863 vs 0.95, det post-lower rise 0.872 vs 0.967, sto post-lower rise 0.690 vs 0.90). Crown jewels clean (det first-rise 0.99 all strata >=0.97, lower 1.0 det+sto). Remaining falls are a genuine over_current stall (switch_peak_a ~2.6A), not an exploit. SECOND miss of the in-context sequence-training mechanism (c3=wrong mechanism, c4=right mechanism/still short) -- per two-miss discipline the class is CLOSED for further dose/diet/schedule resweeps; this is the pre-registered if-false branch, not a surprise, so no dig-in needed. Next lever is an operator product-contract call (align runner rise-schedule semantics to train==deploy, or price post-lower rise directly) -- escalated [operator] in STATUS.md. Product baseline (c1 hierarchy) unaffected. Full chain: SESSION_BULK_GATE.md 'Cohort c4 RESULTS'.

