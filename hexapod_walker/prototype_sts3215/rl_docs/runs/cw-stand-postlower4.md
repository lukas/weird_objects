# cw-stand-postlower4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-15T11:39:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-postlower3

**wandb_id**: rnq37awx

**hypothesis**: Stop teaching the robot to flop onto its belly before standing back up: postlower3's in-context sequence training FAILED because its rise schedule starts at belly-frame 0, paying the policy to re-descend/splay and redo the flat stand-up after every sit (Cohort c3 dig-in: det post-lower rise 0.419 vs parent 0.967, over_current stalls mid-detour, detour visible in success re-renders too). This arm is the identical recipe with ONE change: goal.mode_seq_rise_from_h=1 makes every mid-sequence rise schedule start at the robot's CURRENT height ('stand up from where you are', no commanded descent; default-off key, tests test_rise_from_h_starts_at_current_height). Prediction-if-true: post-lower rises become direct push-ups and Cohort c4 moves sto post-lower rise above the parent's 0.801 with retention intact. Prediction-if-false: post-lower rise still <= parent despite no descent command — in-context sequence training itself doesn't transfer to the runner's reanchor semantics, closing the class (second miss) and sending the train==deploy schedule-alignment fork to the operator.

**gate**: Discovery, 2M, judged on PRE-REGISTERED Cohort c4 (SESSION_BULK_GATE.md 'Cohort c4', fresh banks det 960000../sto 970000.., candidate spec-pl4, registered before training). PASS = all six clauses incl. the new eye clause 6 (watched post-lower re-renders show a DIRECT push-up; any belly detour = FAIL regardless of counts). PARTIAL = sto post-lower rise CI-separated above parent 0.801 but <0.90 with retention+eye clean -> one 6M hardening rerun on fresh c5. FAIL = post-lower rise <= parent or any retention/visual/eye clause broken -> in-context class closed (second miss), schedule-alignment fork goes to the operator.

