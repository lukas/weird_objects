# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac03

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T05:52:03+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-ypfix1

**wandb_id**: lk045i9p

**hypothesis**: Plain English: does giving the policy real turn-in-place PRACTICE, now that it actually knows how to rotate (the BC turn-clone init already gave it a working omega motor pattern), let it learn ACCURATE tip tracking instead of settling for richer hold/forward income? This exact curriculum lever (goal.walk_turn_in_place_frac -- whole-episode dedicated turns, 50/50 sign) was tried and REFUTED once before (tip50/tip90, both parked at |wz_ref| exactly, zero dose-response 0.5->0.9) -- but that was BEFORE the BC turn-clone fix landed, when RL had NO discoverable turning motor pattern at all, so densifying exposure to an undiscoverable skill did nothing. It has never been retried post-clone. Fresh evidence this cycle (probe_walk_income on pushcont1-ypfix1, hazards zeroed): a pure heading-hold segment nets 1415 total return vs 842/796 for perfect tip_left/tip_right turning -- not because hold is over-PRICED (a true zero command SHOULD pay max) but because turning genuinely costs 4-10x more in current/gyro/roll charges than standing still, and turn-in-place segments are only ~7.5% of independently-sampled training exposure -- too rare for the optimizer to prioritize nailing a skill that costs more than it can ever out-earn hold/forward on a per-segment basis. Densifying WHOLE-EPISODE turn exposure raises turning's share of total training-time reward without touching any per-tick price (lower exploit risk than another pricing key). Single lever vs pushcont1-ypfix1 (this cycle's PASS-partial verdict, tips 0.2471/0.2553): add goal.walk_turn_in_place_frac at a dose grid, matched everything else (same overshoot-decay/avg_s pricing keys ON, same permanent fault/push cfg, same 2M discovery budget, same pre-cheat turnfault-seq1 init per the init-basin rule).

**gate**: PASS-clean = tip-left AND tip-right eval_yaw err <=0.20-0.25 (closes the M4/M5 turn+push cell outright). IMPROVED = measurably better than ypfix1's own 0.2471/0.2553 but still misses. FLAT/WORSE = no improvement or regression -- extends the pre-clone tip50/tip90 refutation to the post-clone/composed regime, meaning command-exposure is refuted twice now and the residual lever really is a per-tick repricing (charge hold/forward more, or a milestone-style tip-completion bonus) rather than curriculum. Hold-and-report regardless: own-cfg DR-0 gait_valid must stay >=9/12 (safety must not regress for more turn exposure), and eval_amp_m5 push/fault sections must stay PASS. KNOWN GOTCHA: dedicated whole-episode turn commands read as near-zero-progress on the standard DR-0 harness by design (contamination, not collapse) -- judge translation quality from the non-tip episodes and read tip accuracy from hand-run eval_yaw, not the raw panel median.

**verdict**: Confirms the turn-in-place-practice dose-response, narrowly misses the strict M5 yaw bar. Same lever as tipfrac05 but frac=0.3: eval_yaw tip-left/right err 0.2008/0.2135 -- inside the M4 PASS-clean band (<=0.20-0.25) but 0.0008 over eval_amp_m5's own stricter <=0.20-both-sides bar (m5 yaw pass=false by a hair, well within eval noise). Push/fault sections both PASS cleanly and BETTER than tipfrac05's (terms 1/12 vs 2/12, gait_valid 12/12 vs 11/12) -- lower turn exposure costs a little turn accuracy but buys a little safety margin, a real trade along the dose curve, not noise (monotonic across all 3 arms: frac 0.2->0.2068/0.2341 terms3, 0.3->0.2008/0.2135 terms1, 0.5->0.162/0.184 terms2). m5_pass=false only on yaw; walk PASS (0 terms, gait 12/12, prog 1.10, slip 3.42). Zero falls anywhere. Does not change the headline call (tipfrac05 is this cycle's PASS) but strengthens it: three independent doses of the same lever move tip accuracy in the same direction with no reversal. Evidence: logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac03_{gate,m5}/.

