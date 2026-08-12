# cw-stand-footlow2-level1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-12T15:05:02+00:00

**pod**: hexapod-mjx-train-2

**steps**: 10000000

**parent**: cw-stand-footlow2-hard1

**wandb_id**: kr6zraq1

**hardware_ready**: False

**hypothesis**: Teach the stand to END LEVEL, not just at height: on hardware 2026-08-12 footlow2-hard1 completed the first-ever full belly-stand-belly round trips (2/2, bench_blast 20260812_104910 and _105451) but holds a persistent ~8 deg start-relative body roll after the rise — the policy sees the tilt error in obs yet keeps the choreographed joint pattern, because the whole lineage trained at dr-scale 0.0 on a perfect floor where symmetric joints ARE level, so the corrective state was never in the training distribution (the reward already pays levelness at sigma 1.5 deg — a state-coverage gap, not a reward gap). This arm turns the physics back on around the SAME anchor recipe, warm-started from the hard1 checkpoint: dr-scale 0.35 (contact/servo/latency asymmetries make symmetric rises come up tilted, so IMU-feedback leveling becomes the only way to collect tilt income), ground slope to 5 deg absolute, tipped starts at prob 0.30 (hold episodes spawn standing-but-leaning with a LEVEL tilt reference — sim_env keeps the ref level for tipped starts precisely so the lean is visible and paid to remove). Prediction-if-true: tipped-start holds re-level to <= 2 deg and DR0 retention keeps all four hard1 clauses — hardware stands stop leaning. Prediction-if-false: the BC anchor fights the corrective motion and either leveling never emerges or the rise regresses — then the next lever is annealing the anchor coef under DR, not more DR.

**gate**: PASS if own-cfg (dr-scale 0.35 + dr.ground_tilt_deg=5.0 + dr.tipped_start_prob=0.30) det+sto: tipped-start hold episodes end within 2 deg of LEVEL on >= 10/12 with no park; AND DR0 retention of the hard1 clauses — cold-start det rises (flat/bridge/crouch) valid_plant with h_err <= 5 mm, det hold no real park (no foot duty < 0.5 with end_clear > 2 mm), det+sto lower >= 10/12. FAIL if leveling does not emerge or any retention clause regresses. Quote the end-tilt distribution vs the parent (hardware holds ~8 deg start-relative lean, bench_blast 20260812_104910/_105451).

**verdict**: FAIL — DR0-retention regression, KNOWN two-foot park exploit reopened on plain (non-tipped) hold episodes: det hold 4/6 success vs parent hard1 6/6 (2/6 episodes park feet idx1+idx4 duty 0.03-0.09, end_clear 1.3-4.1mm, roll_tail 4.7-5.0deg, harness success=False; parent: all 6 feet duty 0.95-0.99, roll_tail 0.1deg uniform). sto hold worse under own-DR0.35 (2/6 success, roll_tail up to 9.5deg). The arms own primary hypothesis (tipped-start hold episodes re-level to <=2deg) is UNTESTED by the standard draw -- zero tipped-start episodes appeared across 24 hold episodes (det+sto, both DR passes); start_kind stayed plant throughout. Moot: retention already fails on its own AND clause. One-line known-exploit stop, no forensics. hard1 stays the deployment candidate.

