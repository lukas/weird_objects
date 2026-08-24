# cw-walkcurr-pf-fwd6-actbias1-pdw05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T00:54:04+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1

**wandb_id**: cbj59l61

**hypothesis**: Plain English: actbias1 (this cycle's action-recentering fix) just closed the belly-sit collapse that blocked every prior rung-1 arm -- video-confirmed clean, level, non-terminating stand on both det and sto -- but exposed the NEXT-cheapest static optimum: a stable 3-leg-planted tripod HOLD (park), not a walk. actbias1's recipe (plain rscale50 + bias) never fixed the OTHER known confound this track already found and bank-proved on the hgt2 lineage: goal.park_duty_window_s=2.0 (default) is too long for the trailing per-leg duty-history buffer to fill and start charging a permanently-planted leg within any reasonably short evaluation window, so k_park_duty under-fires against a stand-still policy. Single new lever vs actbias1: goal.park_duty_window_s=0.5 (the exact, already bank-proven-safe confound fix from cw-walkcurr-pf-fwd6-hgt2-pdw05), stacked with the bias fix for the first time. Prediction-if-true: env/reward_park_duty becomes meaningfully more negative than actbias1's own (near-zero, unmeasured) baseline, walk_freeprog_score/walk_speed leave their flat-near-zero band, and det video shows the tripod unlock into six-leg cycling. Prediction-if-false (same clean stable stand): the park-duty charge class is insufficient even against a healthy (non-collapsing) base pose -- next fork raises k_park_duty dose directly (bank-legal up to 1.5x per the hgt-dose bank) before building the foot-contact-charge mechanism.

**gate**: Rung-1 gate: C-env det fixed-forward panel -- prog_ratio>0 and gait_valid on >=4/6 det episodes with visible forward travel on video, env/walk_freeprog_score trends toward/past 0, clip_fraction stays healthy. Additionally: env/height_err_mm should stay near actbias1's own healthy ~15mm band (a regression toward the old 50-116mm collapse band would mean the two fixes interact badly) and env/reward_park_duty should read meaningfully nonzero/negative against the current stand (near-zero on actbias1 alone). PASS = rung-1 lands. FAIL with a still-static-but-stable stand = park-duty class insufficient at this dose, escalate dose before the foot-contact-charge mechanism.

**verdict**: The confound fix mechanically works but doesn't unlock walking -- park-duty dose is insufficient against a healthy, non-collapsing stand. Evidence: env/reward_park_duty now reads meaningfully nonzero (-0.005 -> -0.025 over the run, vs near-zero on actbias1 alone), confirming the goal.park_duty_window_s=0.5 confound fix from hgt2-pdw05 fires correctly here too. But every walking metric is UNCHANGED from actbias1: env/height_err_mm stays low and stable (14-22mm, no collapse), env/walk_direction_err_deg pinned at ~87-89deg (perpendicular/wrong-way) the entire run, env/walk_freeprog_score flat -0.07..-0.08 (no zero-crossing), env/walk_loadslip_ratio still exploding to ~20x cap. Gate eval (podeval, watched det+sto strips): det 0/6 gait_valid, 5/6 legs sacrificed (worse than actbias1's 3/6 -- an even tighter tripod-lock), dir_err 71deg, prog_ratio 0.00; sto 5/6 gait_valid but dir_err 88.6deg and slip/m 43.98 (in-place thrashing, not travel). Det video: robot standing level and upright, visually static across all 10 frames -- same clean park-stand as actbias1, park-duty pricing did not dislodge it. This is exactly the track STATUS's own pre-registered prediction-if-false: park-duty pricing is insufficient at this dose even against a healthy base pose. Reward AND walk-eval both flat/non-improving with adequate budget (quarters 45.3/41.4/28.5/12.9, clip_fraction healthy 0.002->0.14) = genuine FAIL per the 08-21 ruling, not misalignment-needing-more-budget. Next: escalate k_park_duty dose directly (bank-legal to 1.5x, the named next lever) before building the foot-contact-charge mechanism or flagging BC-kickstart to the operator.

