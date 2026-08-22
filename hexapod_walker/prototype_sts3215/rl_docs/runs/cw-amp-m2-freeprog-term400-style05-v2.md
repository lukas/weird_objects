# cw-amp-m2-freeprog-term400-style05-v2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T16:07:15+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-style05

**wandb_id**: s8lnr8oo

**hypothesis**: Plain English: re-run the Wave-1 style-vs-control fork with a motion library that actually contains the verified teacher's gait -- every AMP style arm so far trained against the frame-corrupted teacher_v1 (fb_20260822T145428 audit: raw absolute-tibia TripodGait fed unconverted, knee off up to 15.7deg vs the sim_gait_compat truth), so the discriminator has been pulling the policy toward a subtly WRONG gait and the 'AMP gave cleaner gait_valid, not real travel' fork resolution is contaminated. Single change vs cw-amp-m2-freeprog-term400-style05 (FAIL, statue): --amp-motion-lib=teacher_v2.npz (45/45 clips, slip in teacher band, built via the sim_gait_compat boundary). Everything else byte-identical: SLIPWALK pricing, term_penalty=400, style/task 0.5/0.5, from scratch, 2M discovery.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg), judged RELATIVE to cw-amp-m2-freeprog-term400-noamp AND -style05 at matched budget: PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six-leg cycling with net displacement. Secondary reads either way: amp/d_real-d_fake stays unsaturated, style_rew trend, and whether freeprog_pen leaves the -1.5/tick statue plateau for the first time in this family. Prediction-if-true: the clean prior gives the statue an escape gradient (teacher-like leg cycling earns style income immediately) and travel appears. Prediction-if-false: statue persists identically -> the corrupted-lib caveat is retired as an explanation and the 0.5 style weight being priced out (max 0.5/tick vs -1.5/tick charges) becomes the live suspect, read jointly with the -stylew2-v2 dose twin launched this cycle.

**verdict**: FAIL, twin to stylew2-v2 (both triaged jointly this cycle, eval run by this cycle since it had finished unstaged): DR-0 det median fwd travel 0.03m/15s (bar 0.10m), slip med 8.09/m, gait_valid 5/6 det (leg3 sacrificed ep5) + 6/6 sto, zero terminations. Contact sheet/frame strips visually indistinguishable from stylew2-v2 and from -noamp -- same in-place march/shuffle basin, clean teacher_v2 lib made no visible difference. AMP healthy (d_real 0.781 vs d_fake -0.961) but style_reward_mean 0.053, env/reward_amp_style final 0.027/tick -- an order of magnitude below freeprog_pen's -1.49/tick. W&B reward fell every quarter (-72/-239/-451/-571), never rising -- genuine flat/declining FAIL. Retires the 'corrupted teacher_v1 lib' caveat definitively (clean teacher_v2, still statues) and confirms per the pre-registered decision rule: with stylew2-v2 ALSO statued at 4x dose, the AMP-style-income route is refuted at 2M discovery scale for this pricing family across the whole tested dose range. Next lever per STATUS.md's ladder: a genuinely new income mechanism, not another blend dose -- cw-amp-m2-freeprog-term400-fixedcmd{,-seed11} launched same cycle (isolates task-command complexity vs reward shape using the SLIPWALK bank's own literal fixed-command setup).

