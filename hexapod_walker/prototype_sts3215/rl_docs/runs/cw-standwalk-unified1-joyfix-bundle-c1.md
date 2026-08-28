# cw-standwalk-unified1-joyfix-bundle-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-28T16:24:36+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: 31xrlzi8

**hypothesis**: Plain English: the combined fix for everything the 08-28 drivability audit found real — full-circle command headings (lateral commands finally drawn), the deployable leg-odometry velocity estimator in the obs (policy can finally see its own speed), and the direct normalized command objective — applied together to the unified 60s session recipe, warm from long-s0's 16M PASS checkpoint, as the bundle arm of the 4-arm canary grid (the three single-lever siblings attribute any effect). Coupled bundle justified: heading exposure without velocity observability may not be learnable, and cmd_track prices exactly the velocity error mode 3 makes observable. Predict-if-true: at 2M the lateral response ratio at least doubles from 0.033 AND fwd stays >=0.35 with terminations recovering to parent band. Predict-if-false: the joint perturbation destabilizes the warm policy worse than any single lever (compare siblings) — would say stage the levers sequentially in acquisition. Strongest alternative: bundle matches the best single lever (no synergy) — then acquisition funds only that lever.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only): at 2M — training reward recovering (not collapsed vs siblings at matched steps); DR-0 det walk gait_valid >=5/6 no new sacrificed leg; session terminations <=6/90; probe: max(left,right) speed_ratio >=0.07 AND fwd >=0.35. PASS -> 2-seed 16M acquisition of the bundle (or the winning subset per siblings); FAIL with siblings healthy -> stage levers sequentially.

**verdict**: CANARY FAIL - MECHANISM (2 of 4 AND-gated criteria fail, joint evidence with hdg-c1). Bundle combined heading_max=pi (full-circle) + walk_obs_body_vel=3 (leg-odometry velocity obs) + k_walk_cmd_track=2.0 on top of the unified 60s recipe, warm from long-s0's 16M PASS ckpt. (1) probe_cmd_sensitivity response-mode FAILS the lateral bar: max(left,right) speed_ratio = max(0.011, -0.013) = 0.011, required >=0.07 -- essentially unchanged from hdg-c1's own 0.006 FAIL, so neither the velocity observability fix nor the direct command-tracking reward rescued the lateral response the heading-circle lever alone failed to produce. fwd speed_ratio 0.4 does clear its own >=0.35 bar. (2) training reward is NOT recovering: full wandb_history trace shows ep_rew_mean falling from +38 (0.7M) to a floor of -320..-480 by 1.1-2.0M with NO late recovery (ends -414.6 at 2.03M, matched-step reference cont1 trend is +9..+57 over the same window) -- flat at the bottom, not the shared trough-then-recover shape every other joyfix sibling shows. (3) DR-0 det walk gait_valid 6/6, sac [], terms 0 -- this criterion alone is fine. Evidence: probe /tmp/probe_bundle_c1.json, wandb run 31xrlzi8, logs/ckpt_eval/cw_standwalk_unified1_joyfix_bundle_c1_{gate,owncfg}/report.json. Session terminations (mixedsession, still computing on train-2) cannot flip an already-2/4-failed AND-gate; left running for the record, not re-launched. Conclusion: a full +-180 heading circle is off this recipe's curriculum budget regardless of what else is bundled with it -- the staged heading-SET follow-up (hdgset1, single-lever, currently 2/4 PASS) is the live lever, not the bundle.

