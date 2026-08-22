# cw-amp-m2-taskdown01-style1-v3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T18:06:22+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-taskdown01-style1-v2

**wandb_id**: 7f7r8l0m

**hypothesis**: Retry of cw-amp-m2-taskdown01-style1-v2 (REFUSED at launch on a code_sha_local/pod mismatch from a concurrent snapshot race, never actually trained 0 steps) -- identical config, no changes. Plain English: twin to the currently-RUNNING cw-amp-m2-styleonly-v2, asking the dose question from the other side -- does keeping a SMALL task signal (amp-task-weight 0.1, ~10% of the freeprog stack) preserve command-following pressure without re-burying the AMP style gradient the way every 0.5/0.5 style arm did? Read JOINTLY with styleonly-v2 (task_weight=0): if styleonly climbs but this arm stays pinned, even 0.1x task charges re-bury the gradient; if both climb, this is the better base; if both stay pinned <=0.1, AMP-mechanism defect confirmed with n=2.

**gate**: Discovery (2M, DR-0 harness walk mode 6 det + 6 sto, own cfg, + W&B amp/*): INFORMATIVE-PASS = amp/style_reward_mean >=0.3 by run end with discriminator unsaturated; FULL PASS additionally needs median det fwd travel >= 0.10 m/15s with gait_valid >=4/6 det and six legs cycling on video. FAIL = style pinned <=0.1 (task charges re-bury the gradient even at 0.1x -- read against styleonly-v2 to locate the threshold). Judge within scope: mechanism-isolation arm, not a gait-quality claim.

**verdict**: FAIL (mechanism-isolation, matches its own pre-registered branch). Evidence: amp/style_reward_mean stayed pinned 0.04-0.13 the whole run, ending 0.087 -- under the 0.3 INFORMATIVE bar AND at/under the 0.1 FAIL threshold, LOWER than its zero-task twin cw-amp-m2-styleonly-v2's 0.119 -- despite a healthy discriminator (d_real 0.79/d_fake -0.96, unsaturated, gp finite, 124 disc updates). Fresh DR-0 gate eval (own pod): det gait_valid 1/6, sto 3/6, median fwd travel 0.03m both modes (bar 0.10m), slip 7.25-7.90/m, sacrificed legs [1,3,5] in 3-4/6 det episodes -- contact sheet + frame strips show the same splayed near-static tripod as the whole freeprog/statue family, no coordinated cyclic gait. Training ep_rew_mean FELL every quarter (-2.4/-30.4/-69.2/-85.2) -- genuine flat/declining per the 08-21 ruling, not continue-longer. Why: read JOINTLY with styleonly-v2 per the pre-registered plan -- 'if styleonly climbs but this arm stays pinned, even 0.1x task charges re-bury the gradient' is exactly what happened. Confirms even a SMALL (0.1x) task-charge competition re-buries the AMP style gradient at 2M/from-scratch budget; across the full tested task-weight range (0.0/0.1/0.5/1.0/2.0 -- styleonly/taskdown01/style05-v2/stylew2-v2/style05 lineage) every arm lands in the same statue/near-statue basin. What's next: 'more style weight' and 'less task weight' are BOTH now closed as fixes inside the freeprog pricing family -- the fix has to replace the TASK reward's shape (AMP_LOCOMOTION.md sec5.1's simple Gaussian velocity/yaw/upright/weak-height kernel set) rather than dial AMP's own weight further. Flagging DIG-IN for that redesign: it must avoid reopening the already-root-caused legacy statue exploit (rise_finish/posture/height kernels overpaying stillness) that the freeprog stack was built to fix in the first place -- real reward-architecture work, not a cfg toggle.

