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

**verdict**: FAIL, and the informative half of the joint read with cw-amp-m2-styleonly-v2: even a SMALL task charge (amp-task-weight 0.1, ~10% of the freeprog stack) re-buries the fragile style gradient the pure-style twin showed -- amp/style_reward_mean 0.087 (LOWER than styleonly-v2's 0.119, despite non-zero task competition theoretically adding a second income source) and ep_rew_mean DECLINING every quarter (-2.4/-30.4/-69.2/-85.2, the opposite of styleonly-v2's monotonic rise) -- confirms even 0.1x SLIPWALK-stack task charges are enough to flip the training-reward trend back to the family's usual decline. DR-0 gate: det gait_valid 1/6 (vs styleonly-v2's 0/6, a small improvement), sto gait_valid 3/6 (vs 0/6), speed 0.015-0.035 m/s (vs 0.006-0.025) -- legs [1,3,5] still sacrificed in the majority of episodes, same tripod as its twin. Video/contact sheet: same near-static splayed statue across all 10 frames, no visible coordinated cycling. READS AS: task_weight=0.1 sits BETWEEN the two failure modes (pure task-buried statue at 0.5/0.5, pure-style weak-but-rising at 0.0/1.0) without capturing either's partial upside cleanly -- confirms the STATUS/OPERATOR_QUESTIONS q_20260822T1815Z reading that the SLIPWALK anti-slip apparatus itself (not just its relative dose) is the wrong shape for a from-scratch AMP actor: even heavily discounted, its per-tick charges are structured to punish any noisy exploration hard enough to prevent an AMP-style gradient (proven weakly alive by styleonly-v2) from ever compounding. Read jointly, n=2 confirms: dose-tuning the EXISTING SLIPWALK stack (0.0/0.1/0.5/1.0/2.0 style weights all tried across this and prior arms) does not find a working point. CLOSES the task/style DOSE ladder on the SLIPWALK-derived reward architecture. Next real lever is the section-5 minimal-Gaussian-task reward rewrite itself (q_20260822T1815Z), not another task/style ratio.

