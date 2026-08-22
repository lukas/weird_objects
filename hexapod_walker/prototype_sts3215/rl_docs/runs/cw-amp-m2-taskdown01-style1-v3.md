# cw-amp-m2-taskdown01-style1-v3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T18:06:22+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-taskdown01-style1-v2

**wandb_id**: 7f7r8l0m

**hypothesis**: Retry of cw-amp-m2-taskdown01-style1-v2 (REFUSED at launch on a code_sha_local/pod mismatch from a concurrent snapshot race, never actually trained 0 steps) -- identical config, no changes. Plain English: twin to the currently-RUNNING cw-amp-m2-styleonly-v2, asking the dose question from the other side -- does keeping a SMALL task signal (amp-task-weight 0.1, ~10% of the freeprog stack) preserve command-following pressure without re-burying the AMP style gradient the way every 0.5/0.5 style arm did? Read JOINTLY with styleonly-v2 (task_weight=0): if styleonly climbs but this arm stays pinned, even 0.1x task charges re-bury the gradient; if both climb, this is the better base; if both stay pinned <=0.1, AMP-mechanism defect confirmed with n=2.

**gate**: Discovery (2M, DR-0 harness walk mode 6 det + 6 sto, own cfg, + W&B amp/*): INFORMATIVE-PASS = amp/style_reward_mean >=0.3 by run end with discriminator unsaturated; FULL PASS additionally needs median det fwd travel >= 0.10 m/15s with gait_valid >=4/6 det and six legs cycling on video. FAIL = style pinned <=0.1 (task charges re-bury the gradient even at 0.1x -- read against styleonly-v2 to locate the threshold). Judge within scope: mechanism-isolation arm, not a gait-quality claim.

