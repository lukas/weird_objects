# cw-amp-m3-turnpush1-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-23T01:05:28+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**hypothesis**: Plain English: M3 push-recovery has only ever been trained/tested on the yaw-blind headingsfull substrate (no turning). cw-amp-m2-turnclone-yawcmd0-r2 just showed a single checkpoint can combine full-circle heading translation AND turn-in-place (eval_yaw tip err 0.15/0.16, DR-0 walk gate gait_valid 6/6, slip med 2.24 -- as good as the non-turn lineage). Does that combined substrate still tolerate mid-episode shoves the same way the plain headingsfull-style05 substrate did in pushsmoke1 (topple rate falling 2-3x over 2M, gait_valid staying high, no crouch-statue), or does the added yaw-command channel make push recovery harder (more directions to reacquire after a shove)? Single lever vs cw-amp-m3-pushsmoke1-style05: --init-from swapped from the plain style05-headingsfull checkpoint to the turn-capable cw-amp-m2-turnclone-yawcmd0-r2 checkpoint; identical push cfg (dr.ext_push_prob=1.0, 10-25N/0.15-0.4s/random direction once per episode), identical amp 0.5/0.5, 2M discovery.

**gate**: Discovery (2M, DR-0). Compare against pushsmoke1-style05's own shape (training tilt terminations falling toward ~15 pitch/~7 roll by end of window; DR-0 gate topples ~1/6 det + ~3/6 sto with gait_valid >=5/6, no crouch-statue on video). Prediction-if-true (composes fine): terminations fall on a similar trajectory, gate topples comparable, turn tracking (eval_yaw tip err) stays within ~0.05 of the pre-push checkpoint's 0.15/0.16. Prediction-if-false: terminations stay high/flat (push recovery doesn't transfer to the turn-capable substrate without its own acquisition budget) or turn tracking collapses toward the park fingerprint (0.28-0.33) under the added push disturbance. Either way zero-crouch and video-checked before any claim.

**verdict**: Launch-config bug, zero training happened. The respec chain took pushsmoke1-style05's cfg (74-dim obs, no yaw command column) but swapped --init-from to the turn-capable cw-amp-m2-turnclone-yawcmd0-r2 checkpoint (75-dim obs, goal.walk_yaw_cmd=1 baked in); train_ppo_mjx hard-failed at model load: 'Observation spaces do not match: Box(75,) != Box(74,)' (confirmed in /tmp/train_cw-amp-m3-turnpush1-style05.log on train-2). Not a research result -- no push/turn read is possible from this entry. Already diagnosed and retried by the launching session as cw-amp-m3-turnpush1-style05-r2 (same question + the matching yaw cfg block, obs-contract fix not a reward change), currently RUNNING on train-2 -- the turn+push composition question stays open pending r2.

