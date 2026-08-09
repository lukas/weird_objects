# probe-stance-lowerdense

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T05:41:42+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: rl_move/sim/policies/ppo_goal_cw_stance_bellyrest.zip

**hypothesis**: Probe smoke for reward.end_posture_lower_dense (new mechanism, audit §6): dense lower charging trains without tracebacks, reward_end_posture logged from episode start on lower eps, canaries green at baseline, ep_rew in lineage band.

**gate**: no tracebacks; run completes 150k; env/reward_end_posture present; canary baseline green

**verdict**: launch failed: parent ckpt not present on smoke pod (FileNotFoundError); ckpt copied (md5 6212b44f) and retried as probe-stance-lowerdense-b

**failed_reason**: log not growing (2226 -> 2226 bytes)

