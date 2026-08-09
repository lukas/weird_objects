# probe-stance-lowerdense-b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T05:43:18+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: rl_move/sim/policies/ppo_goal_cw_stance_bellyrest.zip

**hypothesis**: Probe smoke for reward.end_posture_lower_dense (new mechanism, audit §6): dense lower charging trains without tracebacks, reward_end_posture logged from episode start on lower eps, canaries green at baseline, ep_rew in lineage band. Retry of probe-stance-lowerdense (parent ckpt missing on pod, now copied, md5 verified).

**gate**: no tracebacks; run completes 150k; env/reward_end_posture present; canary baseline green

**verdict**: SMOKE PASS: 150k/160s, 0 tracebacks, canary parent baseline 8/8 green + 4 groups protected + green at 150k; dense window mechanics verified controller-side (125/125 ticks charged vs 26/125 terminal, zero-action lower pays 0.0)

