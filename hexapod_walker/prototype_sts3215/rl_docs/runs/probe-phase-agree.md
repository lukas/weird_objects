# probe-phase-agree

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T21:09:41+00:00

**pod**: hexapod-sweep-lower

**steps**: 100000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**checkpoint**: pod lower: rl_move/sim/policies/probe_phase_agree_19281216_steps.zip

**hypothesis**: Audit sec6 probe for the phase-contact mechanism: with audited exploration (std 1.0, ent 0.01) at DR 0, 100k steps from stance init must show the phase reward DRIVING behavior (phase-consistent foot lifts appearing; agreement moving off the 0.5 all-feet-down baseline). Fails -> mechanism bugged or unlearnable, do not burn the 4M arm.

**gate**: probe: post-train rollout shows swing activity on both tripods synchronized to the clock vs stance parent (~0 swings); informative either way

**verdict**: PROBE PASS (audit sec6, cycle 11c): 96k-step ckpt at DR 0 shows ALL SIX legs cycling - det swings 6-21/leg (stance parent ~0, shuffle lineage parks leg 3 at ~1), duties 0.24-0.95, det gait_valid 2/3, speed ~0.023. Phase-contact mechanism is learnable and drives phase-synchronized stepping in <100k steps. Note: launcher verifier killed the trainer at ~110k because the run FINISHED its training loop inside the verification window (log stopped growing) - lesson: sub-2-min smokes will always fail verification; use periodic ckpt. Not a pod fault (oom_kill 0).

**failed_reason**: log not growing (8318 -> 8318 bytes)

