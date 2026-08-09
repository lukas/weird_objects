# cw-walk-kgate

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T02:58:22+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowent_h15b_c1.zip

**wandb_id**: q1ip6y7k

**hypothesis**: Park basin is income-sustained: at 0.02-0.06 m/s commands the absolute-error kernel pays a parked robot 0.97-1.85/tick (measured 1.77 zero-action), so the park stays net-positive (+519/ep) despite k_park_duty. Progress-gating the kernel (reward.walk_kernel_prog_gate=1.0, snapshot 19e97f8; income x clip(along/s_ref,0,1)) flips park returns to ~-225/ep. ONE variable vs the c1 segment. If-true: same-seed 15s harness park 1/6 sto -> 0/12, det duty-skew churn converts, fwd+gv 12/12; env/reward_walk dips then recovers with walk_prog_factor -> 1. If-false: park persists ~1/6 despite strongly negative return -> attractor-mechanical (t=0 commitment), pricing refuted -> reset-state diversity (parked starts) or rung-2 load evenness, NOT a coefficient retry. Strongest alternative (rare-draw noise PPO barely sees) rejected in advance: 1/6 of sto walk eps is common under training sampling and reproduced across 3 segments at the same rate. Parent ppo_goal_cw_walk_lowent_h15b_c1.zip md5 ed71b6f4 (preferred warm-start per cycle 21: behavior within noise of champion h15b, std 1.485). Probe probe-walk-kgate PASSED (150k/68s lower pod, ep_rew ~1060 vs c1 1177 = income cut visible, KL 0.0197, no tracebacks).

**gate**: 15s DR0 harness 6eps/mode det AND sto: fwd >=0.40m 12/12 AND gait_valid 12/12 AND >=2 swings/leg AND 0 terminations AND no final-third frame degradation AND 15s det fwd mean >=0.50m (anti-crawl guard; c1 0.610). Retention: 5s det slip-per-meter <=1.8 (c1 1.50, champion 1.53; raw-slip clause retired, cycle 21 confound)

