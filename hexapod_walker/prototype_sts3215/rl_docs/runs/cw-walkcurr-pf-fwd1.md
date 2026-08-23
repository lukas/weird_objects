# cw-walkcurr-pf-fwd1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T16:11:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: fmbwu9p1

**hypothesis**: Can a prior-free from-scratch PPO policy (no gait clock, no BC teacher, no motion prior) learn plain forward walking when the reward diet is walking and nothing else? Kawawa-Beaudan 2022 recipe re-run with the multi-goal reward-carry failure of cw-kawawa2022-pf-flat1 designed out: walk-pure episode diet, fixed forward command 0.05-0.06 m/s, ELU 128/64/32 MLP with 24-step rollouts, loaded actuator calibration, term_penalty closing the suicide exploit, and the WALKCURR_PF semantics bank proving the reward ranking before launch. Prediction-if-true: real six-leg stepping with along-command progress emerges within 2M steps (the walk goal of the failed run survived its 1M/5M evals even on a 20-30%% walk diet). Prediction-if-false: statue/park or tilt terminations with flat-or-falling walk eval — which per the binding walkcurr triage rule stops same-recipe seeds and forces a reward/eval/simulator audit, not a continuation.

**gate**: Discovery gate at 2M: C-env deterministic fixed-forward panel (n>=6) — walk SURVIVES (zero tilt terminations), along-command progress cmd_prog_frac >= 0.35, direction_err_deg <= 30, slip/m <= 3.0, all six legs cycling contact/swing on >=4/6 episodes, and video showing real stepping (not tapping/skating/parking). BINDING TRIAGE RULE: read reward trend AND walk-eval trend together; reward rising with walk eval flat/down or terminating = misaligned -> STOP same-recipe seeds/continuations and audit reward/eval/sim. Reward AND eval both rising at 2M with the gate not yet met = continue per the 08-21 ruling.

