# cw-walkcurr-pf-fwd6-actbias1-parkstart-p50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T03:17:52+00:00

**pod**: hexapod-mjx-train-7

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1

**wandb_id**: m5a690ik

**hypothesis**: Plain English: dose companion to -parkstart-p25 (same cycle, same rationale: does densifying reset states with tripod-lifted 'park' postures let PPO discover an exit gradient out of the static park-stand attractor?) at a stronger dose -- HALF of all walk episodes now start park-lifted instead of a quarter. Single lever vs actbias1: goal.walk_park_start_frac=0.5. Smoke-verified this cycle against this exact cfg stack: draw rate 31/60 (matches 0.5), park draws show real 2-11deg post-settle hip asymmetry vs 0.0deg at frac=0, stable 50-step zero-action rollout. Prediction-if-true: freeprog crosses zero and/or shows a stronger/faster effect than p25 (dose-response, more of training spent already displaced from the exact static optimum). Prediction-if-false: identical static park-stand at both doses -> reset-state-diversity is dose-insensitive and fully exonerated, strengthening the case that BC-kickstart is the only remaining lever (q_20260824T0233Z).

**gate**: Rung-1 gate: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: env/walk_freeprog_score trend vs the [-0.10,-0.05] dead band; env/height_err_mm stays in actbias1's healthy ~15-22mm band; clip_fraction stays healthy. Read jointly with -parkstart-p25 for the dose-response shape.

