# cw-walkcurr-pf-fwd6-actbias1-parkstart-p50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T03:17:52+00:00

**pod**: hexapod-mjx-train-7

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-actbias1

**wandb_id**: m5a690ik

**hypothesis**: Plain English: dose companion to -parkstart-p25 (same cycle, same rationale: does densifying reset states with tripod-lifted 'park' postures let PPO discover an exit gradient out of the static park-stand attractor?) at a stronger dose -- HALF of all walk episodes now start park-lifted instead of a quarter. Single lever vs actbias1: goal.walk_park_start_frac=0.5. Smoke-verified this cycle against this exact cfg stack: draw rate 31/60 (matches 0.5), park draws show real 2-11deg post-settle hip asymmetry vs 0.0deg at frac=0, stable 50-step zero-action rollout. Prediction-if-true: freeprog crosses zero and/or shows a stronger/faster effect than p25 (dose-response, more of training spent already displaced from the exact static optimum). Prediction-if-false: identical static park-stand at both doses -> reset-state-diversity is dose-insensitive and fully exonerated, strengthening the case that BC-kickstart is the only remaining lever (q_20260824T0233Z).

**gate**: Rung-1 gate: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: env/walk_freeprog_score trend vs the [-0.10,-0.05] dead band; env/height_err_mm stays in actbias1's healthy ~15-22mm band; clip_fraction stays healthy. Read jointly with -parkstart-p25 for the dose-response shape.

**verdict**: Doubling park-start reset-diversity dose to 0.5 (half of all walk episodes spawn tripod-lifted) does NOT unlock walking either -- confirms -p25's finding, closing the dose axis. Evidence: det gate 0/6 gait_valid (sac legs vary per episode: [4], [0,3,4], [4,5] -- no consistent tripod lock this time, but still zero net travel), prog_ratio med 0.01, fwd med 0.03m/15s, slip/m med 3.52 (low -- not skating, just not moving), zero terminations; sto gait_valid 6/6 but slip/m med 43.96 (heavy in-place thrashing, prog ~0.00-0.04, dir_err 87.5deg -- noise-driven jitter, not walking). W&B history: env/walk_freeprog_score bumps from -0.084 to a mid-run best of -0.056 (step ~211-239) then REGRESSES to -0.071 by the end -- same shape as actbias1-idleterm1 and -parkstart-p25, never crossing zero. rollout/ep_len_mean climbs steadily 16->496 (near the ~500-step max) over the run: the policy is learning to recover FROM the perturbed park starts back INTO a stable stand, not to walk -- the exit gradient this mechanism was designed to create genuinely exists (video shows more varied/less rigidly-tripod-locked leg use than plain actbias1), but the economics still price re-standing above stepping, exactly as -p25 found. clip_fraction stayed healthy (0.002->0.08, never collapsed) and reward fell every quarter (45.3/42.5/31.1/16.6) -- an ALIGNED FAIL per the 08-21 ruling, not misalignment or undertraining. height_err_mm ran a bit higher than plain actbias1 (23-30mm vs 14-22mm) but never collapsed toward the belly-sit band -- the action-bias fix held. Video (6 det + 6 sto contact sheets/mp4s watched) confirms: robot recovers to level/upright and stays there, no net body translation across any clip. DOSE READ: 0.25 and 0.5 fail IDENTICALLY (both aligned, both show the rise-to-near-max ep_len recovery signature, freeprog bump-then-regress in the same band) -- park_start_frac is dose-insensitive in this range, closing goal.walk_park_start_frac as a rung-1 discovery mechanism at both tested doses. This was the last credible non-BC, non-reward-pricing, non-architecture lever this campaign could name (see OPERATOR_QUESTIONS.md q_20260824T0233Z): with idle-termination (2 configs), reset-state diversity (2 doses), and 9 earlier reward/architecture classes ALL closed with aligned FAILs at 2M, from-scratch discovery of forward walking is refuted at this budget/architecture for every non-BC mechanism this track has been able to invent. Updating OPERATOR_QUESTIONS.md q_20260824T0233Z to make the BC-kickstart ruling fully live (no further non-BC lever pre-registered or credible) rather than advisory.

