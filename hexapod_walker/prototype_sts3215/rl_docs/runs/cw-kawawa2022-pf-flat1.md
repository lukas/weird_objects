# cw-kawawa2022-pf-flat1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T05:56:00+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40009728

**wandb_id**: m9gqkl5i

**hypothesis**: Kawawa-Beaudan 2022 prior-free flat walker adapted to MJX: from-scratch position-target PPO, LSTM(64)+ELU 128/64/32 heads, 24-step rollouts, loaded actuator calibration, no gait clock/BC teacher, only friction+push DR. Gate (per run notes): deterministic forward panel, zero falls, gait_valid 6/6, cmd_prog_frac>=0.65, slip/m<=2.5, height_factor>=0.8, no sacrificed legs, video of real stepping.

**verdict**: FAIL — the prior-free from-scratch recipe (LSTM(64)+ELU MLP, position-target PPO, no gait clock/BC teacher) never produces a stable walk. Evidence: pod output.log's periodic video reel prints an explicit per-task outcome every ~2M steps; walk shows TERM(tilt_pitch) at EVERY check from step 8.0M through the FINAL 40.0M-step checkpoint (32M of 40M steps, 80% of budget) — confirmed on video (rollout_477, near-final): the walk clip is frozen/planted for ~1-2s then the reel cuts away, matching the fall. W&B scalars agree: eval/walk/survived_frac=1 only at the two earliest checks (1M/5M) then pins at 0 from 9M onward and stays 0 at 40M; eval/walk/direction_err_deg never drops below ~38deg and is WORSE at the end (70.2deg final) than mid-run (~40deg at 20M); wrong_direction_frac rises from ~0 (12-25M) to 0.177 at 40M. Meanwhile rollout/ep_rew_mean rose then plateaued last three quarters (-336/174/236/238) — the plateau is explained by hold/raise/track/unload sub-goals scoring well (raise_success=1, hold_total_reward=352) and CARRYING the aggregate multi-task reward while the walk sub-goal stayed broken the whole time; walk_total_reward SCORE was only 54.6 vs 351.9(hold)/980.4(raise). Per the 08-21 ruling this is a genuine FAIL, not undertrained: the walk-specific task metric is flat-to-worsening for 80% of an adequate (40M-step) budget while the walk task never once recovers in the logged eval reel, including the final checkpoint. Why: straight-line-only heading (walk_heading_max_rad=0) plus a low-weight multi-goal mix let the policy find a hold/raise/track-dominant local optimum that ignores stepping; no gait-clock/BC prior + a comparatively weak k_walk_freeprog/k_walk_heading pricing next to the always-active hold/track/unload terms plausibly starves walk of gradient signal once tilt_pitch termination starts eating the walk episodes (8M+). What's next: NONE from this cycle — this is an out-of-scope, non-tracks.json run (kawawa2022 recipe/launch never landed in the repo per q_20260823T0450Z; ledger entry here is a post-hoc honest-triage backfill from W&B+pod log only). No agent-side follow-up, relaunch, or reward fix is queued; if the operator wants this pursued it needs its own registered arm (walk-only goal mix or higher walk pricing) plus a push of the missing recipe code.

**note**: Out-of-scope, operator/Codex-desktop-initiated (not in tracks.json); code+launch never reached this repo (q_20260823T0450Z) so this entry is a post-hoc honest-triage backfill reconstructed entirely from W&B m9gqkl5i + pod hexapod-mjx-train-1 output.log (no controller-side launch record exists).

