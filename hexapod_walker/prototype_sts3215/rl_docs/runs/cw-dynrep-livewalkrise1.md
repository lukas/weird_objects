# cw-dynrep-livewalkrise1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-17T21:50:11+00:00

**pod**: hexapod-mjx-train-5

**steps**: 10000000

**git_sha**: 3271920ca1e15b8d2e1f5e07ed057c916b0d2d5e

**wandb_id**: 1d1ro5dc

**hardware_ready**: False

**hypothesis**: Let the world model keep learning from what the robot is actually doing — walking in every direction with stops, turns and direction changes, and standing up from flat, crouched and post-sit poses — while the actor stays a plain raw-observation policy and the critic reads dynamics features from a snapshot that may only improve at 1M-step boundaries behind retention gates. Tests whether live-adapted dynamics features beat the static frozen-D transfer (the current best) without re-triggering the nonstationary-critic failure that sank condition E. Operator order fb_20260817T210422_9df9c7 arm A.

**gate**: Pre-registered 10M decision checkpoint, no extension without a verdict. Mechanism invariants throughout: actor KL exactly 0; snapshot version changes only at 1M boundaries with all six gate values logged; every accepted update must NOT be followed by walk-return or live-val regression over the next 1M (the E failure signature — if it is, record LIVE-ADAPTATION HARMFUL and stop). Outcome read at 10M: (a) if >=1 snapshot update was accepted, compare walk return + gait quality (slip_m, peak_roll_deg, slew_sat) and rise/walkrise SCORE trends before/after accepts, and against the frozen-D yardstick cw-dynrep-criticD-40m1 at matched step counts (note confounds: n_envs 8 vs 16, yaw-cmd obs); (b) if NO update ever passes the gates by 10M, record that live adaptation cannot beat frozen features under honest retention gates on this data and close the online line for good. Retain best-by-heldout + periodic checkpoints; verdict quotes visual-quality stats, not scalar reward alone.

**verdict**: CLOSE (pre-registered gate clause b, live vs frozen critic features): across all 10 boundary-gated snapshot attempts (every 1M steps to 10M) the value-jump retention gate rejected every candidate by ~5-7x its threshold (observed dv 30.7-43.3 vs gate 0.10*(|raw_value|+1)~6.1; corpus-val 2.44-2.53 and live walk/rise candidates were themselves in-band -- only value_jump failed, every single time). snapshot_version stayed pinned at 0 for the whole run, so the actor+critic ran on exactly frozen condition-D features throughout; the online predictor trained the entire 10M for nothing the policy ever used. Final gait is clean (walk slip_per_m 0.22, peak_roll ~4.4 deg, mean_h 0.134m, zero early terms, walk return ~409-422) but that is D's behavior, not a live-adaptation result -- this run never actually tested its own hypothesis. Per the pre-registered gate text, this is exactly outcome (b): record that live adaptation cannot beat frozen features under honest retention gates on this data, and close the online-critic-adaptation line for good. No follow-up arm; SIM SPRINT also bans a new dynrep launch regardless.

**note**: Script-owned (pod_livewalkrise.sh). Operator order fb_20260817T210422_9df9c7 arm A continuation: 10M steps, snapshot boundary 1M (up to ~9 gated update attempts), stratified live CUDA replay 75/25 walk/rise (75% fresh + 25% v5 rehearsal), command-rich env (uniform heading, resample 4s + jitter, 15% stops, commanded yaw, rise flat/bridge/crouch + 30% post-lower bank), exogenous cmd priv channels masked on live windows (canary1 finding, 3271920c). Actor raw-obs scratch; 13.62M transformer capacity unchanged; starts exact frozen D.

