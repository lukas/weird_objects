# dynrep-tfwalk-metrics1-A-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-16T04:34:53+00:00

**pod**: hexapod-mjx-train-8

**steps**: 2000000

**git_sha**: b823cc76160234c6bb4cd1176d81a5ec785ffe38

**wandb_id**: jf0tfsqh

**hypothesis**: Scratch-control arm of the Transformer walking transfer rerun: PPO learns commanded-velocity walking from raw stacked obs with no pretrained encoder. Rerun of the gpu1 cohort at 2M steps (double the prior cutoff) with the full transfer-v2 rollout/SCORE metrics contract, so learning-curve shape past 1M is measurable.

**gate**: Judged as a matched triple at 2M steps with full rollout metrics: (1) walk return / steps-to-threshold and gait-quality columns (slip_m, peak_roll_deg, slew_sat) A vs B vs C; (2) the explicit 2M-extension question - is C still improving past the prior 1M cutoff (rollout/reward_per_transition_ema and SCORE/walk_total_reward slope over the 1M-2M half)?; (3) rise/hold retention + heldout dynamics suites.

**note**: Script-owned cohort (pod_tfwalk.sh, manifest tfwalk-metrics1_manifest.jsonl on-pod). Fresh append-only 2M-step rerun of the tfwalk-gpu1 triple per operator order 20260816T042655Z: metrics contract transfer-v2 (rollout/* + SCORE/* wired in b823cc76), new PPO from scratch (no init from any gpu1 checkpoint), schema smoke on arm A mechanically verified before B/C launch.

