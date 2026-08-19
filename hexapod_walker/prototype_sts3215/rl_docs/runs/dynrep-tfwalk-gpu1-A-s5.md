# dynrep-tfwalk-gpu1-A-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-15T22:53:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 1000000

**git_sha**: 745816a059ddfbf62d5b3bae2aec3b192089b426

**wandb_id**: h9yy9fll

**hypothesis**: Scratch-baseline arm of the operator-ordered Transformer walking transfer test: PPO learns commanded-velocity walking from raw stacked obs with no pretrained encoder — the null hypothesis that the dynrep representation gives no sample-efficiency or gait-quality win. GPU-only relaunch (CUDA-required trainer c4f5b211) of the aborted CPU cohort.

**gate**: Judged as a matched triple with dynrep-tfwalk-gpu1-B-s5/C-s5 at 1M steps: walk return/steps-to-threshold and gait-quality columns (slip_m, peak_roll_deg, slew_sat) vs B and C.

**verdict**: LEDGER-HYGIENE CLOSE (08-19 idle-drain cycle), no fresh evaluation of this run: cohort finished clean at 1M (operator corrected stale-RUNNING to FINISHED 08-16, order 20260816T042655Z) and was superseded by the operator-ordered 2M from-scratch rerun dynrep-tfwalk-metrics1-A/B/C-s5, which carries the per-condition verdicts. This cohort's results are absorbed into the line-level conclusion (dynrep/STATUS 08-17: gpu1, metrics1, joint1 all agree -- the pretrained dynamics transformer does not transfer to walking under any tried mechanism; scratch PPO stays best). Recorded only to clear a stale unverdicted entry from the drain queue; no new science claimed.

**note**: Trainer exited cleanly at its 1M-step budget (POD_TFWALK_DONE in pod log, checkpoint ppo_dynrep-tfwalk-gpu1-A-s5.zip on pod); ledger row was stale-RUNNING. Corrected to FINISHED per operator order 20260816T042655Z. Learning-rate question (is C still learning at 1M?) superseded by the 2M dynrep-tfwalk-metrics1 cohort.

