# dynrep-tfwalk-gpu1-B-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-15T22:53:23+00:00

**pod**: hexapod-mjx-train-7

**steps**: 1000000

**parent**: cw-dynrep-tf-state2-recovered1

**git_sha**: dfe6e78e9bd670e404fd6c51136d9a30f99d5c84

**wandb_id**: dg5oj5hs

**hypothesis**: Frozen-encoder arm of the operator-ordered Transformer walking transfer test: the first G1/G1.1+G3-passing Transformer dynrep encoder (cw-dynrep-tf-state2-recovered1) is frozen and PPO learns commanded-velocity walking through its latent z; tests whether the pretrained body-dynamics representation alone speeds/cleans learning vs scratch (A). GPU-only relaunch (CUDA-required trainer c4f5b211).

**gate**: Judged as a matched triple with dynrep-tfwalk-gpu1-A-s5/C-s5 at 1M steps: walk return/steps-to-threshold and gait-quality columns (slip_m, peak_roll_deg, slew_sat) vs A and C.

**verdict**: LEDGER-HYGIENE CLOSE (08-19 idle-drain cycle), no fresh evaluation of this run: cohort finished clean at 1M (operator corrected stale-RUNNING to FINISHED 08-16, order 20260816T042655Z) and was superseded by the operator-ordered 2M from-scratch rerun dynrep-tfwalk-metrics1-A/B/C-s5, which carries the per-condition verdicts. This cohort's results are absorbed into the line-level conclusion (dynrep/STATUS 08-17: gpu1, metrics1, joint1 all agree -- the pretrained dynamics transformer does not transfer to walking under any tried mechanism; scratch PPO stays best). Recorded only to clear a stale unverdicted entry from the drain queue; no new science claimed.

**note**: Trainer exited cleanly at its 1M-step budget (POD_TFWALK_DONE in pod log, checkpoint ppo_dynrep-tfwalk-gpu1-B-s5.zip on pod); ledger row was stale-RUNNING. Corrected to FINISHED per operator order 20260816T042655Z. Learning-rate question (is C still learning at 1M?) superseded by the 2M dynrep-tfwalk-metrics1 cohort.

