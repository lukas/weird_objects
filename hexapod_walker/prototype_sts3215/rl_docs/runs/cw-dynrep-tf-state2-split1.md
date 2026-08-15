# cw-dynrep-tf-state2-split1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-15T19:00:35+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000

**parent**: cw-dynrep-tf-state2-fresh

**hypothesis**: A full 13.62M-parameter causal Transformer trained on at least 10.24M distinct train windows will generalize current and future servo state, velocity, heading, current, and foot contact when checkpoint selection uses a disjoint validation set and final quality is measured once on an untouched test set.

**gate**: Collect at least 10.24M train windows under stable whole-episode 80/10/10 splitting with actor, DR, and mode coverage in validation and test; planned reuse must be <=2x. Train on CUDA with hidden=512,z=256,4 layers,8 heads,ff=1024. W&B must show train_eval/val generalization gaps and no sustained overfit alarm; the selected best-validation checkpoint must report final test physical and contact metrics.

**refused_reason**: Cancelled before process launch: canonical corrected dynrep-fresh pipeline already verified running as cw-dynrep-tf-state2-fresh2 on train-10 (W&B flaf42k7); respec incorrectly inherited trainer=ppo and would have duplicated work.

