# cw-arch-tf-joymodes-scratch1-fallfix1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T19:39:02+00:00

**pod**: hexapod-mjx-train-3

**steps**: 4000000

**parent**: cw-arch-tf-joymodes-scratch1-acq1

**wandb_id**: kt8w93ra

**hypothesis**: Make the joystick robot stop falling over while it follows direction commands: this continuation restarts the stopped from-scratch Transformer run from its 34M-step checkpoint with exactly one change — a horizon-scaled fall charge (reward.term_cost_per_remaining_s=12.0), so a tilt fall at the current ~3.25 s failure time now costs ~-141 instead of the flat -10 that let short command-following bursts followed by a fall still pay ~145/episode. Operator-ordered (fb_20260815T192912_15af2f): direction acquisition was working (v-err 0.045->0.016 m/s, wrong-way 26%->5%) but eval walk survival stayed 0 through 28M with ~607 tilt terminations per rollout; if falling was underpriced rather than unavoidable, survival should climb while direction-following is retained. Budget = the ~4M steps remaining of the lineage's originally promised 40M total.

**gate**: PRIMARY readout is survival/fall count, not reward or direction error. PASS = by the final 1M-cadence evals, eval/walk/survived_frac moves materially off 0 (>=0.25) AND training tilt terminations per rollout drop >=50% vs the ~607 baseline, with directional metrics not regressing beyond noise (requested-direction v-err <=0.02 m/s, wrong-way <=0.10). FAIL = survival still ~0 at 40M lineage total despite the ~-141 fall charge — then the diagnosis shifts from pricing to capability/curriculum and this unchanged lineage gets no further chunks. Rising reward alone is not a safety verdict.

