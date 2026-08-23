# cw-walkcurr-pf-fwd6-rnd10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-23T22:24:03+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: mavw1x9t

**hypothesis**: Dose sibling of cw-walkcurr-pf-fwd6-rnd02: same RND-on-rscale50 mechanism, --rnd-coef 0.10 (5x) instead of 0.02, to bracket whether the intrinsic bonus needs to be a much bigger fraction of the crouch's own charge-avoidance income before it can disturb the static local optimum. If rnd02 already unfreezes, this checks whether more dose helps further or overshoots into flailing (the rung-0 swing9 dose-response precedent: MORE bonus was not strictly better there). If rnd02 freezes flat, this is the discriminating read: unfreezes -> RND works but needed more dose (adopt rnd10, keep searching the floor); also freezes -> RND-as-a-class is refuted at bank-legal reward scales here, next is BC-kickstart (track-rule-brushing last resort) or a much bigger dose/budget probe.

**gate**: Same rung-1 gate as rnd02: prog_ratio>0 + gait_valid>=4/6 det with visible forward travel, walk_freeprog_score crosses 0, clip_fraction stays healthy. Read jointly with rnd02.

