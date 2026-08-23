# cw-walkcurr-pf-fwd6-rnd10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T22:24:03+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: mavw1x9t

**hypothesis**: Dose sibling of cw-walkcurr-pf-fwd6-rnd02: same RND-on-rscale50 mechanism, --rnd-coef 0.10 (5x) instead of 0.02, to bracket whether the intrinsic bonus needs to be a much bigger fraction of the crouch's own charge-avoidance income before it can disturb the static local optimum. If rnd02 already unfreezes, this checks whether more dose helps further or overshoots into flailing (the rung-0 swing9 dose-response precedent: MORE bonus was not strictly better there). If rnd02 freezes flat, this is the discriminating read: unfreezes -> RND works but needed more dose (adopt rnd10, keep searching the floor); also freezes -> RND-as-a-class is refuted at bank-legal reward scales here, next is BC-kickstart (track-rule-brushing last resort) or a much bigger dose/budget probe.

**gate**: Same rung-1 gate as rnd02: prog_ratio>0 + gait_valid>=4/6 det with visible forward travel, walk_freeprog_score crosses 0, clip_fraction stays healthy. Read jointly with rnd02.

**verdict**: RND at 5x dose (rnd_coef=0.10) ALSO does not unfreeze the rung-1 diet, and behaves almost identically to the 0.02 dose sibling -- gate FAIL. Evidence: own-cfg C-env det panel 0/6 gait_valid, all six legs sacrificed (duty<0.10) both det and sto, prog_ratio ~0.00, speed 0.007-0.008 m/s, slip/m 6.7-8.2; video shows the same frozen splayed pose across every sampled frame. W&B: clip_fraction stayed healthy (0.01-0.08, slightly higher variance than rnd02, consistent with 5x more exploration noise) and freeprog reached -0.005 by 2M (marginally better than rnd02's -0.007, but still no zero-crossing). KEY FINDING: rnd/intrinsic_mean and rnd/loss trajectories are NEARLY IDENTICAL between rnd02 and rnd10 (e.g. step 98304: 0.03664 vs 0.03664; step 2064384: 0.00552 vs 0.00565) -- the policy's visited-state distribution barely changed despite a 5x larger reward bonus, meaning the extra dose bought almost no additional exploration. Why: this is a dose-INSENSITIVE freeze, not a dose-starved one -- unlike the crush-fix rscale batch (where x0.1 vs x0.02 produced clearly different clip_fraction/freeprog trajectories), 0.02 vs 0.10 RND coefficient produced statistically indistinguishable behavior, so a small further dose bump is unlikely to help; the anti-exploration pull of the crouch's own charge-avoidance income appears to need either a MUCH larger bonus (order-of-magnitude, not 5x) or a fundamentally different mechanism. Aligned reward + adequate budget + no behavioral movement between two well-separated doses = a genuine, boring refutation, not a continue-longer case. Next: per track STATUS's pre-registered order, either fund one order-of-magnitude-bigger RND dose (~1.0, comparable to the FULL per-step task reward scale rather than the small walk charges) as the last untried point before declaring RND-as-a-class refuted at this budget, or move to the last-resort BC-kickstart (track-rule-brushing item (d)). Read jointly with rnd02 (this cycle) and the rung-0-diet+RND twin (cw-walkcurr-pf-rung0-swing3-rnd1/rnd3, a concurrent cycle's arms).

