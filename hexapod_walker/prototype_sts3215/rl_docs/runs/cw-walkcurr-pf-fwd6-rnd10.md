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

**verdict**: FAIL, jointly with rnd02 (fired the pre-registered joint gate). Result: RND-coef=0.10 (5x rnd02) also does NOT unfreeze rung-1 -- same static splayed crouch. Evidence: gate 0/6 det + 0/6 sto (all 6 legs sacrificed every episode), prog med -0.00, fwd 0.01m, slip 6.73-9.46; contact sheet identical frozen pose across all 12 frames. W&B: env/walk_freeprog_score rose -0.101->-0.0054 (closer to 0 than rnd02 but still never crossed), clip_fraction stayed healthy the whole run (0.012-0.072, never collapsed), std 0.368->0.375 -- but env/walk_speed DECAYED 0.10->0.0058 m/s (same direction as rnd02, worse endpoint), direction_err flat ~88-92deg (chance) the whole run, rnd/intrinsic_mean decayed 0.037->0.0057 -- essentially IDENTICAL trajectory to rnd02 despite 5x the coefficient (raw reward_rnd_intrinsic ~0.06/step at coef 0.1, already comparable to the recipe's other small charges, yet no qualitative change vs 0.02). ep_rew_mean quarters roughly flat-then-down (54.7/62.6/62.3/53.5, driven mostly by the larger intrinsic payment, not task behavior). Why: a 5x dose bracket made no behavioral difference -- the mechanism (intrinsic bonus decaying as fast as the policy narrows onto the frozen pose) is dose-insensitive in this range, not dose-starved. Next: closes the RND dose bracket at 0.02/0.10. This cycle launched two follow-ups to close the remaining axes before the track's BC-kickstart last resort: cw-walkcurr-pf-fwd6-rnd100 (10x further, coef=1.0, makes RND the dominant per-step term) and cw-walkcurr-pf-fwd6-rnd10-cont1 (+4M budget continuation at the same coef=0.10, testing whether the still-rising-but-not-crossed freeprog trajectory eventually tips over with more steps).

