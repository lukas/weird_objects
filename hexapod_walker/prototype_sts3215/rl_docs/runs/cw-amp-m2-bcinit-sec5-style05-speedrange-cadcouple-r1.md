# cw-amp-m2-bcinit-sec5-style05-speedrange-cadcouple-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T23:04:35+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-speedrange-fastphase

**wandb_id**: su5kvtm9

**hypothesis**: Plain English: make the robot's internal step-timing clock run faster when a faster speed is commanded (and slower when slower), so it can actually follow different speed commands instead of walking at one fixed pace. Mechanism: new cfg goal.walk_phase_speed_scale=1.0 makes the walk_phase_obs clock rate proportional to commanded speed (hz_eff = 1.333*(s_ref/0.08), capped at 3.0 Hz), fixing the root cause both fastphase probes isolated: the old clock was speed-INDEPENDENT, so a 0.05-0.25 m/s command range compressed to ~0.10+/-0.02 realized regardless of clock rate or AMP style. Continues the speedrange checkpoint with AMP style kept at 0.5/0.5. (r1: first launch attempt name was blocked by its own refused-launch git tag.) Prediction-if-true: realized speed_mean spread widens well past the parent's 0.084-0.136 band (low commands slow down, high commands speed up; vel_err drops at band edges) with gait_valid preserved. Prediction-if-false: speed stays pinned despite command-tracking cadence => cap is stride/actuation or PPO budget, not timing. Strongest alternative: the discriminator (anchored to teacher_v2's ORIGINAL fixed-cadence clips) vetoes off-nominal cadences - visible as this arm staying pinned or degrading while the paired -nostyle twin widens; that outcome names motion-library cadence augmentation as the next tool. Bank: test_phase_speed_coupling.py 7/7 PASS, default-OFF bit-exact; snapshot exp/cadcouple-phase-clock.

**gate**: Discovery (2M, DR-0, judged jointly with -cadcouple-nostyle). INFORMATIVE-PASS = gait_valid >=5/6 det+sto, no new sacrificed legs/terminations, AND realized det speed_mean range widens meaningfully past the parent band (spread ratio max/min >= 2.0 across the sampled 0.05-0.25 commands, vs ~1.5 pinned today) with per-episode speed correlating with command. FAIL-collapse = terminations/statue/drag or paddle-creep on video even if speed widens (video-overrides-scalar). Style-veto branch = this arm pinned/degraded while -nostyle widens => next lever is cadence-augmenting the motion library, not the clock.

**verdict**: The speed-coupled phase clock partially works but trades gait QUALITY for speed variety -- a mixed result, not a clean pass. DR-0 gate: gait_valid stays 6/6 det+sto, zero terminations/sacrificed legs (mechanism doesn't break the gait outright). Realized speed_mean now spans 0.051-0.114 m/s (ratio 2.24x, clears the pre-registered >=2.0 widening bar vs the pinned ~1.5x parent band) -- the clock coupling DOES let the robot vary its pace with command, confirming the fastphase/nostyle root cause (fixed-rate clock) was correct. BUT slip roughly doubled (det med 4.40 vs speedrange's 2.69, sto 6.59 vs 3.85; worst single episodes hit 6-12/m) and progress collapsed on the harder episodes (det med 0.32, sto 0.25, vs ~0.6-0.7 before). Video (walk_det_1, walk_sto_5) shows WHY: on high-commanded-speed segments the legs cycle visibly FASTER but the body barely advances across the checkerboard -- a high-frequency march/shuffle, not a longer stride. This is the pre-registered 'widens but paddle-creeps' failure mode named in the gate text (video overrides scalar): the clock now tracks command, but nothing couples STRIDE LENGTH to the faster cadence, so faster stepping just wastes more foot-contact time as slip instead of covering more ground. hardware-ready: no (2M discovery, DR-0). Root cause chain: behavior (fast legs, same body speed) <- incentive (task reward rewards velocity-vs-command, which nominally should punish this, but the reward+style blend apparently still nets positive since quarters rose 48->180) <- pricing gap (no term penalizes wasted-motion/slip specifically, so a busier-but-not-faster gait is not actively disincentivized enough at this training budget) <- mechanism (only the CLOCK RATE was coupled to command; stride length/amplitude stayed fixed by construction). Next: couple stride length (or workspace amplitude) to commanded speed alongside the clock, or add an explicit slip-vs-command-speed penalty, before crediting this lever.

